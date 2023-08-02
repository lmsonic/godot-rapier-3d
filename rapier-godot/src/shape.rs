#![allow(clippy::module_name_repetitions)]

use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::prelude::*;

use crate::collision_object::RapierCollisionObject;

pub trait RapierShape {
    fn rid(&self) -> Rid;
    fn get_data(&self) -> Variant;
    fn set_data(&mut self, data: Variant);
    fn get_shape(&self) -> SharedShape;
    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType;
    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>>;

    fn remove_from_owners(&self) {
        for owner in self.owners() {
            owner.borrow_mut().remove_shape_rid(self.rid());
        }
    }
}

pub struct RapierShapeInstance {
    pub shape: Rc<RefCell<dyn RapierShape>>,
    pub isometry: Isometry<f32>,
    pub disabled: bool,
}

impl RapierShapeInstance {
    pub fn new(
        shape: Rc<RefCell<dyn RapierShape>>,
        isometry: Isometry<f32>,
        disabled: bool,
    ) -> Self {
        Self {
            shape,
            isometry,
            disabled,
        }
    }
}

pub struct SeparationRayShape {
    shape: Segment,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
}

impl RapierShape for SeparationRayShape {
    fn rid(&self) -> Rid {
        self.rid
    }

    fn get_data(&self) -> Variant {
        let length = (self.shape.b - self.shape.a).norm();
        Variant::from(dict! {"length":length , "slide_on_slope" :false})
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to::<Dictionary>() {
            Ok(d) => {
                match d.get_or_nil("length").try_to::<f32>() {
                    Ok(length) => self.shape.b = length * point![0.0, 0.0, 1.0],
                    Err(e) => godot_error!("{:?}", e),
                };
                match d.get_or_nil("slide_on_slope").try_to::<bool>() {
                    Ok(value) => {
                        if value {
                            godot_warn!(
                            "Godot Rapier doesn't support slide_on_slope for SeparationRayShape"
                        );
                        }
                    }
                    Err(e) => godot_error!("{:?}", e),
                };
            }
            Err(e) => godot_error!("{:?}", e),
        }
    }

    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape)
    }

    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_SEPARATION_RAY
    }

    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }
}

impl SeparationRayShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: Segment::new(point![0.0, 0.0, 0.0], point![0.0, 0.0, 0.5]),
            owners: vec![],
            rid,
        }
    }
}

pub struct WorldBoundaryShape {
    shape: HalfSpace,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
}

impl WorldBoundaryShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: HalfSpace::new(UnitVector::new_normalize(vector![0.0, 1.0, 0.0])),
            owners: vec![],
            rid,
        }
    }
}
impl RapierShape for WorldBoundaryShape {
    fn rid(&self) -> Rid {
        self.rid
    }

    fn get_data(&self) -> Variant {
        let normal = Vector3::new(
            self.shape.normal.x,
            self.shape.normal.y,
            self.shape.normal.z,
        );
        Variant::from(Plane::new(normal, 0.0))
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to::<Plane>() {
            Ok(plane) => {
                let normal = vector![plane.normal.x, plane.normal.y, plane.normal.z];
                self.shape = HalfSpace::new(UnitVector::new_normalize(normal));
                if plane.d != 0.0 {
                    godot_warn!(
                        "Godot Rapier doesn't support WorldBoundaryShapes not set in the origin"
                    );
                }
            }
            Err(err) => godot_error!("{:?}", err),
        }
    }

    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape)
    }

    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_WORLD_BOUNDARY
    }

    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }
}

pub struct RapierSphereShape {
    shape: Ball,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
}

impl RapierSphereShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: Ball::new(0.5),
            owners: vec![],
            rid,
        }
    }
}

impl RapierShape for RapierSphereShape {
    fn get_data(&self) -> Variant {
        Variant::from(self.shape.radius)
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to() {
            Ok(radius) => {
                self.shape.radius = radius;
            }
            Err(err) => godot_error!("{:?}", err),
        };
    }

    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape)
    }

    fn rid(&self) -> Rid {
        self.rid
    }

    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }

    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_SPHERE
    }
}

pub struct RapierBoxShape {
    shape: Cuboid,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
}

impl RapierBoxShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: Cuboid::new(vector![0.5, 0.5, 0.5]),
            owners: vec![],
            rid,
        }
    }
}

impl RapierShape for RapierBoxShape {
    fn get_data(&self) -> Variant {
        Variant::from(Vector3::new(
            self.shape.half_extents.x,
            self.shape.half_extents.y,
            self.shape.half_extents.z,
        ))
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to::<Vector3>() {
            Ok(half_extents) => {
                self.shape.half_extents = vector![half_extents.x, half_extents.y, half_extents.z];
            }
            Err(err) => godot_error!("{:?}", err),
        };
    }
    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape)
    }

    fn rid(&self) -> Rid {
        self.rid
    }
    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }
    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_BOX
    }
}

pub struct RapierCapsuleShape {
    shape: Capsule,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
}

impl RapierCapsuleShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: Capsule::new_y(0.5, 0.2),
            owners: vec![],
            rid,
        }
    }
}

impl RapierShape for RapierCapsuleShape {
    fn get_data(&self) -> Variant {
        Variant::from(dict! {"radius": self.shape.radius,"height":self.shape.height()})
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to::<Dictionary>() {
            Ok(d) => {
                match d.get_or_nil("radius").try_to() {
                    Ok(radius) => self.shape.radius = radius,
                    Err(e) => godot_error!("{:?}", e),
                };
                match d.get_or_nil("height").try_to::<f32>() {
                    Ok(height) => {
                        self.shape.segment.b = self.shape.segment.a + Vector::y() * height;
                    }
                    Err(e) => godot_error!("{:?}", e),
                };
            }
            Err(e) => godot_error!("{:?}", e),
        };
    }
    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape)
    }

    fn rid(&self) -> Rid {
        self.rid
    }
    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }
    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_CAPSULE
    }
}

pub struct RapierCylinderShape {
    shape: Cylinder,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
}

impl RapierCylinderShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: Cylinder::new(0.5, 0.2),
            owners: vec![],
            rid,
        }
    }
}

impl RapierShape for RapierCylinderShape {
    fn get_data(&self) -> Variant {
        Variant::from(dict! {"radius": self.shape.radius,"height":self.shape.half_height*2.0})
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to::<Dictionary>() {
            Ok(d) => {
                match d.get_or_nil("radius").try_to() {
                    Ok(radius) => self.shape.radius = radius,
                    Err(e) => godot_error!("{:?}", e),
                };
                match d.get_or_nil("height").try_to::<f32>() {
                    Ok(height) => {
                        self.shape.half_height = height * 0.5;
                    }
                    Err(e) => godot_error!("{:?}", e),
                };
            }
            Err(e) => godot_error!("{:?}", e),
        };
    }
    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape)
    }

    fn rid(&self) -> Rid {
        self.rid
    }
    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }
    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_CYLINDER
    }
}
