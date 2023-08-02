#![allow(clippy::module_name_repetitions)]

use std::{
    cell::RefCell,
    collections::{HashMap, HashSet},
    rc::Rc,
};

use godot::prelude::{math::ApproxEq, *};
use rapier3d::{
    na::{dmatrix, matrix},
    prelude::*,
};
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use crate::collision_object::RapierCollisionObject;

pub trait RapierShape {
    fn rid(&self) -> Rid;
    fn get_data(&self) -> Variant;
    fn set_data(&mut self, data: Variant);
    fn get_shape(&self) -> SharedShape;
    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType;
    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>>;
    fn update_owners(&self) {
        for owner in self.owners() {
            owner.borrow_mut().update_shapes();
        }
    }
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
                    Ok(length) => {
                        let cur_length = (self.shape.b - self.shape.a).norm();
                        if !length.approx_eq(&cur_length) {
                            self.shape.b = length * point![0.0, 0.0, 1.0];
                            self.update_owners();
                        }
                    }
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
                let normal = UnitVector::new_normalize(vector![
                    plane.normal.x,
                    plane.normal.y,
                    plane.normal.z
                ]);
                if normal != self.shape.normal {
                    self.shape = HalfSpace::new(normal);
                    self.update_owners();
                }
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
        match data.try_to::<f32>() {
            Ok(radius) => {
                if !radius.approx_eq(&self.shape.radius) {
                    self.shape.radius = radius;
                    self.update_owners();
                }
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
                let new_half_extents = vector![half_extents.x, half_extents.y, half_extents.z];
                if new_half_extents != self.shape.half_extents {
                    self.shape.half_extents = new_half_extents;
                    self.update_owners();
                }
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
                match d.get_or_nil("radius").try_to::<f32>() {
                    Ok(radius) => {
                        if !radius.approx_eq(&self.shape.radius) {
                            self.shape.radius = radius;
                            self.update_owners();
                        }
                    }
                    Err(e) => godot_error!("{:?}", e),
                };
                match d.get_or_nil("height").try_to::<f32>() {
                    Ok(height) => {
                        if !height.approx_eq(&self.shape.height()) {
                            self.shape.segment.b = self.shape.segment.a + Vector::y() * height;
                            self.update_owners();
                        }
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
                match d.get_or_nil("radius").try_to::<f32>() {
                    Ok(radius) => {
                        if !radius.approx_eq(&self.shape.radius) {
                            self.shape.radius = radius;
                            self.update_owners();
                        }
                    }
                    Err(e) => godot_error!("{:?}", e),
                };
                match d.get_or_nil("height").try_to::<f32>() {
                    Ok(height) => {
                        if !height.approx_eq(&(self.shape.half_height * 2.0)) {
                            self.shape.half_height = height * 0.5;
                            self.update_owners();
                        }
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

pub struct RapierConvexShape {
    shape: ConvexPolyhedron,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
}
impl RapierConvexShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: ConvexPolyhedron::from_convex_hull(&[
                point![1.0, 0.0, 0.0],
                point![0.0, 1.0, 0.0],
                point![0.0, 0.0, 1.0],
            ])
            .unwrap(),
            owners: vec![],
            rid,
        }
    }
}

impl RapierShape for RapierConvexShape {
    fn rid(&self) -> Rid {
        self.rid
    }

    fn get_data(&self) -> Variant {
        let points: Array<Vector3> = self
            .shape
            .points()
            .iter()
            .map(|v| Vector3::new(v.x, v.y, v.z))
            .collect();
        Variant::from(points)
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to::<Array<Vector3>>() {
            Ok(vertices) => {
                let points: Vec<Point<f32>> = vertices
                    .iter_shared()
                    .map(|v| point![v.x, v.y, v.z])
                    .collect();
                if self.shape.points() != points {
                    if let Some(convex) = ConvexPolyhedron::from_convex_hull(&points) {
                        self.shape = convex;
                        self.update_owners();
                    }
                }
            }
            Err(e) => godot_error!("{:?}", e),
        }
    }

    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape.clone())
    }

    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_CONVEX_POLYGON
    }

    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }
}

pub struct RapierConcaveShape {
    shape: TriMesh,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
    backface_collision: bool,
}
impl RapierConcaveShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: TriMesh::new(
                vec![
                    point![0.0, 0.0, 0.0],
                    point![0.0, 0.0, 0.0],
                    point![0.0, 0.0, 0.0],
                ],
                vec![[0, 1, 2]],
            ),
            owners: vec![],
            rid,
            backface_collision: false,
        }
    }

    pub fn get_compound_convex_shapes(&self) -> SharedShape {
        SharedShape::convex_decomposition(self.shape.vertices(), self.shape.indices())
    }
}

fn generate_indices(
    input: &Vec<Point<f32>>,
    backface_collision: bool,
) -> (Vec<Point<f32>>, Vec<[u32; 3]>) {
    let mut vertices: Vec<Point<f32>> = Vec::with_capacity(input.len());
    let mut indices = Vec::with_capacity(input.len());
    for face in input.chunks_exact(3) {
        let mut index_triplet = [0, 0, 0];

        // FRONT FACE CW
        for i in 0..3 {
            if let Some(index) = vertices.iter().position(|v| {
                v.x.approx_eq(&face[i].x) && v.y.approx_eq(&face[i].y) && v.z.approx_eq(&face[i].z)
            }) {
                index_triplet[i] = index as u32;
            } else {
                vertices.push(face[i]);
                index_triplet[i] = (vertices.len() - 1) as u32;
            }
        }
        // BACKFACE
        if backface_collision {
            indices.push([index_triplet[2], index_triplet[1], index_triplet[0]]);
        }
        indices.push(index_triplet);
    }
    (vertices, indices)
}

impl RapierShape for RapierConcaveShape {
    fn rid(&self) -> Rid {
        self.rid
    }

    fn get_data(&self) -> Variant {
        let faces = self
            .shape
            .vertices()
            .iter()
            .map(|v| Vector3::new(v.x, v.y, v.z))
            .collect::<Array<Vector3>>();

        Variant::from(dict! {
            "faces" : faces, "backface_collision":self.backface_collision
        })
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to::<Dictionary>() {
            Ok(d) => {
                match d.get_or_nil("backface_collision").try_to::<bool>() {
                    Ok(value) => {
                        if value != self.backface_collision {
                            self.backface_collision = value;
                            let (vertices, indices) =
                                generate_indices(self.shape.vertices(), self.backface_collision);
                            self.shape = TriMesh::new(vertices, indices);
                            self.update_owners();
                        }
                    }
                    Err(e) => godot_error!("{:?}", e),
                }
                match d.get_or_nil("faces").try_to::<PackedVector3Array>() {
                    Ok(vertices) => {
                        let vertices: Vec<Point<f32>> = vertices
                            .to_vec()
                            .iter()
                            .map(|v| point![v.x, v.y, v.z])
                            .collect();
                        if vertices != *self.shape.vertices() {
                            let (vertices, indices) =
                                generate_indices(&vertices, self.backface_collision);
                            self.shape = TriMesh::new(vertices, indices);
                            self.update_owners();
                        }
                    }
                    Err(e) => godot_error!("{:?}", e),
                };
            }
            Err(e) => godot_error!("{:?}", e),
        }
    }

    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape.clone())
    }

    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_CONCAVE_POLYGON
    }

    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }
}

pub struct RapierHeightmapShape {
    shape: HeightField,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
}

impl RapierShape for RapierHeightmapShape {
    fn rid(&self) -> Rid {
        self.rid
    }

    fn get_data(&self) -> Variant {
        todo!()
    }

    fn set_data(&mut self, data: Variant) {
        todo!()
    }

    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape.clone())
    }

    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_HEIGHTMAP
    }

    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }
}

impl RapierHeightmapShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: HeightField::new(dmatrix![0.0,0.0;0.0,0.0], vector![1.0, 1.0, 1.0]),
            owners: vec![],
            rid,
        }
    }
}
