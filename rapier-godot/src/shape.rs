use godot::prelude::{math::ApproxEq, *};
use rapier3d::prelude::*;

type GodotShapeType = godot::engine::physics_server_3d::ShapeType;
const DEFAULT_SOLVER_BIAS: f32 = 0.0;

pub trait Shape {
    fn get_rid(&self) -> Rid;
    fn set_rid(&mut self, rid: Rid);
    fn get_data(&self) -> Variant;
    fn set_data(&mut self, data: Variant);
    fn get_custom_solver_bias(&self) -> f32 {
        DEFAULT_SOLVER_BIAS
    }

    fn set_custom_solver_bias(&mut self, bias: f32) {
        if !bias.approx_eq(&DEFAULT_SOLVER_BIAS) {
            godot_warn!("Custom solver bias is not supported in Godot Rapier");
        }
    }

    fn get_margin(&self) -> f32 {
        0.0
    }

    fn set_margin(&mut self, _margin: f32) {}
    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType;
}

pub struct SphereShape {
    shape: Ball,
    rid: Rid,
}

impl SphereShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: Ball::new(0.5),
            rid,
        }
    }
}

impl Shape for SphereShape {
    fn get_rid(&self) -> Rid {
        self.rid
    }

    fn set_rid(&mut self, rid: Rid) {
        self.rid = rid;
    }

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

    fn get_type(&self) -> GodotShapeType {
        GodotShapeType::SHAPE_SPHERE
    }
}

pub struct BoxShape {
    shape: Cuboid,
    rid: Rid,
}

impl BoxShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: Cuboid::new(vector![0.5, 0.5, 0.5]),
            rid,
        }
    }
}

impl Shape for BoxShape {
    fn get_rid(&self) -> Rid {
        self.rid
    }

    fn set_rid(&mut self, rid: Rid) {
        self.rid = rid;
    }

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

    fn get_type(&self) -> GodotShapeType {
        GodotShapeType::SHAPE_BOX
    }
}

pub struct CapsuleShape {
    shape: Capsule,
    rid: Rid,
}

impl CapsuleShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: Capsule::new_y(0.5, 0.2),
            rid,
        }
    }
}

impl Shape for CapsuleShape {
    fn get_rid(&self) -> Rid {
        self.rid
    }

    fn set_rid(&mut self, rid: Rid) {
        self.rid = rid;
    }

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

    fn get_type(&self) -> GodotShapeType {
        GodotShapeType::SHAPE_CAPSULE
    }
}

pub struct CylinderShape {
    shape: Cylinder,
    rid: Rid,
}

impl CylinderShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: Cylinder::new(0.5, 0.2),
            rid,
        }
    }
}

impl Shape for CylinderShape {
    fn get_rid(&self) -> Rid {
        self.rid
    }

    fn set_rid(&mut self, rid: Rid) {
        self.rid = rid;
    }

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

    fn get_type(&self) -> GodotShapeType {
        GodotShapeType::SHAPE_CYLINDER
    }
}
