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
    fn get_margin(&self) -> f32;
    fn set_margin(&mut self, margin: f32);
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

    fn get_margin(&self) -> f32 {
        0.0
    }

    fn set_margin(&mut self, _margin: f32) {}

    fn get_type(&self) -> GodotShapeType {
        GodotShapeType::SHAPE_SPHERE
    }
}

pub struct BoxShape {
    shape: Cuboid,
    rid: Rid,
    half_extents: Vector3,
}

impl BoxShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: Cuboid::new(vector![0.5, 0.5, 0.5]),
            rid,
            half_extents: Vector3 {
                x: 0.5,
                y: 0.5,
                z: 0.5,
            },
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
        Variant::from(self.half_extents)
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to() {
            Ok(half_extents) => {
                self.half_extents = half_extents;
                self.shape.half_extents = vector![
                    self.half_extents.x,
                    self.half_extents.y,
                    self.half_extents.z,
                ];
            }
            Err(err) => godot_error!("{:?}", err),
        };
    }

    fn get_margin(&self) -> f32 {
        0.0
    }

    fn set_margin(&mut self, _margin: f32) {}

    fn get_type(&self) -> GodotShapeType {
        GodotShapeType::SHAPE_SPHERE
    }
}
