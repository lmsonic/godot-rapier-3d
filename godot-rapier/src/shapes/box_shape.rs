use std::{cell::RefCell, rc::Rc};

use godot::prelude::{math::ApproxEq, *};
use rapier3d::prelude::*;

use crate::collision_object::RapierCollisionObject;

use super::RapierShape;
pub struct RapierBoxShape {
    shape: Cuboid,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
    margin: f32,
}

impl RapierBoxShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: Cuboid::new(vector![0.5, 0.5, 0.5]),
            owners: vec![],
            rid,
            margin: 0.0,
        }
    }
}

impl RapierShape for RapierBoxShape {
    fn data(&self) -> Variant {
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
    fn shared_shape(&self, scale: Vector<f32>) -> SharedShape {
        if self.margin.is_zero_approx() {
            SharedShape::new(self.shape.scaled(&scale))
        } else {
            SharedShape::new(RoundCuboid {
                inner_shape: self.shape.scaled(&scale),
                border_radius: self.margin,
            })
        }
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

    fn set_margin(&mut self, margin: f32) {
        if margin.approx_eq(&self.margin) {
            self.margin = margin;
            self.update_owners();
        }
    }

    fn margin(&self) -> f32 {
        self.margin
    }
}
