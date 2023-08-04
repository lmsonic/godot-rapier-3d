use std::{cell::RefCell, rc::Rc};

use godot::prelude::{math::ApproxEq, *};
use rapier3d::prelude::*;

use crate::collision_object::RapierCollisionObject;

use super::RapierShape;
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
