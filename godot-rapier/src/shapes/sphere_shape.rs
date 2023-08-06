use std::{cell::RefCell, rc::Rc};

use godot::prelude::{math::ApproxEq, *};
use rapier3d::{parry::either::Either, prelude::*};

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
    fn data(&self) -> Variant {
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

    fn shared_shape(&self, scale: Vector<f32>) -> SharedShape {
        self.shape.scaled(&scale, 3).map_or_else(
            || SharedShape::new(self.shape),
            |either| match either {
                Either::Left(ball) => SharedShape::new(ball),
                Either::Right(convex_poly) => SharedShape::new(convex_poly),
            },
        )
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
