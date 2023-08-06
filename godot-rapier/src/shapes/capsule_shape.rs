use std::{cell::RefCell, rc::Rc};

use godot::prelude::{math::ApproxEq, *};
use rapier3d::{parry::either::Either, prelude::*};

use crate::collision_object::RapierCollisionObject;

use super::RapierShape;

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
    fn data(&self) -> Variant {
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
    fn shared_shape(&self, scale: Vector<f32>) -> SharedShape {
        self.shape.scaled(&scale, 3).map_or_else(
            || {
                godot_error!("Scaling one of the collision shape axis to 0");
                SharedShape::new(self.shape)
            },
            |scaled_shape| match scaled_shape {
                Either::Left(capsule) => SharedShape::new(capsule),
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
        godot::engine::physics_server_3d::ShapeType::SHAPE_CAPSULE
    }
}
