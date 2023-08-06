use std::{cell::RefCell, rc::Rc};

use godot::prelude::{math::ApproxEq, *};
use rapier3d::prelude::*;

use crate::collision_object::RapierCollisionObject;

use super::RapierShape;

pub struct RapierSeparationRayShape {
    shape: Segment,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
}

impl RapierSeparationRayShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: Segment::new(point![0.0, 0.0, 0.0], point![0.0, 0.0, 0.5]),
            owners: vec![],
            rid,
        }
    }
}

impl RapierShape for RapierSeparationRayShape {
    fn rid(&self) -> Rid {
        self.rid
    }

    fn data(&self) -> Variant {
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

    fn shared_shape(&self, scale: Vector<f32>) -> SharedShape {
        SharedShape::new(self.shape.scaled(&scale))
    }

    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_SEPARATION_RAY
    }

    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }
    fn owners_mut(&mut self) -> &mut Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &mut self.owners
    }
}
