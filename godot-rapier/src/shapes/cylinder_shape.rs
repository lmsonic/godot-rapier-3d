use std::{cell::RefCell, rc::Rc};

use godot::prelude::{math::ApproxEq, *};
use rapier3d::prelude::*;

use crate::collision_object::RapierCollisionObject;

use super::RapierShape;

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
