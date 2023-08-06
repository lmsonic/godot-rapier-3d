use std::{cell::RefCell, rc::Rc};

use godot::prelude::{math::ApproxEq, *};
use rapier3d::{parry::either::Either, prelude::*};

use crate::collision_object::RapierCollisionObject;

use super::RapierShape;

pub struct RapierCylinderShape {
    shape: Cylinder,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
    margin: f32,
}

impl RapierCylinderShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: Cylinder::new(0.5, 0.2),
            owners: vec![],
            rid,
            margin: 0.0,
        }
    }
}

impl RapierShape for RapierCylinderShape {
    fn data(&self) -> Variant {
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
    fn shared_shape(&self, scale: Vector<f32>) -> SharedShape {
        self.shape.scaled(&scale, 3).map_or_else(
            || {
                godot_error!("Scaling one of the collision shape axis to 0");
                if self.margin.is_zero_approx() {
                    SharedShape::new(self.shape)
                } else {
                    SharedShape::new(RoundCylinder {
                        inner_shape: self.shape,
                        border_radius: self.margin,
                    })
                }
            },
            |scaled_shape| match scaled_shape {
                Either::Left(capsule) => {
                    if self.margin().is_zero_approx() {
                        SharedShape::new(capsule)
                    } else {
                        SharedShape::new(RoundCylinder {
                            inner_shape: capsule,
                            border_radius: self.margin,
                        })
                    }
                }
                Either::Right(convex_poly) => {
                    if self.margin().is_zero_approx() {
                        SharedShape::new(convex_poly)
                    } else {
                        SharedShape::new(RoundConvexPolyhedron {
                            inner_shape: convex_poly,
                            border_radius: self.margin,
                        })
                    }
                }
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
        godot::engine::physics_server_3d::ShapeType::SHAPE_CYLINDER
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
