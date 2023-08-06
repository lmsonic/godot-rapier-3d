#![allow(clippy::module_name_repetitions)]

use std::{cell::RefCell, rc::Rc};

use godot::prelude::{math::ApproxEq, *};
use rapier3d::prelude::*;

use crate::collision_object::RapierCollisionObject;

use super::RapierShape;
pub struct RapierConvexShape {
    shape: ConvexPolyhedron,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
    margin: f32,
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
            margin: 0.0,
        }
    }
}

impl RapierShape for RapierConvexShape {
    fn rid(&self) -> Rid {
        self.rid
    }

    fn data(&self) -> Variant {
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

    fn shared_shape(&self, scale: Vector<f32>) -> SharedShape {
        let shape = self.shape.clone().scaled(&scale).map_or_else(
            || {
                godot_error!("Scaling one of the collision shape axis to 0");
                self.shape.clone()
            },
            |scaled_shape| scaled_shape,
        );
        if self.margin.is_zero_approx() {
            SharedShape::new(shape)
        } else {
            SharedShape::new(RoundConvexPolyhedron {
                inner_shape: self.shape.clone(),
                border_radius: self.margin,
            })
        }
    }

    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_CONVEX_POLYGON
    }

    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }
    fn owners_mut(&mut self) -> &mut Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &mut self.owners
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
