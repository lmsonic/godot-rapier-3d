#![allow(clippy::module_name_repetitions)]

use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::prelude::*;

use crate::collision_object::RapierCollisionObject;

use super::RapierShape;
pub struct RapierConvexShape {
    shape: ConvexPolyhedron,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
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
        }
    }
}

impl RapierShape for RapierConvexShape {
    fn rid(&self) -> Rid {
        self.rid
    }

    fn get_data(&self) -> Variant {
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

    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape.clone())
    }

    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_CONVEX_POLYGON
    }

    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }
}
