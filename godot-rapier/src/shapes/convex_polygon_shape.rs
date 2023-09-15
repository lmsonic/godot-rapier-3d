use godot::prelude::{math::ApproxEq, *};
use rapier3d::prelude::*;

use super::RapierShape;

pub struct ConvexPolygonShape {
    rid: Rid,
    shape: ConvexPolyhedron,
    margin: f32,
    owners: Vec<Rid>,
}

impl ConvexPolygonShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            shape: ConvexPolyhedron::from_convex_hull(&[
                point![1.0, 0.0, 0.0],
                point![0.0, 1.0, 0.0],
                point![0.0, 0.0, 1.0],
            ])
            .unwrap(),
            margin: 0.0,
            owners: vec![],
        }
    }
}

impl RapierShape for ConvexPolygonShape {
    fn collider(&self) -> ColliderBuilder {
        if self.margin.is_zero_approx() {
            ColliderBuilder::new(SharedShape::new(self.shape.clone()))
        } else {
            ColliderBuilder::new(SharedShape::new(RoundShape {
                inner_shape: self.shape.clone(),
                border_radius: self.margin,
            }))
        }
    }
    fn rid(&self) -> Rid {
        self.rid
    }
    fn set_data(&mut self, data: godot::prelude::Variant) {
        match data.try_to::<PackedVector3Array>() {
            Ok(vertices) => {
                let vertices = vertices
                    .to_vec()
                    .iter()
                    .map(|v| point![v.x, v.y, v.z])
                    .collect::<Vec<Point<f32>>>();

                if let Some(convex) = ConvexPolyhedron::from_convex_hull(&vertices) {
                    self.shape = convex;
                } else {
                    godot_error!("ConvexPolygonShape {} is not convex", self.rid);
                }
            }
            Err(e) => godot_error!("{:?}", e),
        }
    }

    fn get_shape_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_CONVEX_POLYGON
    }

    fn get_data(&self) -> Variant {
        let vertices: Array<Vector3> = self
            .shape
            .points()
            .iter()
            .map(|v| Vector3::new(v.x, v.y, v.z))
            .collect();
        Variant::from(vertices)
    }
    fn set_margin(&mut self, margin: f32) {
        self.margin = margin;
    }

    fn get_margin(&self) -> f32 {
        self.margin
    }

    fn add_owner(&mut self, owner: Rid) {
        self.owners.push(owner);
    }

    fn remove_owner(&mut self, owner: Rid) {
        self.owners.retain(|o| *o != owner);
    }

    fn owners(&self) -> &[Rid] {
        self.owners.as_ref()
    }
}
