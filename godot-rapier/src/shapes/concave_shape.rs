#![allow(clippy::module_name_repetitions)]

use std::{cell::RefCell, rc::Rc};

use godot::prelude::{math::ApproxEq, *};
use rapier3d::prelude::*;

use crate::collision_object::RapierCollisionObject;

use super::RapierShape;
pub struct RapierConcaveShape {
    shape: TriMesh,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
    backface_collision: bool,
}

impl RapierConcaveShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: TriMesh::new(
                vec![
                    point![0.0, 0.0, 0.0],
                    point![0.0, 0.0, 0.0],
                    point![0.0, 0.0, 0.0],
                ],
                vec![[0, 1, 2]],
            ),
            owners: vec![],
            rid,
            backface_collision: false,
        }
    }

    pub fn get_compound_convex_shapes(&self, scale: Vector<f32>) -> SharedShape {
        let shape = self.shape.clone().scaled(&scale);
        SharedShape::convex_decomposition(shape.vertices(), shape.indices())
    }
}

fn generate_indices(
    input: &Vec<Point<f32>>,
    backface_collision: bool,
) -> (Vec<Point<f32>>, Vec<[u32; 3]>) {
    let mut vertices: Vec<Point<f32>> = Vec::with_capacity(input.len());
    let mut indices = Vec::with_capacity(input.len());
    for face in input.chunks_exact(3) {
        let mut index_triplet = [0, 0, 0];

        // FRONT FACE CW
        for i in 0..3 {
            if let Some(index) = vertices.iter().position(|v| {
                v.x.approx_eq(&face[i].x) && v.y.approx_eq(&face[i].y) && v.z.approx_eq(&face[i].z)
            }) {
                index_triplet[i] = index as u32;
            } else {
                vertices.push(face[i]);
                index_triplet[i] = (vertices.len() - 1) as u32;
            }
        }
        // BACKFACE
        if backface_collision {
            indices.push([index_triplet[2], index_triplet[1], index_triplet[0]]);
        }
        indices.push(index_triplet);
    }
    (vertices, indices)
}

impl RapierShape for RapierConcaveShape {
    fn rid(&self) -> Rid {
        self.rid
    }

    fn data(&self) -> Variant {
        let faces = self
            .shape
            .vertices()
            .iter()
            .map(|v| Vector3::new(v.x, v.y, v.z))
            .collect::<Array<Vector3>>();

        Variant::from(dict! {
            "faces" : faces, "backface_collision":self.backface_collision
        })
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to::<Dictionary>() {
            Ok(d) => {
                match d.get_or_nil("backface_collision").try_to::<bool>() {
                    Ok(value) => {
                        if value != self.backface_collision {
                            self.backface_collision = value;
                            let (vertices, indices) =
                                generate_indices(self.shape.vertices(), self.backface_collision);
                            self.shape = TriMesh::new(vertices, indices);
                            self.update_owners();
                        }
                    }
                    Err(e) => godot_error!("{:?}", e),
                }
                match d.get_or_nil("faces").try_to::<PackedVector3Array>() {
                    Ok(vertices) => {
                        let vertices: Vec<Point<f32>> = vertices
                            .to_vec()
                            .iter()
                            .map(|v| point![v.x, v.y, v.z])
                            .collect();
                        if vertices != *self.shape.vertices() {
                            let (vertices, indices) =
                                generate_indices(&vertices, self.backface_collision);
                            self.shape = TriMesh::new(vertices, indices);
                            self.update_owners();
                        }
                    }
                    Err(e) => godot_error!("{:?}", e),
                };
            }
            Err(e) => godot_error!("{:?}", e),
        }
    }

    fn shared_shape(&self, scale: Vector<f32>) -> SharedShape {
        //SharedShape::new(self.shape.scaled(&scale))
        self.get_compound_convex_shapes(scale)
    }

    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_CONCAVE_POLYGON
    }

    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }
    fn owners_mut(&mut self) -> &mut Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &mut self.owners
    }
}
