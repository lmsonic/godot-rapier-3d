use godot::prelude::{math::ApproxEq, *};
use rapier3d::prelude::*;

use super::RapierShape;

pub struct ConcavePolygonShape {
    rid: Rid,
    backface_collision: bool,
    shape: TriMesh,

    owners: Vec<Rid>,
}

impl ConcavePolygonShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            backface_collision: false,
            shape: TriMesh::new(
                vec![
                    point![0.0, 0.0, 0.0],
                    point![0.0, 0.0, 0.0],
                    point![0.0, 0.0, 0.0],
                ],
                vec![[0, 1, 2]],
            ),
            owners: vec![],
        }
    }

    fn generate_indices(&mut self, input: &PackedVector3Array) {
        let input: Vec<Point<f32>> = input
            .to_vec()
            .iter()
            .map(|v| point![v.x, v.y, v.z])
            .collect();

        let mut vertices: Vec<Point<f32>> = Vec::with_capacity(input.len());
        let mut indices = Vec::with_capacity(input.len());
        for face in input.chunks_exact(3) {
            let mut index_triplet = [0, 0, 0];

            // FRONT FACE CW
            for i in 0..3 {
                if let Some(index) = vertices.iter().position(|v| {
                    v.x.approx_eq(&face[i].x)
                        && v.y.approx_eq(&face[i].y)
                        && v.z.approx_eq(&face[i].z)
                }) {
                    index_triplet[i] = index as u32;
                } else {
                    vertices.push(face[i]);
                    index_triplet[i] = (vertices.len() - 1) as u32;
                }
            }
            // BACKFACE
            if self.backface_collision {
                indices.push([index_triplet[2], index_triplet[1], index_triplet[0]]);
            }
            indices.push(index_triplet);
        }
        self.shape = TriMesh::new(vertices, indices);
    }
}

impl RapierShape for ConcavePolygonShape {
    fn collider(&self) -> ColliderBuilder {
        ColliderBuilder::new(SharedShape::new(self.shape.clone()))
    }
    fn rid(&self) -> Rid {
        self.rid
    }
    fn set_data(&mut self, data: godot::prelude::Variant) {
        match data.try_to::<Dictionary>() {
            Ok(d) => {
                match d.get_or_nil("backface_collision").try_to::<bool>() {
                    Ok(value) => self.backface_collision = value,
                    Err(e) => godot_error!("{:?}", e),
                }
                match d.get_or_nil("faces").try_to::<PackedVector3Array>() {
                    Ok(vertices) => {
                        self.generate_indices(&vertices);
                    }
                    Err(e) => godot_error!("{:?}", e),
                };
            }
            Err(e) => godot_error!("{:?}", e),
        }
    }

    fn get_shape_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_CONCAVE_POLYGON
    }

    fn get_data(&self) -> godot::prelude::Variant {
        let faces = self
            .shape
            .vertices()
            .iter()
            .map(|v| Vector3::new(v.x, v.y, v.z))
            .collect::<Array<Vector3>>();
        Variant::from(dict! {
            "faces" : faces, "backface_collision":self.backface_collision,
        })
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
