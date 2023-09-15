use godot::prelude::*;

use super::RapierShape;

pub struct ConvexPolygonShape {
    rid: Rid,
    vertices: PackedVector3Array,
    margin: f32,
    owners: Vec<Rid>,
}

impl ConvexPolygonShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            vertices: PackedVector3Array::default(),
            margin: 0.0,
            owners: vec![],
        }
    }
}

impl RapierShape for ConvexPolygonShape {
    fn set_data(&mut self, data: godot::prelude::Variant) {
        match data.try_to::<PackedVector3Array>() {
            Ok(vertices) => self.vertices = vertices,
            Err(e) => godot_error!("{:?}", e),
        }
    }

    fn get_shape_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_CONVEX_POLYGON
    }

    fn get_data(&self) -> godot::prelude::Variant {
        Variant::from(self.vertices.clone())
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
