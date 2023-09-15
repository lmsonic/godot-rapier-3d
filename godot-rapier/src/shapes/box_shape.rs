use godot::prelude::*;

use super::RapierShape;

pub struct BoxShape {
    rid: Rid,
    half_extents: Vector3,
    margin: f32,
    owners: Vec<Rid>,
}

impl BoxShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            half_extents: Vector3::default(),
            margin: 0.0,
            owners: vec![],
        }
    }
}

impl RapierShape for BoxShape {
    fn set_data(&mut self, data: godot::prelude::Variant) {
        match data.try_to::<Vector3>() {
            Ok(half_extents) => self.half_extents = half_extents,
            Err(err) => godot_error!("{:?}", err),
        };
    }

    fn get_shape_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_BOX
    }

    fn get_data(&self) -> godot::prelude::Variant {
        Variant::from(self.half_extents)
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
