use godot::prelude::*;

use super::RapierShape;

pub struct SphereShape {
    rid: Rid,
    radius: f32,
    owners: Vec<Rid>,
}

impl SphereShape {
    pub const fn new(rid: Rid) -> Self {
        Self {
            rid,
            radius: 0.0,
            owners: vec![],
        }
    }
}

impl RapierShape for SphereShape {
    fn set_data(&mut self, data: godot::prelude::Variant) {
        match data.try_to::<f32>() {
            Ok(radius) => self.radius = radius,
            Err(err) => godot_error!("{:?}", err),
        };
    }

    fn get_shape_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_SPHERE
    }

    fn get_data(&self) -> godot::prelude::Variant {
        Variant::from(self.radius)
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
