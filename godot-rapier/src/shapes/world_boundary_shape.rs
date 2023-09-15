use godot::prelude::*;
use rapier3d::prelude::{ColliderBuilder, UnitVector};

use crate::utils::IntoExt;

use super::RapierShape;

pub struct WorldBoundaryShape {
    rid: Rid,
    plane: Plane,
    owners: Vec<Rid>,
}
impl WorldBoundaryShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            plane: Plane::new(Vector3::UP, 0.0),
            owners: vec![],
        }
    }
}

impl RapierShape for WorldBoundaryShape {
    fn collider(&self) -> ColliderBuilder {
        let normal = UnitVector::new_normalize(self.plane.normal.into_ext());
        ColliderBuilder::halfspace(normal)
    }
    fn rid(&self) -> Rid {
        self.rid
    }
    fn set_data(&mut self, data: godot::prelude::Variant) {
        match data.try_to::<Plane>() {
            Ok(plane) => self.plane = plane,
            Err(err) => godot_error!("{:?}", err),
        }
    }

    fn get_shape_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_WORLD_BOUNDARY
    }

    fn get_data(&self) -> godot::prelude::Variant {
        Variant::from(self.plane)
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
