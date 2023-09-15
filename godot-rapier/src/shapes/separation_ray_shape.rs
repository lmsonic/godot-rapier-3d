use godot::prelude::*;
use rapier3d::prelude::*;

use super::RapierShape;

pub struct SeparationRayShape {
    rid: Rid,
    length: f32,
    slide_on_slope: bool,
    owners: Vec<Rid>,
}

impl SeparationRayShape {
    pub const fn new(rid: Rid) -> Self {
        Self {
            rid,
            length: 0.0,
            slide_on_slope: false,
            owners: vec![],
        }
    }
}

impl RapierShape for SeparationRayShape {
    fn collider(&self) -> ColliderBuilder {
        ColliderBuilder::segment(point![0.0, 0.0, 0.0], point![0.0, 0.0, 1.0] * self.length)
    }
    fn rid(&self) -> Rid {
        self.rid
    }
    fn set_data(&mut self, data: godot::prelude::Variant) {
        match data.try_to::<Dictionary>() {
            Ok(d) => {
                match d.get_or_nil("length").try_to::<f32>() {
                    Ok(length) => self.length = length,
                    Err(e) => godot_error!("{:?}", e),
                };
                match d.get_or_nil("slide_on_slope").try_to::<bool>() {
                    Ok(value) => self.slide_on_slope = value,
                    Err(e) => godot_error!("{:?}", e),
                };
            }
            Err(e) => godot_error!("{:?}", e),
        }
    }

    fn get_shape_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_SEPARATION_RAY
    }

    fn get_data(&self) -> godot::prelude::Variant {
        Variant::from(dict! {"length":self.length,"slide_on_slope":self.slide_on_slope})
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
