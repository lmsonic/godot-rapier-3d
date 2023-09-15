use godot::prelude::*;

use super::RapierShape;

pub struct CylinderShape {
    rid: Rid,
    radius: f32,
    height: f32,
    margin: f32,
    owners: Vec<Rid>,
}

impl CylinderShape {
    pub const fn new(rid: Rid) -> Self {
        Self {
            rid,
            radius: 0.0,
            height: 0.0,
            margin: 0.0,
            owners: vec![],
        }
    }
}

impl RapierShape for CylinderShape {
    fn set_data(&mut self, data: godot::prelude::Variant) {
        match data.try_to::<Dictionary>() {
            Ok(d) => {
                match d.get_or_nil("radius").try_to::<f32>() {
                    Ok(radius) => self.radius = radius,
                    Err(e) => godot_error!("{:?}", e),
                };
                match d.get_or_nil("height").try_to::<f32>() {
                    Ok(height) => self.height = height,
                    Err(e) => godot_error!("{:?}", e),
                };
            }
            Err(e) => godot_error!("{:?}", e),
        };
    }

    fn get_shape_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_CYLINDER
    }

    fn get_data(&self) -> godot::prelude::Variant {
        Variant::from(dict! {"radius":self.radius,"height":self.height})
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
