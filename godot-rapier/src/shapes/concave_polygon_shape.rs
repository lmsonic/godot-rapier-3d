use godot::prelude::*;

use super::RapierShape;

pub struct ConcavePolygonShape {
    rid: Rid,
    backface_collision: bool,
    faces: PackedVector3Array,
    owners: Vec<Rid>,
}

impl ConcavePolygonShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            backface_collision: false,
            faces: PackedVector3Array::default(),
            owners: vec![],
        }
    }
}

impl RapierShape for ConcavePolygonShape {
    fn set_data(&mut self, data: godot::prelude::Variant) {
        match data.try_to::<Dictionary>() {
            Ok(d) => {
                match d.get_or_nil("backface_collision").try_to::<bool>() {
                    Ok(value) => self.backface_collision = value,
                    Err(e) => godot_error!("{:?}", e),
                }
                match d.get_or_nil("faces").try_to::<PackedVector3Array>() {
                    Ok(vertices) => self.faces = vertices,
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
        Variant::from(dict! {
            "faces" : self.faces.clone(), "backface_collision":self.backface_collision,
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
