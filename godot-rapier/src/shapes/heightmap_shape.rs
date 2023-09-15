use godot::prelude::*;

use super::RapierShape;

pub struct HeightmapShape {
    rid: Rid,
    heights: PackedFloat32Array,
    width: i32,
    depth: i32,
    owners: Vec<Rid>,
}

impl HeightmapShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            heights: PackedFloat32Array::default(),
            width: 0,
            depth: 0,
            owners: vec![],
        }
    }
}

impl RapierShape for HeightmapShape {
    fn set_data(&mut self, data: godot::prelude::Variant) {
        match data.try_to::<Dictionary>() {
            Ok(d) => {
                match d.get_or_nil("width").try_to::<i32>() {
                    Ok(width) => self.width = width,
                    Err(e) => {
                        godot_error!("{:?}", e);
                    }
                };
                match d.get_or_nil("depth").try_to::<i32>() {
                    Ok(depth) => self.depth = depth,
                    Err(e) => {
                        godot_error!("{:?}", e);
                    }
                };
                match d.get_or_nil("heights").try_to::<PackedFloat32Array>() {
                    Ok(heights) => self.heights = heights,
                    Err(e) => {
                        godot_error!("{:?}", e);
                    }
                };
            }
            Err(e) => godot_error!("{:?}", e),
        }
    }

    fn get_shape_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_HEIGHTMAP
    }

    fn get_data(&self) -> godot::prelude::Variant {
        Variant::from(dict! {"width":self.width,"depth":self.depth,"heigths":self.heights.clone()})
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
