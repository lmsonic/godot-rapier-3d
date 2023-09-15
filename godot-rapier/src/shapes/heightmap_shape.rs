use godot::prelude::*;
use rapier3d::{na::dmatrix, prelude::*};

use super::RapierShape;

pub struct HeightmapShape {
    rid: Rid,
    shape: HeightField,
    owners: Vec<Rid>,
}

impl HeightmapShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            shape: HeightField::new(dmatrix![0.0,0.0;0.0,0.0], vector![1.0, 1.0, 1.0]),
            owners: vec![],
        }
    }
}

impl RapierShape for HeightmapShape {
    fn collider(&self) -> ColliderBuilder {
        ColliderBuilder::new(SharedShape::new(self.shape.clone()))
    }
    fn rid(&self) -> Rid {
        self.rid
    }
    fn set_data(&mut self, data: godot::prelude::Variant) {
        match data.try_to::<Dictionary>() {
            Ok(d) => {
                let width = match d.get_or_nil("width").try_to::<i32>() {
                    Ok(width) => width as usize,
                    Err(e) => {
                        godot_error!("{:?}", e);
                        self.shape.ncols()
                    }
                };
                let depth = match d.get_or_nil("depth").try_to::<i32>() {
                    Ok(depth) => depth as usize,
                    Err(e) => {
                        godot_error!("{:?}", e);
                        self.shape.nrows()
                    }
                };
                let heights = match d.get_or_nil("heights").try_to::<PackedFloat32Array>() {
                    Ok(heights) => heights,
                    Err(e) => {
                        godot_error!("{:?}", e);
                        return;
                    }
                };
                if width * depth != heights.len() {
                    godot_error!("width * depth must equal heights(PackedFloat32Array) size");
                    return;
                }
                let heights = DMatrix::from_fn(depth, width, |i, j| heights.get(i * width + j));

                self.shape = HeightField::new(heights, vector![1.0, 1.0, 1.0]);
            }
            Err(e) => godot_error!("{:?}", e),
        }
    }

    fn get_shape_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_HEIGHTMAP
    }

    fn get_data(&self) -> godot::prelude::Variant {
        let heights: PackedFloat32Array = self.shape.heights().iter().copied().collect();
        let width = self.shape.ncols() as i32;
        let depth = self.shape.nrows() as i32;
        Variant::from(dict! {"width":width,"depth":depth,"heigths":heights})
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
