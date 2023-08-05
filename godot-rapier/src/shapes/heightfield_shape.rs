#![allow(clippy::module_name_repetitions)]

use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::{na::dmatrix, prelude::*};

use crate::collision_object::RapierCollisionObject;

use super::RapierShape;

pub struct RapierHeightmapShape {
    shape: HeightField,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
}

impl RapierShape for RapierHeightmapShape {
    fn rid(&self) -> Rid {
        self.rid
    }

    fn get_data(&self) -> Variant {
        let heights: PackedFloat32Array = self.shape.heights().iter().copied().collect();
        let width = self.shape.ncols() as i32;
        let depth = self.shape.nrows() as i32;
        Variant::from(dict! {"width":width,"depth":depth,"heights":heights})
    }

    fn set_data(&mut self, data: Variant) {
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
                self.update_owners();
            }
            Err(e) => godot_error!("{:?}", e),
        }
    }

    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape.clone())
    }

    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_HEIGHTMAP
    }

    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }
}

impl RapierHeightmapShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: HeightField::new(dmatrix![0.0,0.0;0.0,0.0], vector![1.0, 1.0, 1.0]),
            owners: vec![],
            rid,
        }
    }
}
