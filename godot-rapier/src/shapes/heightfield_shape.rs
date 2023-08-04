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
        Variant::nil()
    }

    fn set_data(&mut self, data: Variant) {}

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
