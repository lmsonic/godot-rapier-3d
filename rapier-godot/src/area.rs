use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;

use crate::shape::{RapierShape, RapierShapeInstance};

#[derive(Default)]
pub struct RapierArea {
    pub space_id: Option<Rid>,
    pub shapes: Vec<RapierShapeInstance>,
}

impl RapierArea {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_shape(
        &mut self,
        shape: Rc<RefCell<dyn RapierShape>>,
        transform: Transform3D,
        disabled: bool,
    ) {
        let isometry = decompose(transform);
        let shape_instance = RapierShapeInstance::new(shape, isometry, disabled);
        self.shapes.push(shape_instance);
    }
}

fn decompose(
    transform: Transform3D,
) -> rapier3d::na::Isometry<f32, rapier3d::na::Unit<rapier3d::na::Quaternion<f32>>, 3> {
    todo!()
}
