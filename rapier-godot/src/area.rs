use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::prelude::*;

use crate::{
    math::transform_to_isometry,
    shape::{RapierShape, RapierShapeInstance},
};

#[derive(Default)]
pub struct RapierArea {
    pub space_id: Option<Rid>,
    pub handle: Option<ColliderHandle>,
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
        let isometry = transform_to_isometry(&transform);
        let shape_instance = RapierShapeInstance::new(shape, isometry, disabled);
        self.shapes.push(shape_instance);
    }
}
