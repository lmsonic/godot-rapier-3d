use std::{cell::RefCell, rc::Rc};

use godot::prelude::{Rid, Transform3D};

use crate::{
    math::transform_to_isometry,
    shape::{RapierShape, RapierShapeInstance},
};
#[derive(Default)]
pub struct RapierBody {
    pub space_id: Option<Rid>,
    pub shapes: Vec<RapierShapeInstance>,
}

impl RapierBody {
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
