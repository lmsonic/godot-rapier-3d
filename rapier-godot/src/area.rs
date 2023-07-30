use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;

use crate::{shape::RapierShape, space::RapierSpace};

use crate::shape::RapierObject;

impl RapierObject for RapierArea {}
#[allow(clippy::module_name_repetitions)]
pub struct RapierArea {
    rid: Rid,
}

impl RapierArea {
    pub fn get_space(&self) -> Rc<RefCell<RapierSpace>> {
        todo!()
    }
    pub fn set_space(&mut self, space: Rc<RefCell<RapierSpace>>) {}
    pub fn add_shape(
        &mut self,
        shape: Rc<RefCell<dyn RapierShape>>,
        transform: Transform3D,
        disabled: bool,
    ) {
    }
    pub fn set_priority(&mut self, priority: i32) {}
}

impl RapierArea {
    pub fn new(rid: Rid) -> Self {
        Self { rid }
    }
}
