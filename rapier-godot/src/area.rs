use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::prelude::*;

use crate::{
    collision_object::RapierCollisionObject,
    math::transform_to_isometry,
    shape::{RapierShape, RapierShapeInstance},
};

#[derive(Default)]
#[allow(clippy::module_name_repetitions)]
pub struct RapierArea {
    space_id: Option<Rid>,
    handle: Option<ColliderHandle>,
    shapes: Vec<RapierShapeInstance>,
    instance_id: Option<u64>,
}

impl RapierCollisionObject for RapierArea {
    fn set_space_id(&mut self, space_id: Rid) {
        self.space_id = Some(space_id);
    }

    fn get_space_id(&self) -> Option<Rid> {
        self.space_id
    }

    fn add_shape(
        &mut self,
        shape: Rc<RefCell<dyn RapierShape>>,
        transform: Transform3D,
        disabled: bool,
    ) {
        let isometry = transform_to_isometry(&transform);
        let shape_instance = RapierShapeInstance::new(shape, isometry, disabled);
        self.shapes.push(shape_instance);
    }

    fn get_shapes(&self) -> &Vec<RapierShapeInstance> {
        &self.shapes
    }

    fn set_instance_id(&mut self, id: u64) {
        self.instance_id = Some(id);
    }

    fn get_instance_id(&self) -> Option<u64> {
        self.instance_id
    }

    fn remove_shape(&mut self, shape_rid: Rid) {
        self.shapes.retain(|s| s.shape.borrow().rid() != shape_rid);
    }
}

impl RapierArea {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_handle(&mut self, handle: ColliderHandle) {
        self.handle = Some(handle);
    }
}
