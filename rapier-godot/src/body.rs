use std::{cell::RefCell, rc::Rc};

use godot::{engine::physics_server_3d::BodyMode, prelude::*};
use rapier3d::prelude::*;

use crate::{
    collision_object::RapierCollisionObject,
    math::transform_to_isometry,
    shape::{RapierShape, RapierShapeInstance},
};

#[allow(clippy::module_name_repetitions)]
pub struct RapierBody {
    space_id: Option<Rid>,
    handle: Option<RigidBodyHandle>,
    shapes: Vec<RapierShapeInstance>,
    body_mode: BodyMode,
    instance_id: Option<u64>,
    body_state_callback: Callable,
}

impl RapierCollisionObject for RapierBody {
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

    fn remove_shape_rid(&mut self, shape_rid: Rid) {
        self.shapes.retain(|s| s.shape.borrow().rid() != shape_rid);
    }
    fn remove_nth_shape(&mut self, idx: usize) {
        self.shapes.swap_remove(idx);
    }

    fn clear_shapes(&mut self) {
        self.shapes.clear();
    }

    fn set_shape(&mut self, idx: usize, s: Rc<RefCell<dyn RapierShape>>) {
        if let Some(shape) = self.shapes.get_mut(idx) {
            shape.shape = s.clone();
        }
    }
    fn set_shape_transform(&mut self, idx: usize, transform: Transform3D) {
        if let Some(shape) = self.shapes.get_mut(idx) {
            shape.isometry = transform_to_isometry(&transform);
        }
    }

    fn set_shape_disabled(&mut self, idx: usize, disabled: bool) {
        if let Some(shape) = self.shapes.get_mut(idx) {
            shape.disabled = disabled;
        }
    }
}

impl RapierBody {
    pub fn new() -> Self {
        Self {
            space_id: None,
            handle: None,
            shapes: vec![],
            body_mode: BodyMode::BODY_MODE_STATIC,
            instance_id: None,
            body_state_callback: Callable::invalid(),
        }
    }
    pub fn set_body_mode(&mut self, mode: BodyMode) {
        self.body_mode = mode;
    }
    pub const fn get_body_mode(&self) -> BodyMode {
        self.body_mode
    }

    pub fn set_handle(&mut self, handle: RigidBodyHandle) {
        self.handle = Some(handle);
    }

    pub fn body_state_callback(&self) -> &Callable {
        &self.body_state_callback
    }

    pub fn set_body_state_callback(&mut self, body_state_callback: Callable) {
        self.body_state_callback = body_state_callback;
    }
}
