#![allow(clippy::option_if_let_else)]
use std::{cell::RefCell, rc::Rc};

use godot::{engine::physics_server_3d::BodyMode, prelude::*};
use rapier3d::prelude::*;

use crate::{
    collision_object::{Handle, RapierCollisionObject},
    conversions::{
        body_mode_to_body_type, godot_vector_to_rapier_point, godot_vector_to_rapier_vector,
        rapier_vector_to_godot_vector,
    },
    direct_body_state_3d::RapierPhysicsDirectBodyState3D,
    error::RapierError,
    error::RapierResult,
    shapes::RapierShapeInstance,
    space::RapierSpace,
};

pub struct RapierBody {
    rid: Rid,
    space: Option<Rc<RefCell<RapierSpace>>>,
    handle: Option<RigidBodyHandle>,
    shapes: Vec<RapierShapeInstance>,
    body_mode: BodyMode,
    instance_id: Option<u64>,
    ccd_enabled: bool,
    body_state_callback: Callable,
    constant_force: Vector<f32>,
    constant_torque: Vector<f32>,

    collision_layer: u32,
    collision_mask: u32,
}

impl Default for RapierBody {
    fn default() -> Self {
        Self {
            rid: Rid::Invalid,
            space: Option::default(),
            handle: Option::default(),
            shapes: Vec::default(),
            body_mode: BodyMode::BODY_MODE_STATIC,
            instance_id: Option::default(),
            ccd_enabled: Default::default(),
            body_state_callback: Callable::invalid(),
            constant_force: Vector::default(),
            constant_torque: Vector::default(),
            collision_layer: 1,
            collision_mask: 1,
        }
    }
}

impl RapierCollisionObject for RapierBody {
    fn set_space(&mut self, space: Rc<RefCell<RapierSpace>>) {
        self.space = Some(space);
    }
    #[track_caller]
    fn space(&self) -> Option<Rc<RefCell<RapierSpace>>> {
        if self.space.is_none() {
            let caller_location = std::panic::Location::caller();
            let file = caller_location.file();
            let line_number = caller_location.line();
            godot_error!(
                "{}, called from {}:{}",
                RapierError::ObjectSpaceNotSet(self.rid),
                file,
                line_number
            );
        }
        self.space.clone()
    }

    fn shapes(&self) -> &Vec<RapierShapeInstance> {
        &self.shapes
    }
    fn shapes_mut(&mut self) -> &mut Vec<RapierShapeInstance> {
        &mut self.shapes
    }

    fn set_instance_id(&mut self, id: u64) {
        self.instance_id = Some(id);
    }

    fn instance_id(&self) -> Option<u64> {
        if self.instance_id.is_none() {
            godot_error!("{}", RapierError::BodyInstanceIDNotSet(self.rid));
        }
        self.instance_id
    }

    fn rid(&self) -> Rid {
        self.rid
    }

    fn remove_from_space(&self) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                space.borrow_mut().remove_body(handle);
            }
        }
    }
    fn remove_space(&mut self) {
        self.space = None;
        self.handle = None;
    }
    fn set_collision_layer(&mut self, layer: u32) {
        self.collision_layer = layer;
    }

    fn get_collision_layer(&self) -> u32 {
        self.collision_layer
    }

    fn set_collision_mask(&mut self, mask: u32) {
        self.collision_mask = mask;
    }

    fn get_collision_mask(&self) -> u32 {
        self.collision_mask
    }

    fn generic_handle(&self) -> Handle {
        if self.handle.is_none() {
            godot_error!("{}", RapierError::AreaHandleNotSet(self.rid));
        }
        match self.handle {
            Some(handle) => Handle::BodyHandle(handle),
            None => Handle::NotSet,
        }
    }
}

impl RapierBody {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            ..Default::default()
        }
    }

    pub fn set_body_mode(&mut self, mode: BodyMode) {
        self.body_mode = mode;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_body_mode(handle, mode);
            }
        }
    }

    pub fn set_enable_ccd(&mut self, enabled: bool) {
        self.ccd_enabled = enabled;
        if let Some(space) = self.space() {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_ccd_enabled(handle, enabled);
            }
        }
    }

    pub const fn is_ccd_enabled(&self) -> bool {
        self.ccd_enabled
    }

    pub const fn get_body_mode(&self) -> BodyMode {
        self.body_mode
    }

    pub fn set_handle(&mut self, handle: RigidBodyHandle) {
        self.handle = Some(handle);
    }

    pub const fn body_state_callback(&self) -> &Callable {
        &self.body_state_callback
    }

    pub fn set_body_state_callback(&mut self, body_state_callback: Callable) {
        self.body_state_callback = body_state_callback;
    }

    pub fn apply_central_impulse(&mut self, impulse: Vector3) {
        if let Some(space) = self.space() {
            if let Some(handle) = self.handle() {
                space.borrow_mut().apply_central_impulse(handle, impulse);
            }
        }
    }
    pub fn apply_impulse(&mut self, impulse: Vector3, position: Vector3) {
        if let Some(space) = self.space() {
            if let Some(handle) = self.handle() {
                space.borrow_mut().apply_impulse(handle, impulse, position);
            }
        }
    }

    pub fn apply_torque_impulse(&mut self, impulse: Vector3) {
        if let Some(space) = self.space() {
            if let Some(handle) = self.handle() {
                space.borrow_mut().apply_torque_impulse(handle, impulse);
            }
        }
    }

    pub fn apply_torque(&mut self, torque: Vector3) {
        if let Some(space) = self.space() {
            if let Some(handle) = self.handle() {
                space.borrow_mut().apply_torque(handle, torque);
            }
        }
    }

    pub fn apply_central_force(&mut self, force: Vector3) {
        if let Some(space) = self.space() {
            if let Some(handle) = self.handle() {
                space.borrow_mut().apply_central_force(handle, force);
            }
        }
    }
    pub fn apply_force(&mut self, force: Vector3, position: Vector3) {
        if let Some(space) = self.space() {
            if let Some(handle) = self.handle() {
                space.borrow_mut().apply_force(handle, force, position);
            }
        }
    }
    pub fn add_constant_force(&mut self, force: Vector3, position: Vector3) {
        if let Some(space) = self.space() {
            if let Some(handle) = self.handle() {
                if let Some(body) = space.borrow().get_body(handle) {
                    let center_of_mass = body.center_of_mass();
                    let translation = body.translation();
                    let center_of_mass_relative = center_of_mass - translation;

                    let point = godot_vector_to_rapier_point(position);
                    let force = godot_vector_to_rapier_vector(force);

                    self.constant_force += force;
                    self.constant_torque += (point - center_of_mass_relative).cross(&force);
                }
            }
        }
    }
    pub fn add_constant_central_force(&mut self, force: Vector3) {
        self.constant_force += godot_vector_to_rapier_vector(force);
    }
    pub fn add_constant_torque(&mut self, torque: Vector3) {
        self.constant_torque += godot_vector_to_rapier_vector(torque);
    }
    pub fn set_constant_force(&mut self, force: Vector3) {
        self.constant_force = godot_vector_to_rapier_vector(force);
    }
    pub fn set_constant_torque(&mut self, torque: Vector3) {
        self.constant_torque = godot_vector_to_rapier_vector(torque);
    }

    pub fn get_constant_force_godot(&self) -> Vector3 {
        rapier_vector_to_godot_vector(self.constant_force)
    }
    pub fn get_constant_torque_godot(&self) -> Vector3 {
        rapier_vector_to_godot_vector(self.constant_torque)
    }
    pub const fn get_constant_force(&self) -> Vector<f32> {
        self.constant_force
    }
    pub const fn get_constant_torque(&self) -> Vector<f32> {
        self.constant_torque
    }

    pub fn handle(&self) -> Option<RigidBodyHandle> {
        if self.handle.is_none() {
            godot_error!("{}", RapierError::BodyHandleNotSet(self.rid));
        }
        self.handle
    }
}
