use std::{cell::RefCell, rc::Rc};

use godot::{engine::physics_server_3d::BodyMode, prelude::*};
use rapier3d::prelude::*;

use crate::{
    collision_object::RapierCollisionObject,
    conversions::{body_mode_to_body_type, transform_to_isometry},
    shape::{RapierShape, RapierShapeInstance},
    space::RapierSpace,
};

#[allow(clippy::module_name_repetitions)]
pub struct RapierBody {
    space: Option<Rc<RefCell<RapierSpace>>>,
    handle: Option<RigidBodyHandle>,
    shapes: Vec<RapierShapeInstance>,
    body_mode: BodyMode,
    instance_id: Option<u64>,
    ccd_enabled: bool,
    body_state_callback: Callable,
    constant_force: Vector<f32>,
    constant_torque: Vector<f32>,
}

impl RapierCollisionObject for RapierBody {
    fn set_space(&mut self, space: Rc<RefCell<RapierSpace>>) {
        self.space = Some(space);
    }

    fn get_space(&self) -> Option<Rc<RefCell<RapierSpace>>> {
        self.space.clone()
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
        self.update_shapes();
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
        self.update_shapes();
    }
    fn remove_nth_shape(&mut self, idx: usize) {
        self.shapes.swap_remove(idx);
        self.update_shapes();
    }

    fn clear_shapes(&mut self) {
        self.shapes.clear();
        self.update_shapes();
    }

    fn set_shape(&mut self, idx: usize, s: Rc<RefCell<dyn RapierShape>>) {
        if let Some(shape) = self.shapes.get_mut(idx) {
            shape.shape = s.clone();
            self.update_shapes();
        }
    }
    fn set_shape_transform(&mut self, idx: usize, transform: Transform3D) {
        if let Some(shape) = self.shapes.get_mut(idx) {
            shape.isometry = transform_to_isometry(&transform);
            self.update_shapes();
        }
    }

    fn set_shape_disabled(&mut self, idx: usize, disabled: bool) {
        if let Some(shape) = self.shapes.get_mut(idx) {
            shape.disabled = disabled;
            self.update_shapes();
        }
    }
}

impl RapierBody {
    pub fn new() -> Self {
        Self {
            space: None,
            handle: None,
            shapes: vec![],
            body_mode: BodyMode::BODY_MODE_STATIC,
            instance_id: None,
            body_state_callback: Callable::invalid(),
            ccd_enabled: false,
            constant_force: vector![0.0, 0.0, 0.0],
            constant_torque: vector![0.0, 0.0, 0.0],
        }
    }
    pub fn set_body_mode(&mut self, mode: BodyMode) {
        self.body_mode = mode;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                if let Some(body) = space.borrow_mut().get_body_mut(handle) {
                    body.set_body_type(body_mode_to_body_type(mode), true);
                }
            }
        }
    }

    pub fn set_enable_ccd(&mut self, enabled: bool) {
        self.ccd_enabled = enabled;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                if let Some(body) = space.borrow_mut().get_body_mut(handle) {
                    body.enable_ccd(enabled);
                }
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

    pub fn body_state_callback(&self) -> &Callable {
        &self.body_state_callback
    }

    pub fn set_body_state_callback(&mut self, body_state_callback: Callable) {
        self.body_state_callback = body_state_callback;
    }

    fn update_shapes(&mut self) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                if let Some(collider) = space.borrow_mut().get_body_collider_mut(handle) {
                    if let Some(shapes) = self.build_shared_shape() {
                        collider.set_shape(shapes);
                    } else {
                        collider.set_enabled(false);
                    }
                }
            }
        }
    }
    pub fn apply_central_impulse(&mut self, impulse: Vector3) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                if let Some(body) = space.borrow_mut().get_body_mut(handle) {
                    body.apply_impulse(vector![impulse.x, impulse.y, impulse.z], true);
                }
            }
        }
    }
    pub fn apply_impulse(&mut self, impulse: Vector3, position: Vector3) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                if let Some(body) = space.borrow_mut().get_body_mut(handle) {
                    body.apply_impulse_at_point(
                        vector![impulse.x, impulse.y, impulse.z],
                        point![position.x, position.y, position.z],
                        true,
                    );
                }
            }
        }
    }

    pub fn apply_torque_impulse(&mut self, impulse: Vector3) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                if let Some(body) = space.borrow_mut().get_body_mut(handle) {
                    body.apply_torque_impulse(vector![impulse.x, impulse.y, impulse.z], true);
                }
            }
        }
    }

    pub fn apply_torque(&mut self, impulse: Vector3) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                if let Some(body) = space.borrow_mut().get_body_mut(handle) {
                    body.add_torque(vector![impulse.x, impulse.y, impulse.z], true);
                }
            }
        }
    }

    pub fn apply_central_force(&mut self, force: Vector3) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                if let Some(body) = space.borrow_mut().get_body_mut(handle) {
                    body.add_force(vector![force.x, force.y, force.z], true);
                }
            }
        }
    }
    pub fn apply_force(&mut self, force: Vector3, position: Vector3) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                if let Some(body) = space.borrow_mut().get_body_mut(handle) {
                    body.add_force_at_point(
                        vector![force.x, force.y, force.z],
                        point![position.x, position.y, position.z],
                        true,
                    );
                }
            }
        }
    }
    pub fn add_constant_force(&mut self, force: Vector3, point: Vector3) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                if let Some(body) = space.borrow_mut().get_body_mut(handle) {
                    let center_of_mass = body.center_of_mass();
                    let position = body.translation();
                    let center_of_mass_relative = center_of_mass - position;

                    let point = point![point.x, point.y, point.z];
                    let force = vector![force.x, force.y, force.z];

                    self.constant_force += force;
                    self.constant_torque += (point - center_of_mass_relative).cross(&force);
                }
            }
        }
    }
    pub fn add_constant_central_force(&mut self, force: Vector3) {
        self.constant_force += vector![force.x, force.y, force.z];
    }
    pub fn add_constant_torque(&mut self, torque: Vector3) {
        self.constant_torque += vector![torque.x, torque.y, torque.z];
    }
    pub fn set_constant_force(&mut self, force: Vector3) {
        self.constant_force = vector![force.x, force.y, force.z];
    }
    pub fn set_constant_torque(&mut self, torque: Vector3) {
        self.constant_torque = vector![torque.x, torque.y, torque.z];
    }

    pub fn get_constant_force_godot(&self) -> Vector3 {
        Vector3::new(
            self.constant_force.x,
            self.constant_force.y,
            self.constant_force.z,
        )
    }
    pub fn get_constant_torque_godot(&self) -> Vector3 {
        Vector3::new(
            self.constant_torque.x,
            self.constant_torque.y,
            self.constant_torque.z,
        )
    }
    pub fn get_constant_force(&self) -> Vector<f32> {
        self.constant_force
    }
    pub fn get_constant_torque(&self) -> Vector<f32> {
        self.constant_torque
    }
}
