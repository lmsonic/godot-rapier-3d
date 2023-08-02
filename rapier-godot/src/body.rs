#![allow(clippy::option_if_let_else)]
use std::{cell::RefCell, rc::Rc};

use godot::{engine::physics_server_3d::BodyMode, prelude::*};
use rapier3d::prelude::*;

use crate::{
    collision_object::RapierCollisionObject, conversions::body_mode_to_body_type,
    shape::RapierShapeInstance, space::RapierSpace, RapierError, RapierResult,
};

#[allow(clippy::module_name_repetitions)]
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
}

impl RapierCollisionObject for RapierBody {
    fn set_space(&mut self, space: Rc<RefCell<RapierSpace>>) {
        self.space = Some(space);
    }

    fn get_space(&self) -> Option<Rc<RefCell<RapierSpace>>> {
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

    fn get_instance_id(&self) -> Option<u64> {
        self.instance_id
    }

    fn rid(&self) -> Rid {
        self.rid
    }
    fn update_shapes(&mut self) {
        let update_shapes = || -> RapierResult<()> {
            let mut space = self
                .space
                .as_ref()
                .ok_or(RapierError::ObjectSpaceNotSet(self.rid))?
                .borrow_mut();
            let handle = self.handle.ok_or(RapierError::BodyHandleNotSet(self.rid))?;
            let collider = space
                .get_body_collider_mut(handle)
                .ok_or(RapierError::BodyHandleInvalid(self.rid))?;

            if let Some(shapes) = self.build_shared_shape() {
                collider.set_shape(shapes);
            } else {
                collider.set_enabled(false);
            }
            Ok(())
        };
        if let Err(e) = update_shapes() {
            godot_error!("{e}");
        }
    }
}

impl RapierBody {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
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
        let set_body_mode = || -> RapierResult<()> {
            let mut space = self
                .space
                .as_ref()
                .ok_or(RapierError::ObjectSpaceNotSet(self.rid))?
                .borrow_mut();
            let handle = self.handle.ok_or(RapierError::BodyHandleNotSet(self.rid))?;
            let body = space
                .get_body_mut(handle)
                .ok_or(RapierError::BodyHandleInvalid(self.rid))?;
            body.set_body_type(body_mode_to_body_type(mode), true);
            Ok(())
        };
        self.body_mode = mode;
        if let Err(e) = set_body_mode() {
            godot_error!("{e}");
        }
    }

    pub fn set_enable_ccd(&mut self, enabled: bool) {
        let set_enable_ccd = || -> RapierResult<()> {
            let mut space = self
                .space
                .as_ref()
                .ok_or(RapierError::ObjectSpaceNotSet(self.rid))?
                .borrow_mut();
            let handle = self.handle.ok_or(RapierError::BodyHandleNotSet(self.rid))?;
            let body = space
                .get_body_mut(handle)
                .ok_or(RapierError::BodyHandleInvalid(self.rid))?;
            body.enable_ccd(enabled);
            Ok(())
        };
        if let Err(e) = set_enable_ccd() {
            godot_error!("{e}");
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
        let apply_central_impulse = || -> RapierResult<()> {
            let mut space = self
                .space
                .as_ref()
                .ok_or(RapierError::ObjectSpaceNotSet(self.rid))?
                .borrow_mut();
            let handle = self.handle.ok_or(RapierError::BodyHandleNotSet(self.rid))?;
            let body = space
                .get_body_mut(handle)
                .ok_or(RapierError::BodyHandleInvalid(self.rid))?;
            body.apply_impulse(vector![impulse.x, impulse.y, impulse.z], true);
            Ok(())
        };
        if let Err(e) = apply_central_impulse() {
            godot_error!("{e}");
        }
    }
    pub fn apply_impulse(&mut self, impulse: Vector3, position: Vector3) {
        let apply_impulse = || -> RapierResult<()> {
            let mut space = self
                .space
                .as_ref()
                .ok_or(RapierError::ObjectSpaceNotSet(self.rid))?
                .borrow_mut();
            let handle = self.handle.ok_or(RapierError::BodyHandleNotSet(self.rid))?;
            let body = space
                .get_body_mut(handle)
                .ok_or(RapierError::BodyHandleInvalid(self.rid))?;
            body.apply_impulse_at_point(
                vector![impulse.x, impulse.y, impulse.z],
                point![position.x, position.y, position.z],
                true,
            );
            Ok(())
        };
        if let Err(e) = apply_impulse() {
            godot_error!("{e}");
        }
    }

    pub fn apply_torque_impulse(&mut self, impulse: Vector3) {
        let apply_torque_impulse = || -> RapierResult<()> {
            let mut space = self
                .space
                .as_ref()
                .ok_or(RapierError::ObjectSpaceNotSet(self.rid))?
                .borrow_mut();
            let handle = self.handle.ok_or(RapierError::BodyHandleNotSet(self.rid))?;
            let body = space
                .get_body_mut(handle)
                .ok_or(RapierError::BodyHandleInvalid(self.rid))?;
            body.apply_torque_impulse(vector![impulse.x, impulse.y, impulse.z], true);

            Ok(())
        };
        if let Err(e) = apply_torque_impulse() {
            godot_error!("{e}");
        }
    }

    pub fn apply_torque(&mut self, impulse: Vector3) {
        let apply_torque = || -> RapierResult<()> {
            let mut space = self
                .space
                .as_ref()
                .ok_or(RapierError::ObjectSpaceNotSet(self.rid))?
                .borrow_mut();
            let handle = self.handle.ok_or(RapierError::BodyHandleNotSet(self.rid))?;
            let body = space
                .get_body_mut(handle)
                .ok_or(RapierError::BodyHandleInvalid(self.rid))?;
            body.add_torque(vector![impulse.x, impulse.y, impulse.z], true);

            Ok(())
        };
        if let Err(e) = apply_torque() {
            godot_error!("{e}");
        }
    }

    pub fn apply_central_force(&mut self, force: Vector3) {
        let apply_central_force = || -> RapierResult<()> {
            let mut space = self
                .space
                .as_ref()
                .ok_or(RapierError::ObjectSpaceNotSet(self.rid))?
                .borrow_mut();
            let handle = self.handle.ok_or(RapierError::BodyHandleNotSet(self.rid))?;
            let body = space
                .get_body_mut(handle)
                .ok_or(RapierError::BodyHandleInvalid(self.rid))?;
            body.add_force(vector![force.x, force.y, force.z], true);
            Ok(())
        };
        if let Err(e) = apply_central_force() {
            godot_error!("{e}");
        }
    }
    pub fn apply_force(&mut self, force: Vector3, position: Vector3) {
        let apply_force = || -> RapierResult<()> {
            let mut space = self
                .space
                .as_ref()
                .ok_or(RapierError::ObjectSpaceNotSet(self.rid))?
                .borrow_mut();
            let handle = self.handle.ok_or(RapierError::BodyHandleNotSet(self.rid))?;
            let body = space
                .get_body_mut(handle)
                .ok_or(RapierError::BodyHandleInvalid(self.rid))?;
            body.add_force_at_point(
                vector![force.x, force.y, force.z],
                point![position.x, position.y, position.z],
                true,
            );
            Ok(())
        };
        if let Err(e) = apply_force() {
            godot_error!("{e}");
        }
    }
    pub fn add_constant_force(&mut self, force: Vector3, point: Vector3) {
        let mut add_constant_force = || -> RapierResult<()> {
            let mut space = self
                .space
                .as_ref()
                .ok_or(RapierError::ObjectSpaceNotSet(self.rid))?
                .borrow_mut();
            let handle = self.handle.ok_or(RapierError::BodyHandleNotSet(self.rid))?;
            let body = space
                .get_body_mut(handle)
                .ok_or(RapierError::BodyHandleInvalid(self.rid))?;
            let center_of_mass = body.center_of_mass();
            let position = body.translation();
            let center_of_mass_relative = center_of_mass - position;

            let point = point![point.x, point.y, point.z];
            let force = vector![force.x, force.y, force.z];

            self.constant_force += force;
            self.constant_torque += (point - center_of_mass_relative).cross(&force);
            Ok(())
        };
        if let Err(e) = add_constant_force() {
            godot_error!("{e}");
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
    pub const fn get_constant_force(&self) -> Vector<f32> {
        self.constant_force
    }
    pub const fn get_constant_torque(&self) -> Vector<f32> {
        self.constant_torque
    }
}
