#![allow(clippy::option_if_let_else)]
use std::{cell::RefCell, rc::Rc};

use godot::{
    engine::{
        physics_server_3d::BodyMode,
        physics_server_3d::{BodyParameter, BodyState},
        rigid_body_3d::DampMode,
    },
    prelude::*,
};
use rapier3d::prelude::*;

use crate::{
    area::RapierArea,
    collision_object::{Handle, RapierCollisionObject},
    conversions::{
        godot_vector_to_rapier_point, godot_vector_to_rapier_vector, isometry_to_transform,
        rapier_point_to_godot_vector, rapier_vector_to_godot_vector, transform_to_isometry,
    },
    error::RapierError,
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
    collision_priority: f32,

    bounce: f32,
    friction: f32,
    mass: f32,
    inertia: Vector3,
    custom_center_of_mass: Vector3,
    has_custom_center_of_mass: bool,
    gravity_scale: f32,
    linear_damp_mode: DampMode,
    angular_damp_mode: DampMode,
    linear_damp: f32,
    angular_damp: f32,

    gravity: Vector3,
    areas: Vec<Rc<RefCell<RapierArea>>>,

    linear_velocity: Vector3,
    angular_velocity: Vector3,
    transform: Transform3D,
    is_sleeping: bool,
    can_sleep: bool,
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
            collision_priority: 1.0,
            bounce: Default::default(),
            friction: Default::default(),
            mass: 1.0,
            inertia: Vector3::default(),
            custom_center_of_mass: Vector3::default(),
            has_custom_center_of_mass: false,
            gravity_scale: 1.0,
            linear_damp_mode: DampMode::DAMP_MODE_COMBINE,
            angular_damp_mode: DampMode::DAMP_MODE_COMBINE,
            linear_damp: Default::default(),
            angular_damp: Default::default(),
            gravity: Vector3::default(),
            areas: Default::default(),
            linear_velocity: Vector3::default(),
            angular_velocity: Vector3::default(),
            transform: Transform3D::IDENTITY,
            is_sleeping: false,
            can_sleep: true,
        }
    }
}

impl RapierCollisionObject for RapierBody {
    fn set_space(&mut self, space: Rc<RefCell<RapierSpace>>) {
        self.space = Some(space);
    }
    fn space(&self) -> Option<Rc<RefCell<RapierSpace>>> {
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

    fn remove_space(&mut self) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                space.borrow_mut().remove_body(handle);
            }
        }
        self.space = None;
        self.handle = None;
    }
    fn set_collision_layer(&mut self, layer: u32) {
        self.collision_layer = layer;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_body_collision_group(
                    handle,
                    self.collision_layer,
                    self.collision_mask,
                );
            }
        }
    }

    fn get_collision_layer(&self) -> u32 {
        self.collision_layer
    }

    fn set_collision_mask(&mut self, mask: u32) {
        self.collision_mask = mask;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_body_collision_group(
                    handle,
                    self.collision_layer,
                    self.collision_mask,
                );
            }
        }
    }

    fn get_collision_mask(&self) -> u32 {
        self.collision_mask
    }

    fn generic_handle(&self) -> Handle {
        self.handle().map_or(Handle::NotSet, Handle::BodyHandle)
    }

    fn isometry(&self) -> Isometry<f32> {
        transform_to_isometry(&self.transform).0
    }

    fn scale(&self) -> Vector<f32> {
        transform_to_isometry(&self.transform).1
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
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_ccd_enabled(handle, enabled);
            }
        }
    }

    pub const fn is_ccd_enabled(&self) -> bool {
        self.ccd_enabled
    }

    pub const fn body_mode(&self) -> BodyMode {
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
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().apply_central_impulse(handle, impulse);
            }
        }
    }
    pub fn apply_impulse(&mut self, impulse: Vector3, position: Vector3) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().apply_impulse(handle, impulse, position);
            }
        }
    }

    pub fn apply_torque_impulse(&mut self, impulse: Vector3) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().apply_torque_impulse(handle, impulse);
            }
        }
    }

    pub fn apply_torque(&mut self, torque: Vector3) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().apply_torque(handle, torque);
            }
        }
    }

    pub fn apply_central_force(&mut self, force: Vector3) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().apply_central_force(handle, force);
            }
        }
    }
    pub fn apply_force(&mut self, force: Vector3, position: Vector3) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().apply_force(handle, force, position);
            }
        }
    }
    pub fn add_constant_force(&mut self, force: Vector3, position: Vector3) {
        if let Some(space) = &self.space {
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

    pub fn constant_force_godot(&self) -> Vector3 {
        rapier_vector_to_godot_vector(self.constant_force)
    }
    pub fn constant_torque_godot(&self) -> Vector3 {
        rapier_vector_to_godot_vector(self.constant_torque)
    }
    pub const fn constant_force(&self) -> Vector<f32> {
        self.constant_force
    }
    pub const fn constant_torque(&self) -> Vector<f32> {
        self.constant_torque
    }

    pub fn handle(&self) -> Option<RigidBodyHandle> {
        if self.handle.is_none() {
            godot_error!("{}", RapierError::BodyHandleNotSet(self.rid));
        }
        self.handle
    }

    pub const fn collision_priority(&self) -> f32 {
        self.collision_priority
    }

    pub fn set_collision_priority(&mut self, priority: f32) {
        self.collision_priority = priority;
    }

    pub fn set_param(&mut self, param: BodyParameter, value: &Variant) {
        match param {
            BodyParameter::BODY_PARAM_BOUNCE => self.set_bounce(value.to()),
            BodyParameter::BODY_PARAM_FRICTION => {
                self.set_friction(value.to());
            }
            BodyParameter::BODY_PARAM_MASS => {
                self.set_mass(value.to());
            }
            BodyParameter::BODY_PARAM_INERTIA => {
                self.set_inertia(value.to());
            }
            BodyParameter::BODY_PARAM_CENTER_OF_MASS => {
                self.set_center_of_mass(value.to());
            }
            BodyParameter::BODY_PARAM_GRAVITY_SCALE => {
                self.set_gravity_scale(value.to());
            }
            BodyParameter::BODY_PARAM_LINEAR_DAMP_MODE => {
                self.linear_damp_mode = value.to();
            }
            BodyParameter::BODY_PARAM_ANGULAR_DAMP_MODE => {
                self.angular_damp_mode = value.to();
            }
            BodyParameter::BODY_PARAM_LINEAR_DAMP => {
                self.set_linear_damp(value.to());
            }
            BodyParameter::BODY_PARAM_ANGULAR_DAMP => {
                self.set_angular_damp(value.to());
            }
            _ => {}
        };
    }
    pub fn get_param(&self, param: BodyParameter) -> Variant {
        match param {
            BodyParameter::BODY_PARAM_BOUNCE => Variant::from(self.bounce),
            BodyParameter::BODY_PARAM_FRICTION => Variant::from(self.friction),
            BodyParameter::BODY_PARAM_MASS => Variant::from(self.mass),
            BodyParameter::BODY_PARAM_INERTIA => Variant::from(self.inertia),
            BodyParameter::BODY_PARAM_CENTER_OF_MASS => Variant::from(self.center_of_mass()),
            BodyParameter::BODY_PARAM_GRAVITY_SCALE => Variant::from(self.gravity_scale),
            BodyParameter::BODY_PARAM_LINEAR_DAMP_MODE => Variant::from(self.linear_damp_mode),
            BodyParameter::BODY_PARAM_ANGULAR_DAMP_MODE => Variant::from(self.angular_damp_mode),
            BodyParameter::BODY_PARAM_LINEAR_DAMP => Variant::from(self.linear_damp),
            BodyParameter::BODY_PARAM_ANGULAR_DAMP => Variant::from(self.angular_damp),
            _ => Variant::nil(),
        }
    }

    pub fn set_state(&mut self, state: BodyState, value: &Variant) {
        match state {
            BodyState::BODY_STATE_TRANSFORM => self.set_transform(value.to()),
            BodyState::BODY_STATE_LINEAR_VELOCITY => self.set_linear_velocity(value.to()),
            BodyState::BODY_STATE_ANGULAR_VELOCITY => self.set_angular_velocity(value.to()),
            BodyState::BODY_STATE_SLEEPING => self.set_is_sleeping(value.to()),
            BodyState::BODY_STATE_CAN_SLEEP => self.set_can_sleep(value.to()),
            _ => {}
        };
    }
    pub fn get_state(&self, state: BodyState) -> Variant {
        match state {
            BodyState::BODY_STATE_TRANSFORM => Variant::from(self.transform()),
            BodyState::BODY_STATE_LINEAR_VELOCITY => Variant::from(self.linear_velocity()),
            BodyState::BODY_STATE_ANGULAR_VELOCITY => Variant::from(self.angular_velocity()),
            BodyState::BODY_STATE_SLEEPING => Variant::from(self.is_sleeping()),
            BodyState::BODY_STATE_CAN_SLEEP => Variant::from(self.can_sleep()),
            _ => Variant::nil(),
        }
    }

    pub fn set_bounce(&mut self, bounce: f32) {
        self.bounce = bounce;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_bounce(handle, bounce);
            }
        }
    }

    pub fn set_friction(&mut self, friction: f32) {
        self.friction = friction;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_friction(handle, friction);
            }
        }
    }

    pub fn set_mass(&mut self, mass: f32) {
        self.mass = mass;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space
                    .borrow_mut()
                    .set_mass(handle, mass, self.has_custom_center_of_mass);
            }
        }
    }

    pub fn set_inertia(&mut self, inertia: Vector3) {
        self.inertia = inertia;

        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                if inertia == Vector3::ZERO {
                    space
                        .borrow_mut()
                        .set_mass(handle, self.mass, self.has_custom_center_of_mass);
                } else {
                    space.borrow_mut().set_inertia(handle, inertia);
                }
            }
        }
    }

    pub fn set_center_of_mass(&mut self, center_of_mass: Vector3) {
        self.custom_center_of_mass = center_of_mass;
        self.has_custom_center_of_mass = true;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space
                    .borrow_mut()
                    .set_custom_center_of_mass(handle, center_of_mass);
            }
        }
    }

    pub fn set_gravity_scale(&mut self, gravity_scale: f32) {
        self.gravity_scale = gravity_scale;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_gravity_scale(handle, gravity_scale);
            }
        }
    }
    pub fn set_linear_damp(&mut self, linear_damp: f32) {
        self.angular_damp = linear_damp;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_linear_damp(handle, linear_damp);
            }
        }
    }

    pub fn set_angular_damp(&mut self, angular_damp: f32) {
        self.angular_damp = angular_damp;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_angular_damp(handle, angular_damp);
            }
        }
    }

    pub fn reset_mass_properties(&mut self) {
        self.inertia = Vector3::ZERO;
        self.custom_center_of_mass = Vector3::ZERO;
        self.has_custom_center_of_mass = false;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_mass(handle, self.mass, false);
            }
        }
    }

    pub fn center_of_mass(&self) -> Vector3 {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                if let Some(body) = space.borrow().get_body(handle) {
                    return rapier_point_to_godot_vector(*body.center_of_mass());
                }
            }
        }
        Vector3::ZERO
    }
    pub fn local_center_of_mass(&self) -> Vector3 {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                if let Some(body) = space.borrow().get_body(handle) {
                    let local_com = body.mass_properties().local_mprops.local_com;
                    return rapier_point_to_godot_vector(local_com);
                }
            }
        }
        Vector3::ZERO
    }
    pub fn principal_inertia_axes(&self) -> Basis {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                if let Some(body) = space.borrow().get_body(handle) {
                    let inertia = body.mass_properties().local_mprops.principal_inertia();
                    return self.transform().basis
                        * Basis::from_diagonal(inertia.x, inertia.y, inertia.z);
                }
            }
        }
        self.transform().basis
            * Basis::from_diagonal(self.inertia.x, self.inertia.y, self.inertia.z)
    }

    pub fn inverse_mass(&self) -> f32 {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                if let Some(body) = space.borrow().get_body(handle) {
                    return body.mass_properties().local_mprops.inv_mass;
                }
            }
        }
        1.0 / self.mass
    }

    pub fn inverse_inertia(&self) -> Vector3 {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                if let Some(body) = space.borrow().get_body(handle) {
                    let inv_inertia = body
                        .mass_properties()
                        .local_mprops
                        .inv_principal_inertia_sqrt;
                    return Vector3::new(inv_inertia.x, inv_inertia.y, inv_inertia.z);
                }
            }
        }
        self.inertia.inverse()
    }
    pub fn inverse_inertia_tensor(&self) -> Basis {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                if let Some(body) = space.borrow().get_body(handle) {
                    let inv_inertia = body
                        .mass_properties()
                        .local_mprops
                        .inv_principal_inertia_sqrt;
                    return Basis::from_diagonal(inv_inertia.x, inv_inertia.y, inv_inertia.z);
                }
            }
        }
        let inv = self.inertia.inverse();
        Basis::from_diagonal(inv.x, inv.y, inv.z)
    }

    pub fn set_transform(&mut self, value: Transform3D) {
        self.transform = value;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_transform(handle, value);
            }
        }
    }

    pub fn set_linear_velocity(&mut self, value: Vector3) {
        self.linear_velocity = value;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_linear_velocity(handle, value);
            }
        }
    }

    pub fn set_angular_velocity(&mut self, value: Vector3) {
        self.angular_velocity = value;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_angular_velocity(handle, value);
            }
        }
    }

    pub fn set_is_sleeping(&mut self, value: bool) {
        self.is_sleeping = value;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_is_sleeping(handle, value);
            }
        }
    }

    pub fn set_can_sleep(&mut self, value: bool) {
        self.can_sleep = value;
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_can_sleep(handle, value);
            }
        }
    }

    pub fn transform(&self) -> Transform3D {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                if let Some(body) = space.borrow().get_body(handle) {
                    return isometry_to_transform(body.position());
                }
            }
        }
        self.transform
    }

    pub fn linear_velocity(&self) -> Vector3 {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                if let Some(body) = space.borrow().get_body(handle) {
                    return rapier_vector_to_godot_vector(*body.linvel());
                }
            }
        }
        self.linear_velocity
    }
    pub fn angular_velocity(&self) -> Vector3 {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                if let Some(body) = space.borrow().get_body(handle) {
                    return rapier_vector_to_godot_vector(*body.angvel());
                }
            }
        }
        self.angular_velocity
    }
    pub fn is_sleeping(&self) -> bool {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                if let Some(body) = space.borrow().get_body(handle) {
                    return body.is_sleeping();
                }
            }
        }
        self.is_sleeping
    }
    pub const fn can_sleep(&self) -> bool {
        self.can_sleep
    }

    pub fn pre_step(&mut self, step: f32) {
        match self.body_mode {
            BodyMode::BODY_MODE_RIGID | BodyMode::BODY_MODE_RIGID_LINEAR => {
                self.integrate_forces(step);
            }
            BodyMode::BODY_MODE_KINEMATIC => {
                self.move_kinematic(step);
            }
            _ => {}
        };
    }

    fn integrate_forces(&mut self, step: f32) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle() {
                let s = space.borrow_mut();
                if let Some(body) = s.get_body(handle) {
                    self.gravity = Vector3::ZERO;
                    let position = rapier_vector_to_godot_vector(*body.translation());

                    // TODO
                    for area in &self.areas {}
                }
            }
        }
    }

    fn move_kinematic(&mut self, step: f32) {
        // TODO
    }

    pub const fn bounce(&self) -> f32 {
        self.bounce
    }

    pub const fn friction(&self) -> f32 {
        self.friction
    }

    pub const fn mass(&self) -> f32 {
        self.mass
    }

    pub const fn inertia(&self) -> Vector3 {
        self.inertia
    }

    pub const fn custom_center_of_mass(&self) -> Vector3 {
        self.custom_center_of_mass
    }

    pub const fn has_custom_center_of_mass(&self) -> bool {
        self.has_custom_center_of_mass
    }

    pub const fn gravity_scale(&self) -> f32 {
        self.gravity_scale
    }

    pub const fn linear_damp_mode(&self) -> DampMode {
        self.linear_damp_mode
    }

    pub const fn angular_damp_mode(&self) -> DampMode {
        self.angular_damp_mode
    }

    pub const fn linear_damp(&self) -> f32 {
        self.linear_damp
    }

    pub const fn angular_damp(&self) -> f32 {
        self.angular_damp
    }
}
