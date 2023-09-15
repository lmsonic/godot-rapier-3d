use godot::{
    engine::{
        physics_server_3d::{BodyDampMode, BodyMode, BodyParameter, BodyState},
        PhysicsDirectBodyState3DExtensionVirtual, PhysicsDirectSpaceState3D,
    },
    prelude::*,
};

use crate::{collision_object::RapierCollisionObject, shapes::ShapeInstance, space::RapierSpace};

#[derive(GodotClass)]
#[class(base=PhysicsDirectBodyState3DExtension)]
pub struct RapierBody {
    rid: Rid,
    space: Option<Gd<RapierSpace>>,
    body_mode: BodyMode,
    shapes: Vec<ShapeInstance>,
    instance_id: Option<u64>,
    collision_layer: u32,
    collision_mask: u32,
    ccd_enabled: bool,
    priority: f32,
    user_flags: u32,
    params: BodyParams,
    state: State,
    constant_force: Vector3,
    constant_torque: Vector3,
}

struct State {
    transform: Transform3D,
    linear_velocity: Vector3,
    angular_velocity: Vector3,
    is_sleeping: bool,
    can_sleep: bool,
}

impl Default for State {
    fn default() -> Self {
        Self {
            transform: Transform3D::default(),
            linear_velocity: Vector3::default(),
            angular_velocity: Vector3::default(),
            is_sleeping: Default::default(),
            can_sleep: true,
        }
    }
}
impl State {
    fn get(&self, param: BodyState) -> Variant {
        match param {
            BodyState::BODY_STATE_TRANSFORM => self.transform.to_variant(),
            BodyState::BODY_STATE_LINEAR_VELOCITY => self.linear_velocity.to_variant(),
            BodyState::BODY_STATE_ANGULAR_VELOCITY => self.angular_velocity.to_variant(),
            BodyState::BODY_STATE_SLEEPING => self.is_sleeping.to_variant(),
            BodyState::BODY_STATE_CAN_SLEEP => self.can_sleep.to_variant(),
            _ => Variant::default(),
        }
    }

    fn set(&mut self, param: BodyState, value: Variant) {
        match param {
            BodyState::BODY_STATE_TRANSFORM => self.transform = value.to(),
            BodyState::BODY_STATE_LINEAR_VELOCITY => self.linear_velocity = value.to(),
            BodyState::BODY_STATE_ANGULAR_VELOCITY => self.angular_velocity = value.to(),
            BodyState::BODY_STATE_SLEEPING => self.is_sleeping = value.to(),
            BodyState::BODY_STATE_CAN_SLEEP => self.can_sleep = value.to(),
            _ => {}
        }
    }
}

struct BodyParams {
    bounce: f32,
    friction: f32,
    mass: f32,
    inertia: Vector3,
    center_of_mass_local: Vector3,
    gravity_scale: f32,
    linear_damp_mode: BodyDampMode,
    angular_damp_mode: BodyDampMode,
    linear_damp: f32,
    angular_damp: f32,
}

impl Default for BodyParams {
    fn default() -> Self {
        Self {
            bounce: Default::default(),
            friction: Default::default(),
            mass: 1.0,
            inertia: Default::default(),
            center_of_mass_local: Vector3::default(),
            gravity_scale: 1.0,
            linear_damp_mode: BodyDampMode::BODY_DAMP_MODE_COMBINE,
            angular_damp_mode: BodyDampMode::BODY_DAMP_MODE_COMBINE,
            linear_damp: Default::default(),
            angular_damp: Default::default(),
        }
    }
}

impl BodyParams {
    fn get(&self, param: BodyParameter) -> Variant {
        match param {
            BodyParameter::BODY_PARAM_BOUNCE => self.bounce.to_variant(),
            BodyParameter::BODY_PARAM_FRICTION => self.friction.to_variant(),
            BodyParameter::BODY_PARAM_MASS => self.mass.to_variant(),
            BodyParameter::BODY_PARAM_INERTIA => self.inertia.to_variant(),
            BodyParameter::BODY_PARAM_CENTER_OF_MASS => self.center_of_mass_local.to_variant(),
            BodyParameter::BODY_PARAM_GRAVITY_SCALE => self.gravity_scale.to_variant(),
            BodyParameter::BODY_PARAM_LINEAR_DAMP_MODE => self.linear_damp_mode.to_variant(),
            BodyParameter::BODY_PARAM_ANGULAR_DAMP_MODE => self.angular_damp_mode.to_variant(),
            BodyParameter::BODY_PARAM_LINEAR_DAMP => self.linear_damp.to_variant(),
            BodyParameter::BODY_PARAM_ANGULAR_DAMP => self.angular_damp.to_variant(),
            _ => Variant::default(),
        }
    }

    fn set(&mut self, param: BodyParameter, value: &Variant) {
        match param {
            BodyParameter::BODY_PARAM_BOUNCE => self.bounce = value.to(),
            BodyParameter::BODY_PARAM_FRICTION => self.friction = value.to(),
            BodyParameter::BODY_PARAM_MASS => self.mass = value.to(),
            BodyParameter::BODY_PARAM_INERTIA => self.inertia = value.to(),
            BodyParameter::BODY_PARAM_CENTER_OF_MASS => self.center_of_mass_local = value.to(),
            BodyParameter::BODY_PARAM_GRAVITY_SCALE => self.gravity_scale = value.to(),
            BodyParameter::BODY_PARAM_LINEAR_DAMP_MODE => self.linear_damp_mode = value.to(),
            BodyParameter::BODY_PARAM_ANGULAR_DAMP_MODE => self.angular_damp_mode = value.to(),
            BodyParameter::BODY_PARAM_LINEAR_DAMP => self.linear_damp = value.to(),
            BodyParameter::BODY_PARAM_ANGULAR_DAMP => self.angular_damp = value.to(),
            _ => {}
        }
    }
}

impl RapierCollisionObject for RapierBody {
    fn shapes(&self) -> &[ShapeInstance] {
        &self.shapes
    }

    fn shapes_mut(&mut self) -> &mut Vec<ShapeInstance> {
        &mut self.shapes
    }

    fn set_instance_id(&mut self, instance_id: u64) {
        self.instance_id = Some(instance_id);
    }

    fn get_instance_id(&self) -> Option<u64> {
        self.instance_id
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
}

impl RapierBody {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            space: None,
            body_mode: BodyMode::BODY_MODE_STATIC,
            shapes: vec![],
            instance_id: None,
            collision_layer: Default::default(),
            collision_mask: Default::default(),
            ccd_enabled: false,
            priority: 0.0,
            user_flags: 0,
            params: BodyParams::default(),
            state: State::default(),
            constant_force: Vector3::default(),
            constant_torque: Vector3::default(),
        }
    }

    pub const fn get_space(&self) -> Option<&Gd<RapierSpace>> {
        self.space.as_ref()
    }

    pub fn set_space(&mut self, space: Gd<RapierSpace>) {
        self.space = Some(space);
    }

    pub const fn get_body_mode(&self) -> BodyMode {
        self.body_mode
    }

    pub fn set_body_mode(&mut self, body_mode: BodyMode) {
        self.body_mode = body_mode;
    }

    pub const fn get_ccd_enabled(&self) -> bool {
        self.ccd_enabled
    }

    pub fn set_ccd_enabled(&mut self, ccd_enabled: bool) {
        self.ccd_enabled = ccd_enabled;
    }

    pub const fn get_priority(&self) -> f32 {
        self.priority
    }

    pub const fn get_user_flags(&self) -> u32 {
        self.user_flags
    }

    pub fn set_priority(&mut self, priority: f32) {
        self.priority = priority;
    }

    pub fn set_user_flags(&mut self, user_flags: u32) {
        self.user_flags = user_flags;
    }
    pub fn get_param(&self, param: BodyParameter) -> Variant {
        self.params.get(param)
    }
    pub fn set_param(&mut self, param: BodyParameter, value: &Variant) {
        self.params.set(param, value);
    }

    pub fn reset_mass_properties(&mut self) {
        // TODO: recalculate inertia and center of mass
    }

    pub fn get_state(&self, param: BodyState) -> Variant {
        self.state.get(param)
    }
    pub fn set_state(&mut self, param: BodyState, value: Variant) {
        self.state.set(param, value);
    }
    pub fn get_position(&self) -> Vector3 {
        self.state.transform.origin
    }
}

#[godot_api]
impl PhysicsDirectBodyState3DExtensionVirtual for RapierBody {
    fn get_total_gravity(&self) -> Vector3 {
        Vector3::default()
    }

    fn get_total_linear_damp(&self) -> f32 {
        Default::default()
    }

    fn get_total_angular_damp(&self) -> f32 {
        Default::default()
    }

    fn get_center_of_mass(&self) -> Vector3 {
        self.state.transform * self.params.center_of_mass_local
    }

    fn get_center_of_mass_local(&self) -> Vector3 {
        self.params.center_of_mass_local
    }

    fn get_principal_inertia_axes(&self) -> Basis {
        Basis::default()
    }

    fn get_inverse_mass(&self) -> f32 {
        1.0 / self.params.mass
    }

    fn get_inverse_inertia(&self) -> Vector3 {
        self.params.inertia.inverse()
    }

    fn get_inverse_inertia_tensor(&self) -> Basis {
        Basis::default()
    }

    fn set_linear_velocity(&mut self, velocity: Vector3) {
        self.state.linear_velocity = velocity;
    }

    fn get_linear_velocity(&self) -> Vector3 {
        self.state.linear_velocity
    }

    fn set_angular_velocity(&mut self, velocity: Vector3) {
        self.state.angular_velocity = velocity;
    }

    fn get_angular_velocity(&self) -> Vector3 {
        self.state.angular_velocity
    }

    fn set_transform(&mut self, transform: Transform3D) {
        self.state.transform = transform;
    }

    fn get_transform(&self) -> Transform3D {
        self.state.transform
    }

    fn get_velocity_at_local_position(&self, local_position: Vector3) -> Vector3 {
        Vector3::default()
    }

    fn apply_central_impulse(&mut self, impulse: Vector3) {}

    fn apply_impulse(&mut self, impulse: Vector3, position: Vector3) {}

    fn apply_torque_impulse(&mut self, impulse: Vector3) {}

    fn apply_central_force(&mut self, force: Vector3) {}

    fn apply_force(&mut self, force: Vector3, position: Vector3) {}

    fn apply_torque(&mut self, torque: Vector3) {}

    fn add_constant_central_force(&mut self, force: Vector3) {
        self.constant_force += force;
    }

    fn add_constant_force(&mut self, force: Vector3, position: Vector3) {
        let center_of_mass = self.get_center_of_mass();
        let body_position = self.get_position();
        let center_of_mass_relative = center_of_mass - body_position;

        self.constant_force += force;
        self.constant_torque += (position - center_of_mass_relative).cross(force);
    }

    fn add_constant_torque(&mut self, torque: Vector3) {
        self.constant_torque += torque;
    }

    fn set_constant_force(&mut self, force: Vector3) {
        self.constant_force = force;
    }

    fn get_constant_force(&self) -> Vector3 {
        self.constant_force
    }

    fn set_constant_torque(&mut self, torque: Vector3) {
        self.constant_torque = torque;
    }

    fn get_constant_torque(&self) -> Vector3 {
        self.constant_torque
    }

    fn set_sleep_state(&mut self, enabled: bool) {
        self.state.is_sleeping = enabled;
    }

    fn is_sleeping(&self) -> bool {
        self.state.is_sleeping
    }

    fn get_contact_count(&self) -> i32 {
        Default::default()
    }

    fn get_contact_local_position(&self, contact_idx: i32) -> Vector3 {
        Vector3::default()
    }

    fn get_contact_local_normal(&self, contact_idx: i32) -> Vector3 {
        Vector3::default()
    }

    fn get_contact_impulse(&self, contact_idx: i32) -> Vector3 {
        Vector3::default()
    }

    fn get_contact_local_shape(&self, contact_idx: i32) -> i32 {
        Default::default()
    }

    fn get_contact_local_velocity_at_position(&self, contact_idx: i32) -> Vector3 {
        Vector3::default()
    }

    fn get_contact_collider(&self, contact_idx: i32) -> Rid {
        Rid::Invalid
    }

    fn get_contact_collider_position(&self, contact_idx: i32) -> Vector3 {
        Vector3::default()
    }

    fn get_contact_collider_id(&self, contact_idx: i32) -> u64 {
        Default::default()
    }

    fn get_contact_collider_object(&self, contact_idx: i32) -> Option<Gd<Object>> {
        Option::default()
    }

    fn get_contact_collider_shape(&self, contact_idx: i32) -> i32 {
        Default::default()
    }

    fn get_contact_collider_velocity_at_position(&self, contact_idx: i32) -> Vector3 {
        Vector3::default()
    }

    fn get_step(&self) -> f32 {
        Default::default()
    }

    fn integrate_forces(&mut self) {}

    fn get_space_state(&mut self) -> Option<Gd<PhysicsDirectSpaceState3D>> {
        self.space.as_ref().map(|s| s.share().upcast())
    }
}
