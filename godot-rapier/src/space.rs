use std::{cell::RefCell, collections::HashMap, rc::Rc};

use godot::{
    engine::{physics_server_3d::BodyMode, physics_server_3d::SpaceParameter},
    prelude::*,
};
use rapier3d::prelude::*;

use crate::{
    area::RapierArea,
    body::RapierBody,
    collision_object::RapierCollisionObject,
    conversions::{
        body_mode_to_body_type, godot_vector_to_rapier_point, godot_vector_to_rapier_vector,
        transform_to_isometry,
    },
};

pub struct RapierSpace {
    rid: Rid,
    bodies: HashMap<RigidBodyHandle, Rc<RefCell<RapierBody>>>,
    areas: HashMap<ColliderHandle, Rc<RefCell<RapierArea>>>,
    default_area: Option<Rc<RefCell<RapierArea>>>,

    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    gravity: Vector<Real>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    physics_hooks: (),
    event_handler: (),
}

const DEFAULT_CONTACT_RECYCLE_RADIUS: f32 = 0.01;
const DEFAULT_CONTACT_MAX_SEPARATION: f32 = 0.05;
const DEFAULT_CONTACT_MAX_ALLOWED_PENETRATION: f32 = 0.01;
const DEFAULT_CONTACT_DEFAULT_BIAS: f32 = 0.8;
const DEFAULT_SLEEP_THRESHOLD_LINEAR: f32 = 0.1;
const DEFAULT_SLEEP_THRESHOLD_ANGULAR: f32 = 8.0 * std::f32::consts::PI / 180.0;
const DEFAULT_SOLVER_ITERATIONS: u32 = 8;

impl RapierSpace {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            bodies: HashMap::default(),
            areas: HashMap::default(),
            default_area: None,
            rigid_body_set: RigidBodySet::default(),
            collider_set: ColliderSet::default(),
            gravity: Vector::default(),
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::default(),
            island_manager: IslandManager::default(),
            broad_phase: BroadPhase::default(),
            narrow_phase: NarrowPhase::default(),
            impulse_joint_set: ImpulseJointSet::default(),
            multibody_joint_set: MultibodyJointSet::default(),
            ccd_solver: CCDSolver::default(),
            physics_hooks: Default::default(),
            event_handler: Default::default(),
        }
    }

    pub fn call_queries(&mut self) {
        for body in self.bodies.values() {
            body.borrow_mut().call_queries();
        }
        for area in self.areas.values() {
            area.borrow_mut().call_queries();
        }
    }

    fn pre_step(&self) {
        for body in self.bodies.values() {
            body.borrow_mut().pre_step(self.integration_parameters.dt);
        }
    }

    pub fn step(&mut self) {
        self.pre_step();
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,
            &self.physics_hooks,
            &self.event_handler,
        );
    }
    pub fn set_area_collision_group(
        &mut self,
        handle: ColliderHandle,
        collision_layer: u32,
        collision_mask: u32,
    ) {
        if let Some(area_collider) = self.collider_set.get_mut(handle) {
            area_collider.set_collision_groups(InteractionGroups::new(
                Group::from(collision_layer),
                Group::from(collision_mask),
            ));
        }
    }

    pub fn set_body_collision_group(
        &mut self,
        handle: RigidBodyHandle,
        collision_layer: u32,
        collision_mask: u32,
    ) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            if let Some(collider) = self.collider_set.get_mut(body.colliders()[0]) {
                collider.set_collision_groups(InteractionGroups::new(
                    Group::from(collision_layer),
                    Group::from(collision_mask),
                ));
            }
        }
    }

    pub fn set_area_isometry(&mut self, handle: ColliderHandle, isometry: Isometry<f32>) {
        if let Some(area_collider) = self.collider_set.get_mut(handle) {
            area_collider.set_position(isometry);
        }
    }

    pub fn update_area_shape(&mut self, handle: ColliderHandle, shape: Option<SharedShape>) {
        if let Some(area_collider) = self.collider_set.get_mut(handle) {
            match shape {
                Some(shape) => area_collider.set_shape(shape),
                None => area_collider.set_enabled(false),
            }
        }
    }
    pub fn update_body_shape(&mut self, handle: RigidBodyHandle, shape: Option<SharedShape>) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            if let Some(collider) = self.collider_set.get_mut(body.colliders()[0]) {
                match shape {
                    Some(shape) => collider.set_shape(shape),
                    None => collider.set_enabled(false),
                }
            }
        }
    }

    pub fn set_body_mode(&mut self, handle: RigidBodyHandle, mode: BodyMode) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_body_type(body_mode_to_body_type(mode), true);
        }
    }

    pub fn set_ccd_enabled(&mut self, handle: RigidBodyHandle, enabled: bool) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.enable_ccd(enabled);
        }
    }

    pub fn apply_central_impulse(&mut self, handle: RigidBodyHandle, impulse: Vector<f32>) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.apply_impulse(impulse, true);
        }
    }
    pub fn apply_impulse(
        &mut self,
        handle: RigidBodyHandle,
        impulse: Vector<f32>,
        position: Point<f32>,
    ) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.apply_impulse_at_point(impulse, position, true);
        }
    }
    pub fn apply_torque_impulse(&mut self, handle: RigidBodyHandle, impulse: Vector<f32>) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.apply_torque_impulse(impulse, true);
        }
    }
    pub fn apply_torque(&mut self, handle: RigidBodyHandle, torque: Vector<f32>) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.add_torque(torque, true);
        }
    }
    pub fn apply_central_force(&mut self, handle: RigidBodyHandle, force: Vector<f32>) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.add_force(force, true);
        }
    }
    pub fn apply_force(
        &mut self,
        handle: RigidBodyHandle,
        force: Vector<f32>,
        position: Point<f32>,
    ) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.add_force_at_point(force, position, true);
        }
    }

    pub fn move_kinematic(&mut self, handle: RigidBodyHandle, new_pos: Isometry<f32>) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_next_kinematic_position(new_pos);
        }
    }

    pub fn set_bounce(&mut self, handle: RigidBodyHandle, value: f32) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            if let Some(collider) = self.collider_set.get_mut(body.colliders()[0]) {
                collider.set_restitution(value);
            }
        }
    }
    pub fn set_friction(&mut self, handle: RigidBodyHandle, value: f32) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            if let Some(collider) = self.collider_set.get_mut(body.colliders()[0]) {
                collider.set_friction(value);
            }
        }
    }
    pub fn set_inertia(&mut self, handle: RigidBodyHandle, inertia: Vector3) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            if let Some(collider) = self.collider_set.get_mut(body.colliders()[0]) {
                let mp = collider.mass_properties();
                collider.set_mass_properties(MassProperties::new(
                    mp.local_com,
                    body.mass(),
                    godot_vector_to_rapier_vector(inertia),
                ));
            }
        }
    }
    pub fn set_custom_center_of_mass(&mut self, handle: RigidBodyHandle, center_of_mass: Vector3) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            if let Some(collider) = self.collider_set.get_mut(body.colliders()[0]) {
                let mp = collider.mass_properties();
                collider.set_mass_properties(MassProperties::new(
                    godot_vector_to_rapier_point(center_of_mass),
                    body.mass(),
                    mp.principal_inertia(),
                ));
            }
        }
    }

    pub fn set_mass(&mut self, handle: RigidBodyHandle, value: f32, custom_mass_props: bool) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            if let Some(collider) = self.collider_set.get_mut(body.colliders()[0]) {
                if custom_mass_props {
                    let mp = collider.mass_properties();
                    collider.set_mass_properties(MassProperties::new(
                        mp.local_com,
                        value,
                        mp.principal_inertia(),
                    ));
                } else {
                    collider.set_mass(value);
                }
            }
        }
    }
    pub fn set_gravity_scale(&mut self, handle: RigidBodyHandle, value: f32) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_gravity_scale(value, true);
        }
    }
    pub fn set_linear_damp(&mut self, handle: RigidBodyHandle, value: f32) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_linear_damping(value);
        }
    }
    pub fn set_angular_damp(&mut self, handle: RigidBodyHandle, value: f32) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_angular_damping(value);
        }
    }
    pub fn set_transform(&mut self, handle: RigidBodyHandle, value: Transform3D) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            let (isometry, _) = transform_to_isometry(&value);
            body.set_position(isometry, true);
        }
    }
    pub fn set_linear_velocity(&mut self, handle: RigidBodyHandle, value: Vector<f32>) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_linvel(value, true);
        }
    }
    pub fn set_angular_velocity(&mut self, handle: RigidBodyHandle, value: Vector<f32>) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.set_angvel(value, true);
        }
    }
    pub fn set_is_sleeping(&mut self, handle: RigidBodyHandle, value: bool) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            if value {
                body.sleep();
            } else {
                body.wake_up(true);
            }
        }
    }
    pub fn set_can_sleep(&mut self, handle: RigidBodyHandle, value: bool) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            let act = body.activation_mut();
            *act = if value {
                if act.is_active() {
                    RigidBodyActivation::active()
                } else {
                    RigidBodyActivation::inactive()
                }
            } else {
                RigidBodyActivation::cannot_sleep()
            }
        }
    }

    pub fn get_body(&self, handle: RigidBodyHandle) -> Option<&RigidBody> {
        self.rigid_body_set.get(handle)
    }

    pub fn set_param(&mut self, param: SpaceParameter, value: f32) {
        // TODO
        match param {
            SpaceParameter::SPACE_PARAM_CONTACT_RECYCLE_RADIUS => {
                godot_warn!(
                    "Space-specific contact recycle radius is not supported by Godot Rapier. 
                    Any such value will be ignored."
                );
            }
            SpaceParameter::SPACE_PARAM_CONTACT_MAX_SEPARATION => {
                godot_warn!(
                    "Space-specific contact max separation is not supported by Godot Rapier. 
                Any such value will be ignored."
                );
            }
            SpaceParameter::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION => {
                godot_warn!(
                    "Space-specific contact max allowed penetration is not supported by Godot Rapier. 
                    Any such value will be ignored."
                );
            }
            SpaceParameter::SPACE_PARAM_CONTACT_DEFAULT_BIAS => {
                godot_warn!(
                    "Space-specific contact default bias is not supported by Godot Rapier. 
                    Any such value will be ignored."
                );
            }
            SpaceParameter::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD => {
                godot_warn!(
                    "Space-specific linear velocity sleep threshold is not supported by Godot Rapier. 
                    Any such value will be ignored."
                );
            }
            SpaceParameter::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD => {
                godot_warn!(
                    "Space-specific angular velocity sleep threshold is not supported by Godot Rapier. 
                    Any such value will be ignored."
                );
            }
            SpaceParameter::SPACE_PARAM_BODY_TIME_TO_SLEEP => {
                godot_warn!(
                    "Space-specific body sleep time is not supported by Godot Rapier. 
                    Any such value will be ignored."
                );
            }
            SpaceParameter::SPACE_PARAM_SOLVER_ITERATIONS => {
                godot_warn!(
                    "Space-specific solver iterations is not supported by Godot Rapier. 
                    Any such value will be ignored."
                );
            }
            _ => {
                godot_error!("Unhandled space parameter: {:?}", param);
            }
        }
    }

    pub const fn get_param(&self, param: SpaceParameter) -> f32 {
        // TODO
        match param {
            SpaceParameter::SPACE_PARAM_CONTACT_RECYCLE_RADIUS => DEFAULT_CONTACT_RECYCLE_RADIUS,
            SpaceParameter::SPACE_PARAM_CONTACT_MAX_SEPARATION => DEFAULT_CONTACT_MAX_SEPARATION,
            SpaceParameter::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION => {
                DEFAULT_CONTACT_MAX_ALLOWED_PENETRATION
            }
            SpaceParameter::SPACE_PARAM_CONTACT_DEFAULT_BIAS => DEFAULT_CONTACT_DEFAULT_BIAS,
            SpaceParameter::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD => {
                DEFAULT_SLEEP_THRESHOLD_LINEAR
            }
            SpaceParameter::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD => {
                DEFAULT_SLEEP_THRESHOLD_ANGULAR
            }
            SpaceParameter::SPACE_PARAM_BODY_TIME_TO_SLEEP => 0.5,
            SpaceParameter::SPACE_PARAM_SOLVER_ITERATIONS => DEFAULT_SOLVER_ITERATIONS as f32,
            _ => 0.0,
        }
    }

    pub fn add_area(&mut self, area: &Rc<RefCell<RapierArea>>) {
        let mut area_borrow = area.borrow_mut();
        let collider = area_borrow.build_collider().sensor(true);
        let handle = self.collider_set.insert(collider);
        area_borrow.set_handle(handle);
        self.areas.insert(handle, area.clone());
    }

    pub fn set_default_area(&mut self, area: &Rc<RefCell<RapierArea>>) {
        let mut area_borrow = area.borrow_mut();
        let collider = area_borrow.build_collider().sensor(true);
        let handle = self.collider_set.insert(collider);
        area_borrow.set_handle(handle);
        self.default_area = Some(area.clone());
    }

    pub fn remove_area(&mut self, handle: ColliderHandle) {
        self.collider_set.remove(
            handle,
            &mut self.island_manager,
            &mut self.rigid_body_set,
            false,
        );
        self.areas.remove(&handle);
    }

    pub fn remove_body(&mut self, handle: RigidBodyHandle) {
        self.rigid_body_set.remove(
            handle,
            &mut self.island_manager,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            true,
        );
        self.bodies.remove(&handle);
    }

    pub fn add_body(&mut self, body: &Rc<RefCell<RapierBody>>) {
        let mut b = body.borrow_mut();
        let mut collider = b
            .build_collider()
            .restitution(b.bounce())
            .friction(b.friction())
            .mass(b.mass())
            .build();

        if b.has_custom_center_of_mass() || b.inertia() != Vector3::ZERO {
            let mp = collider.mass_properties();
            let center_of_mass = if b.has_custom_center_of_mass() {
                godot_vector_to_rapier_point(b.custom_center_of_mass())
            } else {
                mp.local_com
            };
            let inertia = if b.inertia() == Vector3::ZERO {
                mp.principal_inertia()
            } else {
                godot_vector_to_rapier_vector(b.inertia())
            };

            collider.set_mass_properties(MassProperties::new(center_of_mass, b.mass(), inertia));
        }

        let body_type = body_mode_to_body_type(b.body_mode());
        let rigid_body = RigidBodyBuilder::new(body_type)
            .ccd_enabled(b.is_ccd_enabled())
            .linear_damping(b.linear_damp())
            .angular_damping(b.angular_damp())
            .gravity_scale(b.gravity_scale())
            .position(b.isometry())
            .linvel(godot_vector_to_rapier_vector(b.linear_velocity()))
            .angvel(godot_vector_to_rapier_vector(b.angular_velocity()))
            .can_sleep(b.can_sleep())
            .sleeping(b.is_sleeping());

        let handle = self.rigid_body_set.insert(rigid_body);
        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        b.set_handle(handle);
        self.bodies.insert(handle, body.clone());
    }
    pub fn remove_space_from_bodies_areas(&mut self) {
        for area in self.areas.values() {
            area.borrow_mut().remove_space();
        }
        for body in self.bodies.values() {
            body.borrow_mut().remove_space();
        }
    }

    pub const fn rid(&self) -> Rid {
        self.rid
    }

    pub const fn get_step(&self) -> f32 {
        self.integration_parameters.dt
    }

    pub const fn default_area(&self) -> Option<&Rc<RefCell<RapierArea>>> {
        self.default_area.as_ref()
    }
}
