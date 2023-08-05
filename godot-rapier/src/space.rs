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
        isometry_to_transform, transform_to_isometry,
    },
};

pub struct RapierSpace {
    rid: Rid,
    godot_bodies: HashMap<RigidBodyHandle, Rc<RefCell<RapierBody>>>,
    godot_areas: HashMap<ColliderHandle, Rc<RefCell<RapierArea>>>,
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
            godot_bodies: HashMap::default(),
            godot_areas: HashMap::default(),
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
        // TODO
    }

    pub fn step(&mut self) {
        for (handle, body) in self.rigid_body_set.iter_mut() {
            if let Some(godot_body) = self.godot_bodies.get(&handle) {
                body.add_force(godot_body.borrow().get_constant_force(), true);
                body.add_torque(godot_body.borrow().get_constant_torque(), true);
            }
        }

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

    pub fn set_area_transform(&mut self, handle: ColliderHandle, transform: Transform3D) {
        if let Some(area_collider) = self.collider_set.get_mut(handle) {
            area_collider.set_position(transform_to_isometry(&transform));
        }
    }
    pub fn get_area_transform(&self, handle: ColliderHandle) -> Transform3D {
        self.collider_set
            .get(handle)
            .map_or(Transform3D::IDENTITY, |area_collider| {
                isometry_to_transform(area_collider.position())
            })
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

    pub fn apply_central_impulse(&mut self, handle: RigidBodyHandle, impulse: Vector3) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.apply_impulse(godot_vector_to_rapier_vector(impulse), true);
        }
    }
    pub fn apply_impulse(&mut self, handle: RigidBodyHandle, impulse: Vector3, position: Vector3) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.apply_impulse_at_point(
                godot_vector_to_rapier_vector(impulse),
                godot_vector_to_rapier_point(position),
                true,
            );
        }
    }
    pub fn apply_torque_impulse(&mut self, handle: RigidBodyHandle, impulse: Vector3) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.apply_torque_impulse(godot_vector_to_rapier_vector(impulse), true);
        }
    }
    pub fn apply_torque(&mut self, handle: RigidBodyHandle, torque: Vector3) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.add_torque(godot_vector_to_rapier_vector(torque), true);
        }
    }
    pub fn apply_central_force(&mut self, handle: RigidBodyHandle, force: Vector3) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.add_force(godot_vector_to_rapier_vector(force), true);
        }
    }
    pub fn apply_force(&mut self, handle: RigidBodyHandle, force: Vector3, position: Vector3) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.add_force_at_point(
                godot_vector_to_rapier_vector(force),
                godot_vector_to_rapier_point(position),
                true,
            );
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
        let collider = area_borrow.build_collider(true);
        let handle = self.collider_set.insert(collider);
        area_borrow.set_handle(handle);
        self.godot_areas.insert(handle, area.clone());
    }

    pub fn set_default_area(&mut self, area: &Rc<RefCell<RapierArea>>) {
        let mut area_borrow = area.borrow_mut();
        let collider = area_borrow.build_collider(true);
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
    }

    pub fn add_body(&mut self, body: &Rc<RefCell<RapierBody>>) {
        let mut body_borrow = body.borrow_mut();
        let collider = body_borrow.build_collider(false);
        let body_type = body_mode_to_body_type(body_borrow.get_body_mode());
        let handle = self
            .rigid_body_set
            .insert(RigidBodyBuilder::new(body_type).ccd_enabled(body_borrow.is_ccd_enabled()));
        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        body_borrow.set_handle(handle);
        self.godot_bodies.insert(handle, body.clone());
    }
    pub fn remove_space_from_bodies_areas(&self) {
        for area in self.godot_areas.values() {
            area.borrow_mut().remove_space();
        }
        for body in self.godot_bodies.values() {
            body.borrow_mut().remove_space();
        }
    }

    pub const fn rid(&self) -> Rid {
        self.rid
    }
}
