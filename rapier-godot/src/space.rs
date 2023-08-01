use std::{cell::RefCell, collections::HashMap, rc::Rc};

use godot::{
    engine::{physics_server_3d::BodyMode, physics_server_3d::SpaceParameter},
    prelude::*,
};
use rapier3d::prelude::*;

use crate::{
    area::RapierArea, body::RapierBody, collision_object::RapierCollisionObject,
    conversions::body_mode_to_body_type, conversions::isometry_to_transform,
};

#[allow(clippy::module_name_repetitions)]
pub struct RapierSpace {
    rid: Rid,
    godot_bodies: HashMap<RigidBodyHandle, Rc<RefCell<RapierBody>>>,

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

#[allow(clippy::default_trait_access)]
impl RapierSpace {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            godot_bodies: Default::default(),
            rigid_body_set: Default::default(),
            collider_set: Default::default(),
            gravity: Default::default(),
            integration_parameters: Default::default(),
            physics_pipeline: Default::default(),
            island_manager: Default::default(),
            broad_phase: Default::default(),
            narrow_phase: Default::default(),
            impulse_joint_set: Default::default(),
            multibody_joint_set: Default::default(),
            ccd_solver: Default::default(),
            physics_hooks: Default::default(),
            event_handler: Default::default(),
        }
    }

    pub fn call_queries(&mut self) {}

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

    pub fn get_area(&self, handle: ColliderHandle) -> Option<&Collider> {
        self.collider_set.get(handle)
    }

    pub fn get_area_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        self.collider_set.get_mut(handle)
    }

    pub fn get_body(&self, handle: RigidBodyHandle) -> Option<&RigidBody> {
        self.rigid_body_set.get(handle)
    }

    pub fn get_body_mut(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody> {
        self.rigid_body_set.get_mut(handle)
    }

    pub fn get_body_collider(&self, handle: RigidBodyHandle) -> Option<&Collider> {
        if let Some(body) = self.get_body(handle) {
            let coll_handle = body.colliders()[0];
            return self.collider_set.get(coll_handle);
        }
        None
    }

    pub fn get_body_collider_mut(&mut self, handle: RigidBodyHandle) -> Option<&mut Collider> {
        if let Some(body) = self.get_body_mut(handle) {
            let coll_handle = body.colliders()[0];
            return self.collider_set.get_mut(coll_handle);
        }
        None
    }

    pub fn set_param(&mut self, param: SpaceParameter, value: f32) {
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

    pub fn get_param(&self, param: SpaceParameter) -> f32 {
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
        let area_borrow = area.borrow();
        let collider = area_borrow.build_collider(true);
        let handle = self.collider_set.insert(collider);
        area.borrow_mut().set_handle(handle);
    }

    pub fn add_body(&mut self, body: &Rc<RefCell<RapierBody>>) {
        let body_borrow = body.borrow();
        let collider = body_borrow.build_collider(false);
        let body_type = body_mode_to_body_type(body.borrow().get_body_mode());
        let handle = self.rigid_body_set.insert(RigidBodyBuilder::new(body_type));
        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        body.borrow_mut().set_handle(handle);
        self.godot_bodies.insert(handle, body.clone());
    }

    pub const fn rid(&self) -> Rid {
        self.rid
    }
}
