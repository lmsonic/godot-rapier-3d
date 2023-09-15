use godot::{
    engine::{
        native::{
            PhysicsServer3DExtensionRayResult, PhysicsServer3DExtensionShapeRestInfo,
            PhysicsServer3DExtensionShapeResult,
        },
        physics_server_3d::SpaceParameter,
        PhysicsDirectSpaceState3DExtensionVirtual, ProjectSettings,
    },
    prelude::*,
};
use rapier3d::prelude::*;

use crate::{area::RapierArea, body::RapierBody, utils::IntoExt};

#[derive(GodotClass)]
#[class(base=PhysicsDirectSpaceState3DExtension)]
pub struct RapierSpace {
    rid: Rid,
    params: SpaceParams,
    default_area: Rid,

    rigidbody_set: RigidBodySet,
    collider_set: ColliderSet,
    gravity: Vector<real>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    physics_hooks: (),
    event_handler: (),
}

#[derive(Default)]
struct SpaceParams {
    contact_recycle_radius: f32,
    contact_max_separation: f32,
    max_allowed_penetration: f32,
    contact_default_bias: f32,
    linear_velocity_sleep_threshold: f32,
    angular_velocity_sleep_threshold: f32,
    body_time_to_sleep: f32,
    solver_iterations: f32,
}

impl SpaceParams {
    fn set(&mut self, param: SpaceParameter, value: f32) {
        match param {
            SpaceParameter::SPACE_PARAM_CONTACT_RECYCLE_RADIUS => {
                self.contact_recycle_radius = value;
            }
            SpaceParameter::SPACE_PARAM_CONTACT_MAX_SEPARATION => {
                self.contact_max_separation = value;
            }
            SpaceParameter::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION => {
                self.max_allowed_penetration = value;
            }
            SpaceParameter::SPACE_PARAM_CONTACT_DEFAULT_BIAS => {
                self.contact_default_bias = value;
            }
            SpaceParameter::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD => {
                self.linear_velocity_sleep_threshold = value;
            }
            SpaceParameter::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD => {
                self.angular_velocity_sleep_threshold = value;
            }
            SpaceParameter::SPACE_PARAM_BODY_TIME_TO_SLEEP => {
                self.body_time_to_sleep = value;
            }
            SpaceParameter::SPACE_PARAM_SOLVER_ITERATIONS => {
                self.solver_iterations = value;
            }
            _ => {}
        }
    }

    const fn get(&self, param: SpaceParameter) -> f32 {
        match param {
            SpaceParameter::SPACE_PARAM_CONTACT_RECYCLE_RADIUS => self.contact_recycle_radius,
            SpaceParameter::SPACE_PARAM_CONTACT_MAX_SEPARATION => self.contact_max_separation,
            SpaceParameter::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION => {
                self.max_allowed_penetration
            }
            SpaceParameter::SPACE_PARAM_CONTACT_DEFAULT_BIAS => self.contact_default_bias,
            SpaceParameter::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD => {
                self.linear_velocity_sleep_threshold
            }
            SpaceParameter::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD => {
                self.angular_velocity_sleep_threshold
            }
            SpaceParameter::SPACE_PARAM_BODY_TIME_TO_SLEEP => self.body_time_to_sleep,
            SpaceParameter::SPACE_PARAM_SOLVER_ITERATIONS => self.solver_iterations,
            _ => 0.0,
        }
    }
}

impl RapierSpace {
    pub fn new(rid: Rid) -> Self {
        let gravity_vector: Vector3 = ProjectSettings::singleton()
            .get("physics/3d/default_gravity_vector".into())
            .to();
        let gravity_magnitude: f32 = ProjectSettings::singleton()
            .get("physics/3d/default_gravity".into())
            .to();
        let gravity = (gravity_vector * gravity_magnitude).into_ext();
        Self {
            rid,
            params: SpaceParams::default(),
            default_area: rid,
            rigidbody_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            gravity,
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::default(),
            physics_hooks: (),
            event_handler: (),
        }
    }

    pub const fn rid(&self) -> Rid {
        self.rid
    }
    pub fn step(&mut self) {
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigidbody_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &self.physics_hooks,
            &self.event_handler,
        );
        self.query_pipeline
            .update(&self.rigidbody_set, &self.collider_set);
    }
    #[must_use]
    pub fn add_body(&mut self, body: &Gd<RapierBody>) -> RigidBodyHandle {
        let body = body.bind();
        let (isometry, _) = body.state.transform.into_ext();
        let center_of_mass = body.params.center_of_mass_local.into_ext();
        let principal_inertia = body.params.inertia.into_ext();
        let additional_mp =
            MassProperties::new(center_of_mass, body.params.mass, principal_inertia);
        let rigidbody = RigidBodyBuilder::new(body.body_mode.into_ext())
            .position(isometry)
            .linvel(body.state.linear_velocity.into_ext())
            .angvel(body.state.angular_velocity.into_ext())
            .gravity_scale(body.params.gravity_scale)
            .can_sleep(body.state.can_sleep)
            .ccd_enabled(body.ccd_enabled)
            .user_data(u128::from(body.user_flags))
            .additional_mass_properties(additional_mp)
            .enabled_translations(
                body.axis_locked.linear_x,
                body.axis_locked.linear_y,
                body.axis_locked.linear_z,
            )
            .enabled_rotations(
                body.axis_locked.angular_x,
                body.axis_locked.angular_y,
                body.axis_locked.angular_z,
            )
            .linear_damping(body.params.linear_damp)
            .angular_damping(body.params.angular_damp)
            .sleeping(body.state.is_sleeping);

        let handle = self.rigidbody_set.insert(rigidbody);

        let collider = ColliderBuilder::ball(0.5);

        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigidbody_set);
        handle
    }
    pub fn add_area(&self, area: &RapierArea) {}
}

impl RapierSpace {
    pub fn set_param(&mut self, param: SpaceParameter, value: f32) {
        self.params.set(param, value);
    }
    pub const fn get_param(&self, param: SpaceParameter) -> f32 {
        self.params.get(param)
    }
}

#[godot_api]
impl PhysicsDirectSpaceState3DExtensionVirtual for RapierSpace {
    unsafe fn intersect_ray(
        &mut self,
        from: Vector3,
        to: Vector3,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        hit_from_inside: bool,
        hit_back_faces: bool,
        pick_ray: bool,
        result: *mut PhysicsServer3DExtensionRayResult,
    ) -> bool {
        Default::default()
    }

    unsafe fn intersect_point(
        &mut self,
        position: Vector3,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut PhysicsServer3DExtensionShapeResult,
        max_results: i32,
    ) -> i32 {
        Default::default()
    }

    unsafe fn intersect_shape(
        &mut self,
        shape_rid: Rid,
        transform: Transform3D,
        motion: Vector3,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        result_count: *mut PhysicsServer3DExtensionShapeResult,
        max_results: i32,
    ) -> i32 {
        Default::default()
    }

    unsafe fn cast_motion(
        &mut self,
        shape_rid: Rid,
        transform: Transform3D,
        motion: Vector3,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        closest_safe: *mut f64,
        closest_unsafe: *mut f64,
        info: *mut PhysicsServer3DExtensionShapeRestInfo,
    ) -> bool {
        Default::default()
    }

    unsafe fn collide_shape(
        &mut self,
        shape_rid: Rid,
        transform: Transform3D,
        motion: Vector3,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut std::ffi::c_void,
        max_results: i32,
        result_count: *mut i32,
    ) -> bool {
        Default::default()
    }

    unsafe fn rest_info(
        &mut self,
        shape_rid: Rid,
        transform: Transform3D,
        motion: Vector3,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        rest_info: *mut PhysicsServer3DExtensionShapeRestInfo,
    ) -> bool {
        Default::default()
    }

    fn get_closest_point_to_object_volume(&self, object: Rid, point: Vector3) -> Vector3 {
        Vector3::default()
    }
}
