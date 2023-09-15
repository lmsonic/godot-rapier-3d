use godot::{
    engine::{
        native::{
            PhysicsServer3DExtensionRayResult, PhysicsServer3DExtensionShapeRestInfo,
            PhysicsServer3DExtensionShapeResult,
        },
        physics_server_3d::SpaceParameter,
        PhysicsDirectSpaceState3DExtensionVirtual,
    },
    prelude::*,
};

#[derive(GodotClass)]
#[class(base=PhysicsDirectSpaceState3DExtension)]
pub struct RapierSpace {
    rid: Rid,
    params: SpaceParams,
    default_area: Rid,
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
        Self {
            rid,
            params: SpaceParams::default(),
            default_area: rid,
        }
    }

    pub const fn rid(&self) -> Rid {
        self.rid
    }
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
