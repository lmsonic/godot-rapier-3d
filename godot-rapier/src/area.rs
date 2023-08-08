use std::{cell::RefCell, rc::Rc};

use godot::{
    engine::physics_server_3d::{AreaParameter, AreaSpaceOverrideMode},
    prelude::*,
};
use rapier3d::prelude::*;

use crate::{
    collision_object::{Handle, RapierCollisionObject},
    conversions::transform_to_isometry,
    error::RapierError,
    shapes::RapierShapeInstance,
    space::RapierSpace,
};

const DEFAULT_WIND_FORCE_MAGNITUDE: f32 = 0.0;
const DEFAULT_WIND_ATTENUATION_FACTOR: f32 = 0.0;

const DEFAULT_WIND_SOURCE: Vector3 = Vector3::ZERO;
const DEFAULT_WIND_DIRECTION: Vector3 = Vector3::ZERO;

pub struct RapierArea {
    rid: Rid,
    space: Option<Rc<RefCell<RapierSpace>>>,
    handle: Option<ColliderHandle>,
    shapes: Vec<RapierShapeInstance>,
    instance_id: Option<u64>,

    priority: f32,
    gravity: f32,
    gravity_vector: Vector3,
    is_point_gravity: bool,
    point_gravity_distance: f32,
    gravity_mode: AreaSpaceOverrideMode,
    linear_damp: f32,
    linear_damp_mode: AreaSpaceOverrideMode,
    angular_damp: f32,
    angular_damp_mode: AreaSpaceOverrideMode,

    body_monitor_callback: Callable,
    area_monitor_callback: Callable,

    monitorable: bool,
    collision_layer: u32,
    collision_mask: u32,

    pub transform: Transform3D,
}

impl Default for RapierArea {
    fn default() -> Self {
        Self {
            rid: Rid::Invalid,

            space: Option::default(),
            handle: Option::default(),
            shapes: Vec::default(),
            instance_id: Option::default(),

            priority: Default::default(),
            gravity: Default::default(),
            gravity_vector: Vector3::default(),
            is_point_gravity: Default::default(),
            point_gravity_distance: Default::default(),
            gravity_mode: AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_DISABLED,
            linear_damp: Default::default(),
            linear_damp_mode: AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_DISABLED,
            angular_damp: Default::default(),
            angular_damp_mode: AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_DISABLED,

            body_monitor_callback: Callable::invalid(),
            area_monitor_callback: Callable::invalid(),

            monitorable: Default::default(),
            collision_layer: 1,
            collision_mask: 1,
            transform: Transform3D::IDENTITY,
        }
    }
}

impl RapierCollisionObject for RapierArea {
    fn rid(&self) -> Rid {
        self.rid
    }

    fn set_space(&mut self, space: Rc<RefCell<RapierSpace>>) {
        self.space = Some(space);
    }

    fn space(&self) -> Option<&Rc<RefCell<RapierSpace>>> {
        if self.space.is_none() {
            let caller_location = std::panic::Location::caller();
            let file = caller_location.file();
            let line_number = caller_location.line();
            godot_error!(
                "{} called from {file}:{line_number}",
                RapierError::ObjectSpaceNotSet(self.rid)
            );
        }
        self.space.as_ref()
    }
    fn remove_space(&mut self, remove_from_space: bool) {
        if remove_from_space {
            if let Some(space) = self.space() {
                if let Some(handle) = self.handle() {
                    space.borrow_mut().remove_area(handle);
                }
            }
        }
        self.space = None;
        self.handle = None;
    }

    fn generic_handle(&self) -> Handle {
        self.handle().map_or(Handle::NotSet, Handle::AreaHandle)
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
            godot_error!("{}", RapierError::AreaInstanceIDNotSet(self.rid));
        }
        self.instance_id
    }

    fn isometry(&self) -> Isometry<f32> {
        transform_to_isometry(&self.transform).0
    }

    fn scale(&self) -> Vector<f32> {
        transform_to_isometry(&self.transform).1
    }

    fn set_collision_layer(&mut self, layer: u32) {
        self.collision_layer = layer;
        if let Some(space) = self.space() {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_area_collision_group(
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
        if let Some(space) = self.space() {
            if let Some(handle) = self.handle() {
                space.borrow_mut().set_area_collision_group(
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
}

#[allow(clippy::default_trait_access)]
impl RapierArea {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            ..Default::default()
        }
    }

    pub fn set_handle(&mut self, handle: ColliderHandle) {
        self.handle = Some(handle);
    }

    pub fn set_transform(&mut self, transform: Transform3D) {
        self.transform = transform;
        if let Some(space) = self.space() {
            if let Some(handle) = self.handle() {
                let (isometry, _) = transform_to_isometry(&transform);
                space.borrow_mut().set_area_isometry(handle, isometry);
            }
        }
    }

    pub const fn get_transform(&self) -> Transform3D {
        self.transform
    }

    pub fn get_param(&self, param: AreaParameter) -> Variant {
        match param {
            AreaParameter::AREA_PARAM_GRAVITY_OVERRIDE_MODE => Variant::from(self.gravity_mode),
            AreaParameter::AREA_PARAM_GRAVITY => Variant::from(self.gravity),
            AreaParameter::AREA_PARAM_GRAVITY_VECTOR => Variant::from(self.gravity_vector),
            AreaParameter::AREA_PARAM_GRAVITY_IS_POINT => Variant::from(self.is_point_gravity),
            AreaParameter::AREA_PARAM_GRAVITY_POINT_UNIT_DISTANCE => {
                Variant::from(self.point_gravity_distance)
            }
            AreaParameter::AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE => {
                Variant::from(self.linear_damp_mode)
            }
            AreaParameter::AREA_PARAM_LINEAR_DAMP => Variant::from(self.linear_damp),
            AreaParameter::AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE => {
                Variant::from(self.angular_damp_mode)
            }
            AreaParameter::AREA_PARAM_ANGULAR_DAMP => Variant::from(self.angular_damp),
            AreaParameter::AREA_PARAM_PRIORITY => Variant::from(self.priority),
            AreaParameter::AREA_PARAM_WIND_FORCE_MAGNITUDE => {
                Variant::from(DEFAULT_WIND_FORCE_MAGNITUDE)
            }
            AreaParameter::AREA_PARAM_WIND_SOURCE => Variant::from(DEFAULT_WIND_SOURCE),
            AreaParameter::AREA_PARAM_WIND_DIRECTION => Variant::from(DEFAULT_WIND_DIRECTION),
            AreaParameter::AREA_PARAM_WIND_ATTENUATION_FACTOR => {
                Variant::from(DEFAULT_WIND_ATTENUATION_FACTOR)
            }
            _ => Variant::nil(),
        }
    }

    pub fn set_param(&mut self, param: AreaParameter, value: &Variant) {
        match param {
            AreaParameter::AREA_PARAM_GRAVITY_OVERRIDE_MODE => {
                self.gravity_mode = value.to();
            }
            AreaParameter::AREA_PARAM_GRAVITY => {
                self.gravity = value.to();
            }
            AreaParameter::AREA_PARAM_GRAVITY_VECTOR => {
                self.gravity_vector = value.to();
            }
            AreaParameter::AREA_PARAM_GRAVITY_IS_POINT => {
                self.is_point_gravity = value.to();
            }
            AreaParameter::AREA_PARAM_GRAVITY_POINT_UNIT_DISTANCE => {
                self.point_gravity_distance = value.to();
            }
            AreaParameter::AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE => {
                self.linear_damp_mode = value.to();
            }
            AreaParameter::AREA_PARAM_LINEAR_DAMP => {
                self.linear_damp = value.to();
            }
            AreaParameter::AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE => {
                self.angular_damp_mode = value.to();
            }
            AreaParameter::AREA_PARAM_ANGULAR_DAMP => {
                self.angular_damp = value.to();
            }
            AreaParameter::AREA_PARAM_PRIORITY => {
                self.priority = value.to();
            }
            AreaParameter::AREA_PARAM_WIND_FORCE_MAGNITUDE => {
                godot_warn!("Area wind force magnitude is not supported by Godot Rapier. Any such value will be ignored.");
            }
            AreaParameter::AREA_PARAM_WIND_SOURCE => {
                godot_warn!("Area wind source is not supported by Godot Rapier. Any such value will be ignored.");
            }
            AreaParameter::AREA_PARAM_WIND_DIRECTION => {
                godot_warn!("Area wind direction is not supported by Godot Rapier. Any such value will be ignored.");
            }
            AreaParameter::AREA_PARAM_WIND_ATTENUATION_FACTOR => {
                godot_warn!("Area wind attenuation factor is not supported by Godot Rapier. Any such value will be ignored.");
            }
            _ => {}
        };
    }

    pub fn set_area_monitor_callback(&mut self, callback: Callable) {
        self.area_monitor_callback = callback;
    }
    pub fn set_body_monitor_callback(&mut self, callback: Callable) {
        self.body_monitor_callback = callback;
    }

    pub fn set_monitorable(&mut self, monitorable: bool) {
        self.monitorable = monitorable;
    }

    pub fn handle(&self) -> Option<ColliderHandle> {
        if self.handle.is_none() {
            godot_error!("{}", RapierError::AreaHandleNotSet(self.rid));
        }
        self.handle
    }

    pub fn compute_gravity(&self, position: Vector3) -> Vector3 {
        if !self.is_point_gravity {
            return self.gravity_vector * self.gravity;
        }
        let point = self.transform * self.gravity_vector;
        let to_point = point - position;
        let to_point_dst_squared = f32::max(to_point.length_squared(), math::FloatExt::CMP_EPSILON);
        let to_point_dir = to_point / f32::sqrt(to_point_dst_squared);
        let gravity_distance_sqr = self.point_gravity_distance * self.point_gravity_distance;

        to_point_dir * (self.gravity * gravity_distance_sqr / to_point_dst_squared)
    }

    pub const fn gravity_mode(&self) -> AreaSpaceOverrideMode {
        self.gravity_mode
    }

    pub fn call_queries(&mut self) {
        // TODO
    }

    pub const fn linear_damp_mode(&self) -> AreaSpaceOverrideMode {
        self.linear_damp_mode
    }

    pub const fn angular_damp_mode(&self) -> AreaSpaceOverrideMode {
        self.angular_damp_mode
    }

    pub const fn linear_damp(&self) -> f32 {
        self.linear_damp
    }

    pub const fn angular_damp(&self) -> f32 {
        self.angular_damp
    }
}
