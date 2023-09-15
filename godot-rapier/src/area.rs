#![allow(clippy::cast_sign_loss)]
use godot::{
    engine::{physics_server_3d::AreaParameter, physics_server_3d::AreaSpaceOverrideMode},
    prelude::*,
};

use crate::{collision_object::RapierCollisionObject, shapes::ShapeInstance};

pub struct RapierArea {
    rid: Rid,
    space: Rid,
    shapes: Vec<ShapeInstance>,
    instance_id: Option<u64>,
    params: AreaParams,
    transform: Transform3D,
    collision_layer: u32,
    collision_mask: u32,
    monitorable: bool,
    ray_pickable: bool,
    body_monitor_callback: Callable,
    area_monitor_callback: Callable,
}

struct AreaParams {
    gravity_override_mode: AreaSpaceOverrideMode,
    gravity: f32,
    gravity_vector: Vector3,
    gravity_is_point: bool,
    gravity_point_unit_distance: f32,
    linear_damp_override_mode: AreaSpaceOverrideMode,
    linear_damp: f32,
    angular_damp_override_mod: AreaSpaceOverrideMode,
    angular_damp: f32,
    priority: i32,
    wind_force_magnitude: f32,
    wind_source: NodePath,
    wind_direction: f32,
    wind_attenuation_factor: f32,
}

impl Default for AreaParams {
    fn default() -> Self {
        Self {
            gravity_override_mode: AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_DISABLED,
            gravity: Default::default(),
            gravity_vector: Vector3::default(),
            gravity_is_point: Default::default(),
            gravity_point_unit_distance: Default::default(),
            linear_damp_override_mode: AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_DISABLED,
            linear_damp: Default::default(),
            angular_damp_override_mod: AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_DISABLED,
            angular_damp: Default::default(),
            priority: Default::default(),
            wind_force_magnitude: Default::default(),
            wind_source: NodePath::default(),
            wind_direction: Default::default(),
            wind_attenuation_factor: Default::default(),
        }
    }
}

impl AreaParams {
    fn set(&mut self, param: AreaParameter, value: &Variant) {
        match param {
            AreaParameter::AREA_PARAM_GRAVITY_OVERRIDE_MODE => {
                self.gravity_override_mode = value.to();
            }
            AreaParameter::AREA_PARAM_GRAVITY => {
                self.gravity = value.to();
            }
            AreaParameter::AREA_PARAM_GRAVITY_VECTOR => {
                self.gravity_vector = value.to();
            }
            AreaParameter::AREA_PARAM_GRAVITY_IS_POINT => {
                self.gravity_is_point = value.to();
            }
            AreaParameter::AREA_PARAM_GRAVITY_POINT_UNIT_DISTANCE => {
                self.gravity_point_unit_distance = value.to();
            }
            AreaParameter::AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE => {
                self.linear_damp_override_mode = value.to();
            }
            AreaParameter::AREA_PARAM_LINEAR_DAMP => {
                self.linear_damp = value.to();
            }
            AreaParameter::AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE => {
                self.angular_damp_override_mod = value.to();
            }
            AreaParameter::AREA_PARAM_ANGULAR_DAMP => {
                self.angular_damp = value.to();
            }
            AreaParameter::AREA_PARAM_PRIORITY => {
                self.priority = value.to();
            }
            AreaParameter::AREA_PARAM_WIND_FORCE_MAGNITUDE => {
                self.wind_force_magnitude = value.to();
            }
            AreaParameter::AREA_PARAM_WIND_SOURCE => {
                self.wind_source = value.to();
            }
            AreaParameter::AREA_PARAM_WIND_DIRECTION => {
                self.wind_direction = value.to();
            }
            AreaParameter::AREA_PARAM_WIND_ATTENUATION_FACTOR => {
                self.wind_attenuation_factor = value.to();
            }
            _ => {}
        }
    }
    fn get(&self, param: AreaParameter) -> Variant {
        match param {
            AreaParameter::AREA_PARAM_GRAVITY_OVERRIDE_MODE => {
                self.gravity_override_mode.to_variant()
            }
            AreaParameter::AREA_PARAM_GRAVITY => self.gravity.to_variant(),
            AreaParameter::AREA_PARAM_GRAVITY_VECTOR => self.gravity_vector.to_variant(),
            AreaParameter::AREA_PARAM_GRAVITY_IS_POINT => self.gravity_is_point.to_variant(),
            AreaParameter::AREA_PARAM_GRAVITY_POINT_UNIT_DISTANCE => {
                self.gravity_point_unit_distance.to_variant()
            }
            AreaParameter::AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE => {
                self.linear_damp_override_mode.to_variant()
            }
            AreaParameter::AREA_PARAM_LINEAR_DAMP => self.linear_damp.to_variant(),
            AreaParameter::AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE => {
                self.angular_damp_override_mod.to_variant()
            }
            AreaParameter::AREA_PARAM_ANGULAR_DAMP => self.angular_damp.to_variant(),
            AreaParameter::AREA_PARAM_PRIORITY => self.priority.to_variant(),
            AreaParameter::AREA_PARAM_WIND_FORCE_MAGNITUDE => {
                self.wind_force_magnitude.to_variant()
            }
            AreaParameter::AREA_PARAM_WIND_SOURCE => self.wind_source.to_variant(),
            AreaParameter::AREA_PARAM_WIND_DIRECTION => self.wind_direction.to_variant(),
            AreaParameter::AREA_PARAM_WIND_ATTENUATION_FACTOR => {
                self.wind_attenuation_factor.to_variant()
            }
            _ => Variant::default(),
        }
    }
}

impl RapierCollisionObject for RapierArea {
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

impl RapierArea {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            space: Rid::Invalid,
            shapes: vec![],
            instance_id: None,
            params: AreaParams::default(),
            transform: Transform3D::default(),
            collision_layer: Default::default(),
            collision_mask: Default::default(),
            monitorable: Default::default(),
            ray_pickable: Default::default(),
            body_monitor_callback: Callable::invalid(),
            area_monitor_callback: Callable::invalid(),
        }
    }

    pub fn set_space(&mut self, space: Rid) {
        self.space = space;
    }

    pub const fn get_space(&self) -> Rid {
        self.space
    }

    pub const fn rid(&self) -> Rid {
        self.rid
    }

    pub fn set_param(&mut self, param: AreaParameter, value: &Variant) {
        self.params.set(param, value);
    }
    pub fn get_param(&self, param: AreaParameter) -> Variant {
        self.params.get(param)
    }

    pub fn set_transform(&mut self, transform: Transform3D) {
        self.transform = transform;
    }
    pub const fn get_transform(&self) -> Transform3D {
        self.transform
    }

    pub fn set_monitorable(&mut self, monitorable: bool) {
        self.monitorable = monitorable;
    }

    pub fn set_ray_pickable(&mut self, ray_pickable: bool) {
        self.ray_pickable = ray_pickable;
    }

    pub fn set_body_monitor_callback(&mut self, body_monitor_callback: Callable) {
        self.body_monitor_callback = body_monitor_callback;
    }

    pub fn set_area_monitor_callback(&mut self, area_monitor_callback: Callable) {
        self.area_monitor_callback = area_monitor_callback;
    }
}
