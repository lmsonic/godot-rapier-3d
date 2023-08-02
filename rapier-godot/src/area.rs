use std::{cell::RefCell, rc::Rc};

use godot::{
    engine::physics_server_3d::{AreaParameter, AreaSpaceOverrideMode},
    prelude::*,
};
use rapier3d::prelude::*;

use crate::{
    collision_object::RapierCollisionObject,
    conversions::{isometry_to_transform, transform_to_isometry},
    shape::RapierShapeInstance,
    space::RapierSpace,
    RapierError, RapierResult,
};

const DEFAULT_WIND_FORCE_MAGNITUDE: f32 = 0.0;
const DEFAULT_WIND_ATTENUATION_FACTOR: f32 = 0.0;

const DEFAULT_WIND_SOURCE: Vector3 = Vector3::ZERO;
const DEFAULT_WIND_DIRECTION: Vector3 = Vector3::ZERO;

#[allow(clippy::module_name_repetitions)]
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
}

impl Default for RapierArea {
    fn default() -> Self {
        Self {
            rid: Rid::Invalid,
            space: Default::default(),
            handle: Default::default(),
            shapes: Default::default(),
            instance_id: Default::default(),
            priority: Default::default(),
            gravity: Default::default(),
            gravity_vector: Default::default(),
            is_point_gravity: Default::default(),
            point_gravity_distance: Default::default(),
            gravity_mode: AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_DISABLED,
            linear_damp: Default::default(),
            linear_damp_mode: AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_DISABLED,
            angular_damp: Default::default(),
            angular_damp_mode: AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_DISABLED,
        }
    }
}

impl RapierCollisionObject for RapierArea {
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
            let handle = self.handle.ok_or(RapierError::AreaHandleNotSet(self.rid))?;
            let collider = space
                .get_area_mut(handle)
                .ok_or(RapierError::AreaHandleInvalid(self.rid))?;

            if let Some(shapes) = self.build_shared_shape() {
                collider.set_shape(shapes);
            } else {
                collider.set_enabled(false);
            }
            Ok(())
        };
        if let Err(e) = update_shapes() {
            godot_error!("{}", e);
        }
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
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                if let Some(collider) = space.borrow_mut().get_area_mut(handle) {
                    collider.set_position(transform_to_isometry(&transform));
                }
                godot_error!("{}", RapierError::AreaHandleInvalid(self.rid));
            } else {
                godot_error!("{}", RapierError::AreaHandleNotSet(self.rid));
            }
        } else {
            godot_error!("{}", RapierError::ObjectSpaceNotSet(self.rid));
        }
    }

    pub fn get_transform(&self) -> Option<Transform3D> {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                if let Some(collider) = space.borrow().get_area(handle) {
                    return Some(isometry_to_transform(collider.position()));
                }
                godot_error!("{}", RapierError::AreaHandleInvalid(self.rid));
            } else {
                godot_error!("{}", RapierError::AreaHandleNotSet(self.rid));
            }
        } else {
            godot_error!("{}", RapierError::ObjectSpaceNotSet(self.rid));
        }
        None
    }

    pub fn get_param(&mut self, param: AreaParameter) -> Variant {
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

    pub fn set_param(&mut self, param: AreaParameter, value: Variant) {
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
}
