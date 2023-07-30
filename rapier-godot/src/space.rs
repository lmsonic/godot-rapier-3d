use std::{cell::RefCell, rc::Rc};

use godot::{engine::physics_server_3d::SpaceParameter, prelude::*};

use crate::area::RapierArea;

#[allow(clippy::module_name_repetitions)]
pub struct RapierSpace {
    rid: Rid,
    default_area: Option<Rc<RefCell<RapierArea>>>,
}
const DEFAULT_CONTACT_RECYCLE_RADIUS: f32 = 0.01;
const DEFAULT_CONTACT_MAX_SEPARATION: f32 = 0.05;
const DEFAULT_CONTACT_MAX_ALLOWED_PENETRATION: f32 = 0.01;
const DEFAULT_CONTACT_DEFAULT_BIAS: f32 = 0.8;
const DEFAULT_SLEEP_THRESHOLD_LINEAR: f32 = 0.1;
const DEFAULT_SLEEP_THRESHOLD_ANGULAR: f32 = 8.0 * std::f32::consts::PI / 180.0;
const DEFAULT_SOLVER_ITERATIONS: f32 = 8.0;
const DEFAULT_BODY_TIME_TO_SLEEP: f32 = 0.5;

impl RapierSpace {
    pub fn set_default_area(&mut self, area: Rc<RefCell<RapierArea>>) {
        self.default_area = Some(area);
    }
    #[allow(clippy::match_same_arms, clippy::unused_self)]
    pub fn set_param(&mut self, param: SpaceParameter, _value: f32) {
        match param {
            SpaceParameter::SPACE_PARAM_CONTACT_RECYCLE_RADIUS => {}
            SpaceParameter::SPACE_PARAM_CONTACT_MAX_SEPARATION => {}
            SpaceParameter::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION => {}
            SpaceParameter::SPACE_PARAM_CONTACT_DEFAULT_BIAS => {}
            SpaceParameter::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD => {}
            SpaceParameter::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD => {}
            SpaceParameter::SPACE_PARAM_BODY_TIME_TO_SLEEP => {}
            SpaceParameter::SPACE_PARAM_SOLVER_ITERATIONS => {}
            _ => {}
        }
    }
    #[allow(clippy::unused_self)]
    pub const fn get_param(&self, param: SpaceParameter) -> f32 {
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
            SpaceParameter::SPACE_PARAM_BODY_TIME_TO_SLEEP => DEFAULT_BODY_TIME_TO_SLEEP,
            SpaceParameter::SPACE_PARAM_SOLVER_ITERATIONS => DEFAULT_SOLVER_ITERATIONS,
            _ => 0.0,
        }
    }
    pub const fn get_rid(&self) -> Rid {
        self.rid
    }
}

impl RapierSpace {
    pub fn new(rid: Rid) -> Self {
        Self {
            rid,
            default_area: None,
        }
    }
}
