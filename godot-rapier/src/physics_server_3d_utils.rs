use std::{cell::RefCell, rc::Rc};

use godot::prelude::{
    utilities::{rid_allocate_id, rid_from_int64},
    *,
};

use crate::{
    area::RapierArea,
    body::RapierBody,
    error::{RapierError, RapierResult},
    joint::RapierJoint,
    physics_server_3d::RapierPhysicsServer3D,
    shapes::RapierShape,
    space::RapierSpace,
};

#[inline]
pub fn make_rid() -> Rid {
    rid_from_int64(rid_allocate_id())
}

impl RapierPhysicsServer3D {
    pub(crate) fn get_shape(&self, rid: Rid) -> RapierResult<&Rc<RefCell<dyn RapierShape>>> {
        if let Some(shape) = self.shapes.get(&rid) {
            return Ok(shape);
        }
        godot_error!("{}", RapierError::ShapeRidMissing(rid));
        Err(RapierError::ShapeRidMissing(rid))
    }
    #[track_caller]
    pub(crate) fn get_area(&self, rid: Rid) -> RapierResult<&Rc<RefCell<RapierArea>>> {
        if let Some(area) = self.areas.get(&rid) {
            return Ok(area);
        }
        let caller_location = std::panic::Location::caller();
        let file = caller_location.file();
        let line_number = caller_location.line();
        godot_error!(
            "{} called from {file}:{line_number}",
            RapierError::SpaceRidMissing(rid)
        );
        Err(RapierError::AreaRidMissing(rid))
    }
    pub(crate) fn get_body(&self, rid: Rid) -> RapierResult<&Rc<RefCell<RapierBody>>> {
        if let Some(body) = self.bodies.get(&rid) {
            return Ok(body);
        }
        godot_error!("{}", RapierError::BodyRidMissing(rid));
        Err(RapierError::BodyRidMissing(rid))
    }
    #[track_caller]
    pub(crate) fn get_space(&self, rid: Rid) -> RapierResult<&Rc<RefCell<RapierSpace>>> {
        if let Some(space) = self.spaces.get(&rid) {
            return Ok(space);
        }
        let caller_location = std::panic::Location::caller();
        let file = caller_location.file();
        let line_number = caller_location.line();
        godot_error!(
            "{} called from {file}:{line_number}",
            RapierError::SpaceRidMissing(rid)
        );
        Err(RapierError::SpaceRidMissing(rid))
    }
    pub(crate) fn has_space(&self, rid: Rid) -> bool {
        if self.spaces.contains_key(&rid) {
            return true;
        }
        godot_error!("{}", RapierError::SpaceRidMissing(rid));
        false
    }

    pub(crate) fn get_joint(&self, rid: Rid) -> RapierResult<&Rc<RefCell<RapierJoint>>> {
        if let Some(joint) = self.joints.get(&rid) {
            return Ok(joint);
        }
        godot_error!("{}", RapierError::JointRidMissing(rid));
        Err(RapierError::JointRidMissing(rid))
    }
}
