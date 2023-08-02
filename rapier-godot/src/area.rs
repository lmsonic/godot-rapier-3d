use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::prelude::*;

use crate::{
    collision_object::RapierCollisionObject,
    conversions::{isometry_to_transform, transform_to_isometry},
    shape::RapierShapeInstance,
    space::RapierSpace,
    RapierError, RapierResult,
};

#[allow(clippy::module_name_repetitions)]
pub struct RapierArea {
    rid: Rid,
    space: Option<Rc<RefCell<RapierSpace>>>,
    handle: Option<ColliderHandle>,
    shapes: Vec<RapierShapeInstance>,
    instance_id: Option<u64>,
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
            space: Default::default(),
            handle: Default::default(),
            shapes: Default::default(),
            instance_id: Default::default(),
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
}
