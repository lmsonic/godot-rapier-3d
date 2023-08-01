use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::prelude::*;

use crate::{
    collision_object::RapierCollisionObject,
    conversions::{isometry_to_transform, transform_to_isometry},
    shape::{RapierShape, RapierShapeInstance},
    space::RapierSpace,
};

#[derive(Default)]
#[allow(clippy::module_name_repetitions)]
pub struct RapierArea {
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

    fn add_shape(
        &mut self,
        shape: Rc<RefCell<dyn RapierShape>>,
        transform: Transform3D,
        disabled: bool,
    ) {
        let isometry = transform_to_isometry(&transform);
        let shape_instance = RapierShapeInstance::new(shape, isometry, disabled);
        self.shapes.push(shape_instance);

        self.update_shapes();
    }

    fn get_shapes(&self) -> &Vec<RapierShapeInstance> {
        &self.shapes
    }

    fn set_instance_id(&mut self, id: u64) {
        self.instance_id = Some(id);
    }

    fn get_instance_id(&self) -> Option<u64> {
        self.instance_id
    }

    fn remove_shape_rid(&mut self, shape_rid: Rid) {
        self.shapes.retain(|s| s.shape.borrow().rid() != shape_rid);

        self.update_shapes();
    }
    fn remove_nth_shape(&mut self, idx: usize) {
        self.shapes.swap_remove(idx);

        self.update_shapes();
    }

    fn clear_shapes(&mut self) {
        self.shapes.clear();

        self.update_shapes();
    }

    fn set_shape(&mut self, idx: usize, s: Rc<RefCell<dyn RapierShape>>) {
        if let Some(shape) = self.shapes.get_mut(idx) {
            shape.shape = s.clone();
            self.update_shapes();
        }
    }

    fn set_shape_transform(&mut self, idx: usize, transform: Transform3D) {
        if let Some(shape) = self.shapes.get_mut(idx) {
            shape.isometry = transform_to_isometry(&transform);
            self.update_shapes();
        }
    }

    fn set_shape_disabled(&mut self, idx: usize, disabled: bool) {
        if let Some(shape) = self.shapes.get_mut(idx) {
            shape.disabled = disabled;
            self.update_shapes();
        }
    }
}

impl RapierArea {
    pub fn new() -> Self {
        Self::default()
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
            }
        }
    }

    pub fn get_transform(&self) -> Option<Transform3D> {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                if let Some(collider) = space.borrow().get_area(handle) {
                    return Some(isometry_to_transform(collider.position()));
                }
            }
        }
        None
    }

    fn update_shapes(&mut self) {
        if let Some(space) = &self.space {
            if let Some(handle) = self.handle {
                if let Some(collider) = space.borrow_mut().get_area_mut(handle) {
                    if let Some(shapes) = self.build_shared_shape() {
                        collider.set_shape(shapes);
                    } else {
                        collider.set_enabled(false);
                    }
                }
            }
        }
    }
}
