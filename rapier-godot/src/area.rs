use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;

use crate::{shape::RapierShape, space::RapierSpace};

use crate::shape::{RapierCollisionObject, RapierShapeInstance};

impl RapierCollisionObject for RapierArea {}
#[allow(clippy::module_name_repetitions)]
pub struct RapierArea {
    rid: Rid,
    space: Option<Rc<RefCell<RapierSpace>>>,
    shapes: Vec<RapierShapeInstance>,
    priority: i32,
}

fn decompose(mut transform: Transform3D) -> (Transform3D, Vector3) {
    let scale = transform.basis.scale();
    let mut x = transform.basis.col_a();
    let mut y = transform.basis.col_b();
    let mut z = transform.basis.col_c();
    x /= scale.x;
    y -= x * x.dot(y);
    y /= scale.y;
    z -= x * x.dot(z) - y * y.dot(z);
    z /= scale.z;
    transform.basis.set_col_a(x);
    transform.basis.set_col_b(y);
    transform.basis.set_col_c(z);

    (transform, scale)
}

impl RapierArea {
    pub fn get_space(&self) -> Option<Rc<RefCell<RapierSpace>>> {
        self.space.clone()
    }
    pub fn set_space(&mut self, space: Rc<RefCell<RapierSpace>>) {
        self.space = Some(space);
    }
    pub fn add_shape(
        &mut self,
        shape: Rc<RefCell<dyn RapierShape>>,
        transform: Transform3D,
        disabled: bool,
    ) {
        // let (transform, scale) = decompose(transform);
        // self.shapes
        //     .push(RapierShapeInstance::with_transform_scale_disabled(
        //         self,
        //         shape.clone(),
        //         transform,
        //         scale,
        //         disabled,
        //     ))
    }
    pub fn set_priority(&mut self, priority: i32) {
        self.priority = priority;
    }
}

impl RapierArea {
    pub const fn new(rid: Rid) -> Self {
        Self {
            rid,
            space: None,
            shapes: vec![],
            priority: 0,
        }
    }
}
