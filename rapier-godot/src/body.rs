use std::{cell::RefCell, rc::Rc};

use godot::prelude::{Rid, Transform3D};
use rapier3d::prelude::RigidBody;

use crate::shape::{RapierShape, RapierShapeInstance};
#[derive(Default)]
pub struct RapierBody {
    pub space_id: Option<Rid>,
    pub body: RigidBody,
    pub shapes: Vec<RapierShapeInstance>,
}

impl RapierBody {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_shape(
        &mut self,
        shape: Rc<RefCell<dyn RapierShape>>,
        transform: Transform3D,
        disabled: bool,
    ) {
        let isometry = decompose(transform);
        let shape_instance = RapierShapeInstance::new(shape, isometry, disabled);
        self.shapes.push(shape_instance);
    }
}

fn decompose(
    transform: Transform3D,
) -> rapier3d::na::Isometry<f32, rapier3d::na::Unit<rapier3d::na::Quaternion<f32>>, 3> {
    todo!()
}
