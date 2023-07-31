use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::prelude::*;

use crate::shape::{RapierShape, RapierShapeInstance};

pub trait RapierCollisionObject {
    fn set_space_id(&mut self, space_id: Rid);
    fn get_space_id(&self) -> Option<Rid>;

    fn add_shape(
        &mut self,
        shape: Rc<RefCell<dyn RapierShape>>,
        transform: Transform3D,
        disabled: bool,
    );

    fn remove_shape(&mut self, shape_rid: Rid);

    fn get_shapes(&self) -> &Vec<RapierShapeInstance>;

    fn set_instance_id(&mut self, id: u64);

    fn get_instance_id(&self) -> Option<u64>;
}
