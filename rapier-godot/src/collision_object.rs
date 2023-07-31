use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::prelude::*;

use crate::shape::{RapierShape, RapierShapeInstance};

#[allow(clippy::module_name_repetitions)]
pub trait RapierCollisionObject {
    fn set_space_id(&mut self, space_id: Rid);
    fn get_space_id(&self) -> Option<Rid>;

    fn add_shape(
        &mut self,
        shape: Rc<RefCell<dyn RapierShape>>,
        transform: Transform3D,
        disabled: bool,
    );

    fn remove_shape_rid(&mut self, shape_rid: Rid);
    fn remove_nth_shape(&mut self, idx: usize);

    fn clear_shapes(&mut self);

    fn get_shapes(&self) -> &Vec<RapierShapeInstance>;

    fn set_shape(&mut self, idx: usize, shape: Rc<RefCell<dyn RapierShape>>);
    fn set_shape_transform(&mut self, idx: usize, transform: Transform3D);
    fn set_shape_disabled(&mut self, idx: usize, disabled: bool);

    fn set_instance_id(&mut self, id: u64);

    fn get_instance_id(&self) -> Option<u64>;
}
