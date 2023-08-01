use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::{na::Dim, prelude::*};

use crate::{
    shape::{RapierShape, RapierShapeInstance},
    space::RapierSpace,
};

#[allow(clippy::module_name_repetitions)]
pub trait RapierCollisionObject {
    fn set_space(&mut self, space: Rc<RefCell<RapierSpace>>);
    fn get_space(&self) -> Option<Rc<RefCell<RapierSpace>>>;

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

    fn build_shared_shape(&self) -> Option<SharedShape> {
        if self.get_shapes().is_empty() {
            None
        } else if self.get_shapes().len() == 1 {
            let shape_instance = &self.get_shapes()[0];
            Some(shape_instance.shape.borrow().get_shape())
        } else {
            let compound_shapes: Vec<(Isometry<f32>, SharedShape)> = self
                .get_shapes()
                .iter()
                .filter(|shape_instance| !shape_instance.disabled)
                .map(|shape_instance| {
                    (
                        shape_instance.isometry,
                        shape_instance.shape.borrow().get_shape(),
                    )
                })
                .collect();
            Some(SharedShape::new(Compound::new(compound_shapes)))
        }
    }

    fn build_collider(&self, is_sensor: bool) -> Collider {
        self.build_shared_shape().map_or_else(
            || {
                // Empty shape collider
                ColliderBuilder::ball(0.5)
                    .enabled(false)
                    .sensor(true)
                    .build()
            },
            |shared_shape| {
                if shared_shape.as_compound().is_some() {
                    ColliderBuilder::new(shared_shape).sensor(is_sensor).build()
                } else {
                    let shape_instance = &self.get_shapes()[0];
                    ColliderBuilder::new(shared_shape)
                        .position(shape_instance.isometry)
                        .sensor(is_sensor)
                        .enabled(!shape_instance.disabled)
                        .build()
                }
            },
        )
    }
}
