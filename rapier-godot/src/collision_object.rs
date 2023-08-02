use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::prelude::*;

use crate::{
    conversions::transform_to_isometry,
    shape::{RapierShape, RapierShapeInstance},
    space::RapierSpace,
    RapierError,
};

#[allow(clippy::module_name_repetitions)]
pub trait RapierCollisionObject {
    fn rid(&self) -> Rid;
    fn set_space(&mut self, space: Rc<RefCell<RapierSpace>>);
    fn space(&self) -> Option<Rc<RefCell<RapierSpace>>>;
    fn remove_space(&mut self);

    fn add_shape(
        &mut self,
        shape: Rc<RefCell<dyn RapierShape>>,
        transform: Transform3D,
        disabled: bool,
    ) {
        let isometry = transform_to_isometry(&transform);
        let shape_instance = RapierShapeInstance::new(shape, isometry, disabled);
        self.shapes_mut().push(shape_instance);

        self.update_shapes();
    }
    fn remove_shape_rid(&mut self, shape_rid: Rid) {
        self.shapes_mut()
            .retain(|s| s.shape.borrow().rid() != shape_rid);

        self.update_shapes();
    }
    fn remove_nth_shape(&mut self, idx: usize) {
        self.shapes_mut().swap_remove(idx);

        self.update_shapes();
    }

    fn clear_shapes(&mut self) {
        self.shapes_mut().clear();
        self.update_shapes();
    }

    fn set_shape(&mut self, idx: usize, s: Rc<RefCell<dyn RapierShape>>) {
        if let Some(shape) = self.shapes_mut().get_mut(idx) {
            shape.shape = s.clone();
            self.update_shapes();
        } else {
            godot_error!("{}", RapierError::ShapeNotInObject(idx, self.rid()));
        }
    }

    fn set_shape_transform(&mut self, idx: usize, transform: Transform3D) {
        if let Some(shape) = self.shapes_mut().get_mut(idx) {
            shape.isometry = transform_to_isometry(&transform);
            self.update_shapes();
        } else {
            godot_error!("{}", RapierError::ShapeNotInObject(idx, self.rid()));
        }
    }

    fn set_shape_disabled(&mut self, idx: usize, disabled: bool) {
        if let Some(shape) = self.shapes_mut().get_mut(idx) {
            shape.disabled = disabled;
            self.update_shapes();
        } else {
            godot_error!("{}", RapierError::ShapeNotInObject(idx, self.rid()));
        }
    }

    fn shapes(&self) -> &Vec<RapierShapeInstance>;
    fn shapes_mut(&mut self) -> &mut Vec<RapierShapeInstance>;
    fn update_shapes(&mut self);
    fn set_instance_id(&mut self, id: u64);
    fn instance_id(&self) -> Option<u64>;

    fn remove_from_space(&self);

    fn build_shared_shape(&self) -> Option<SharedShape> {
        if self.shapes().is_empty() {
            godot_error!("{}", RapierError::BuildingObjectWithNoShapes(self.rid()));
            None
        } else if self.shapes().len() == 1 {
            let shape_instance = &self.shapes()[0];
            Some(shape_instance.shape.borrow().get_shape())
        } else {
            let compound_shapes: Vec<(Isometry<f32>, SharedShape)> = self
                .shapes()
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
        self.build_shared_shape().map_or(
            {
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
                    let shape_instance = &self.shapes()[0];
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
