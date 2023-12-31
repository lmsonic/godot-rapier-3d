use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::prelude::*;

use crate::{
    conversions::IntoExt,
    error::RapierError,
    shapes::{RapierShape, RapierShapeInstance},
};

pub trait RapierCollisionObject {
    fn rid(&self) -> Rid;
    fn remove_space(&mut self, remove_from_space: bool);

    fn add_shape(
        &mut self,
        shape: Rc<RefCell<dyn RapierShape>>,
        transform: Transform3D,
        disabled: bool,
    ) {
        let (isometry, scale) = transform.into_ext();
        let shape_instance = RapierShapeInstance::new(shape, isometry, scale, disabled);
        self.shapes_mut().push(shape_instance);
        self.update_shapes();
    }
    fn remove_shape_rid(&mut self, shape_rid: Rid) {
        let rid = self.rid();
        self.shapes_mut().retain(|s| {
            s.shape.borrow_mut().remove_owner(rid);
            s.shape.borrow().rid() != shape_rid
        });
        self.update_shapes();
    }
    fn remove_nth_shape(&mut self, idx: usize) -> RapierShapeInstance {
        let rid = self.rid();
        let shape_inst = self.shapes_mut().remove(idx);

        self.update_shapes();
        shape_inst
    }

    fn clear_shapes(&mut self) {
        for shape in self.shapes() {
            shape.shape.borrow_mut().remove_owner(self.rid());
        }
        self.shapes_mut().clear();
        self.update_shapes();
    }

    fn set_shape(&mut self, idx: usize, s: Rc<RefCell<dyn RapierShape>>) {
        let rid = self.rid();
        if let Some(shape) = self.shapes_mut().get_mut(idx) {
            shape.shape.borrow_mut().remove_owner(rid);
            shape.shape = s.clone();
            self.update_shapes();
        } else {
            godot_error!("{}", RapierError::ShapeNotInObject(idx, self.rid()));
        }
    }

    fn nth_shape_instance(&self, idx: usize) -> Option<&RapierShapeInstance> {
        if let Some(shape) = self.shapes().get(idx) {
            return Some(shape);
        }
        godot_error!("{}", RapierError::ShapeNotInObject(idx, self.rid()));
        None
    }

    fn set_shape_transform(&mut self, idx: usize, transform: Transform3D) {
        if let Some(shape) = self.shapes_mut().get_mut(idx) {
            let (isometry, scale) = transform.into_ext();
            shape.isometry = isometry;
            shape.scale = scale;
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

    fn build_shared_shape(&self) -> Option<SharedShape> {
        if self.shapes().is_empty() {
            None
        } else if self.shapes().len() == 1 {
            let shape_instance = &self.shapes()[0];
            Some(shape_instance.shared_shape())
        } else {
            let compound_shapes: Vec<(Isometry<f32>, SharedShape)> = self
                .shapes()
                .iter()
                .filter(|shape_instance| !shape_instance.disabled)
                .map(|shape_instance| (shape_instance.isometry, shape_instance.shared_shape()))
                .collect();
            Some(SharedShape::new(Compound::new(compound_shapes)))
        }
    }
    fn transform(&self) -> Transform3D;
    fn isometry(&self) -> Isometry<f32> {
        let (iso, _) = self.transform().into_ext();
        iso
    }
    fn scale(&self) -> Vector<f32> {
        let (_, scale) = self.transform().into_ext();
        scale
    }

    fn build_collider(&self) -> ColliderBuilder {
        let collision_groups = InteractionGroups::new(
            Group::from(self.get_collision_layer()),
            Group::from(self.get_collision_mask()),
        );
        self.build_shared_shape().map_or(
            {
                // Empty shape collider
                ColliderBuilder::ball(0.1)
                    .enabled(false)
                    .sensor(true)
                    .collision_groups(collision_groups)
            },
            |shared_shape| {
                if shared_shape.as_compound().is_some() {
                    ColliderBuilder::new(shared_shape)
                        .collision_groups(collision_groups)
                        .position(self.isometry())
                } else {
                    let shape_instance = &self.shapes()[0];
                    ColliderBuilder::new(shared_shape)
                        .position(self.isometry() * shape_instance.isometry)
                        .enabled(!shape_instance.disabled)
                        .collision_groups(collision_groups)
                }
            },
        )
    }

    fn set_collision_layer(&mut self, layer: u32);
    fn get_collision_layer(&self) -> u32;
    fn set_collision_mask(&mut self, mask: u32);
    fn get_collision_mask(&self) -> u32;
}
