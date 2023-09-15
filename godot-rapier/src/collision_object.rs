#![allow(clippy::cast_sign_loss)]
use godot::prelude::*;

use crate::shapes::ShapeInstance;

pub trait RapierCollisionObject {
    fn shapes(&self) -> &[ShapeInstance];
    fn shapes_mut(&mut self) -> &mut Vec<ShapeInstance>;

    fn add_shape(&mut self, shape: Rid, transform: Transform3D, disabled: bool) {
        self.shapes_mut()
            .push(ShapeInstance::new(shape, transform, disabled));
    }
    #[must_use]
    fn remove_shape(&mut self, shape_idx: i32) -> Rid {
        let old_shape = self.shapes_mut().remove(shape_idx as usize);
        old_shape.shape
    }
    fn remove_shape_rid(&mut self, rid: Rid) {
        self.shapes_mut().retain(|s| s.shape != rid);
    }
    #[must_use]
    fn clear_shapes(&mut self) -> Vec<Rid> {
        let old_shapes = self.shapes().iter().map(|s| s.shape).collect();
        self.shapes_mut().clear();
        old_shapes
    }

    #[must_use]
    fn set_shape(&mut self, shape_idx: i32, shape: Rid) -> Rid {
        let shape_idx = shape_idx as usize;
        let old_shape = self.shapes()[shape_idx].shape;
        self.shapes_mut()[shape_idx].shape = shape;
        old_shape
    }
    fn set_shape_transform(&mut self, shape_idx: i32, transform: Transform3D) {
        self.shapes_mut()[shape_idx as usize].transform = transform;
    }
    fn set_shape_disabled(&mut self, shape_idx: i32, disabled: bool) {
        self.shapes_mut()[shape_idx as usize].disabled = disabled;
    }

    fn get_shape(&self, shape_idx: i32) -> Rid {
        self.shapes()[shape_idx as usize].shape
    }
    fn get_shape_transform(&self, shape_idx: i32) -> Transform3D {
        self.shapes()[shape_idx as usize].transform
    }

    fn set_instance_id(&mut self, instance_id: u64);

    fn get_instance_id(&self) -> Option<u64>;
    fn set_collision_layer(&mut self, layer: u32);
    fn get_collision_layer(&self) -> u32;
    fn set_collision_mask(&mut self, mask: u32);
    fn get_collision_mask(&self) -> u32;
}
