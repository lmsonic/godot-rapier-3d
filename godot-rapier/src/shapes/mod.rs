#![allow(clippy::module_name_repetitions)]

use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::prelude::*;

use crate::collision_object::RapierCollisionObject;

pub mod box_shape;
pub mod capsule_shape;
pub mod concave_shape;
pub mod convex_shape;
pub mod cylinder_shape;
pub mod heightfield_shape;
pub mod separation_ray_shape;
pub mod sphere_shape;
pub mod world_boundary_shape;

pub use self::box_shape::RapierBoxShape;
pub use self::capsule_shape::RapierCapsuleShape;
pub use self::concave_shape::RapierConcaveShape;
pub use self::convex_shape::RapierConvexShape;
pub use self::cylinder_shape::RapierCylinderShape;
pub use self::heightfield_shape::RapierHeightmapShape;
pub use self::separation_ray_shape::RapierSeparationRayShape;
pub use self::sphere_shape::RapierSphereShape;
pub use self::world_boundary_shape::RapierWorldBoundaryShape;

const DEFAULT_SOLVER_BIAS: f32 = 0.0;
pub trait RapierShape {
    fn rid(&self) -> Rid;
    fn data(&self) -> Variant;
    fn set_data(&mut self, data: Variant);
    fn shared_shape(&self, scale: Vector<f32>) -> SharedShape;
    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType;
    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>>;
    fn update_owners(&self) {
        for owner in self.owners() {
            owner.borrow_mut().update_shapes();
        }
    }
    fn remove_from_owners(&self) {
        for owner in self.owners() {
            owner.borrow_mut().remove_shape_rid(self.rid());
        }
    }

    fn set_solver_bias(&mut self, bias: f32) {
        godot_warn!("Custom solver bias for shapes is not supported by Godot Rapier.");
    }
    fn solver_bias(&self) -> f32 {
        DEFAULT_SOLVER_BIAS
    }
    fn set_margin(&mut self, margin: f32) {}
    fn margin(&self) -> f32 {
        0.0
    }
}

pub struct RapierShapeInstance {
    pub shape: Rc<RefCell<dyn RapierShape>>,
    pub isometry: Isometry<f32>,
    pub disabled: bool,
    pub scale: Vector<f32>,
}

impl RapierShapeInstance {
    pub fn shared_shape(&self) -> SharedShape {
        self.shape.borrow().shared_shape(self.scale)
    }
}

impl RapierShapeInstance {
    pub fn new(
        shape: Rc<RefCell<dyn RapierShape>>,
        isometry: Isometry<f32>,
        scale: Vector<f32>,
        disabled: bool,
    ) -> Self {
        Self {
            shape,
            isometry,
            disabled,
            scale,
        }
    }
}
