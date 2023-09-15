use godot::{engine::physics_server_3d::ShapeType, prelude::*};

pub mod box_shape;
pub mod capsule_shape;
pub mod concave_polygon_shape;
pub mod convex_polygon_shape;
pub mod cylinder_shape;
pub mod heightmap_shape;
pub mod separation_ray_shape;
pub mod sphere_shape;
pub mod world_boundary_shape;

#[allow(unused)]
const DEFAULT_SOLVER_BIAS: f32 = 0.0;
pub struct ShapeInstance {
    pub shape: Rid,
    pub transform: Transform3D,
    pub disabled: bool,
}

impl ShapeInstance {
    pub const fn new(shape: Rid, transform: Transform3D, disabled: bool) -> Self {
        Self {
            shape,
            transform,
            disabled,
        }
    }
}
pub trait RapierShape {
    fn set_data(&mut self, data: Variant);
    fn set_custom_solver_bias(&mut self, _bias: f32) {
        godot_warn!(
            "Custom solver bias for shapes is not supported by Godot Rapier.
Any such value will be ignored.
This shape belongs to {:?}.",
            self.owners()
        );
    }
    fn get_custom_solver_bias(&self) -> f32 {
        DEFAULT_SOLVER_BIAS
    }
    fn set_margin(&mut self, _margin: f32) {}

    fn get_margin(&self) -> f32 {
        0.0
    }
    fn get_shape_type(&self) -> ShapeType;
    fn get_data(&self) -> Variant;
    fn add_owner(&mut self, owner: Rid);
    fn remove_owner(&mut self, owner: Rid);
    fn owners(&self) -> &[Rid];
}
