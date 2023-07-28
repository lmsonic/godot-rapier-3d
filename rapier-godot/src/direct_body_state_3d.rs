use godot::engine::{PhysicsDirectBodyState3DExtension, PhysicsDirectBodyState3DExtensionVirtual};
use godot::prelude::*;

#[derive(GodotClass)]
#[class(base=PhysicsDirectBodyState3DExtension)]
struct RapierPhysicsDirectBodyState3D {
    #[base]
    base: Base<PhysicsDirectBodyState3DExtension>,
}
#[godot_api]
impl PhysicsDirectBodyState3DExtensionVirtual for RapierPhysicsDirectBodyState3D {
    fn get_total_gravity(&self) -> Vector3 {
        unimplemented!()
    }
    fn get_total_linear_damp(&self) -> f32 {
        unimplemented!()
    }
    fn get_total_angular_damp(&self) -> f32 {
        unimplemented!()
    }
    fn get_center_of_mass(&self) -> Vector3 {
        unimplemented!()
    }
    fn get_center_of_mass_local(&self) -> Vector3 {
        unimplemented!()
    }
    fn get_principal_inertia_axes(&self) -> Basis {
        unimplemented!()
    }
    fn get_inverse_mass(&self) -> f32 {
        unimplemented!()
    }
    fn get_inverse_inertia(&self) -> Vector3 {
        unimplemented!()
    }
    fn get_inverse_inertia_tensor(&self) -> Basis {
        unimplemented!()
    }
    fn set_linear_velocity(&mut self, velocity: Vector3) {
        unimplemented!()
    }
    fn get_linear_velocity(&self) -> Vector3 {
        unimplemented!()
    }
    fn set_angular_velocity(&mut self, velocity: Vector3) {
        unimplemented!()
    }
    fn get_angular_velocity(&self) -> Vector3 {
        unimplemented!()
    }
    fn set_transform(&mut self, transform: Transform3D) {
        unimplemented!()
    }
    fn get_transform(&self) -> Transform3D {
        unimplemented!()
    }
    fn get_velocity_at_local_position(&self, local_position: Vector3) -> Vector3 {
        unimplemented!()
    }
    fn apply_central_impulse(&mut self, impulse: Vector3) {
        unimplemented!()
    }
    fn apply_impulse(&mut self, impulse: Vector3, position: Vector3) {
        unimplemented!()
    }
    fn apply_torque_impulse(&mut self, impulse: Vector3) {
        unimplemented!()
    }
    fn apply_central_force(&mut self, force: Vector3) {
        unimplemented!()
    }
    fn apply_force(&mut self, force: Vector3, position: Vector3) {
        unimplemented!()
    }
    fn apply_torque(&mut self, torque: Vector3) {
        unimplemented!()
    }
    fn add_constant_central_force(&mut self, force: Vector3) {
        unimplemented!()
    }
    fn add_constant_force(&mut self, force: Vector3, position: Vector3) {
        unimplemented!()
    }
    fn add_constant_torque(&mut self, torque: Vector3) {
        unimplemented!()
    }
    fn set_constant_force(&mut self, force: Vector3) {
        unimplemented!()
    }
    fn get_constant_force(&self) -> Vector3 {
        unimplemented!()
    }
    fn set_constant_torque(&mut self, torque: Vector3) {
        unimplemented!()
    }
    fn get_constant_torque(&self) -> Vector3 {
        unimplemented!()
    }
    fn set_sleep_state(&mut self, enabled: bool) {
        unimplemented!()
    }
    fn is_sleeping(&self) -> bool {
        unimplemented!()
    }
    fn get_contact_count(&self) -> i32 {
        unimplemented!()
    }
    fn get_contact_local_position(&self, contact_idx: i32) -> Vector3 {
        unimplemented!()
    }
    fn get_contact_local_normal(&self, contact_idx: i32) -> Vector3 {
        unimplemented!()
    }
    fn get_contact_impulse(&self, contact_idx: i32) -> Vector3 {
        unimplemented!()
    }
    fn get_contact_local_shape(&self, contact_idx: i32) -> i32 {
        unimplemented!()
    }
    fn get_contact_local_velocity_at_position(&self, contact_idx: i32) -> Vector3 {
        unimplemented!()
    }
    fn get_contact_collider(&self, contact_idx: i32) -> Rid {
        unimplemented!()
    }
    fn get_contact_collider_position(&self, contact_idx: i32) -> Vector3 {
        unimplemented!()
    }
    fn get_contact_collider_id(&self, contact_idx: i32) -> u64 {
        unimplemented!()
    }
    fn get_contact_collider_object(&self, contact_idx: i32) -> Option<Gd<godot::engine::Object>> {
        unimplemented!()
    }
    fn get_contact_collider_shape(&self, contact_idx: i32) -> i32 {
        unimplemented!()
    }
    fn get_contact_collider_velocity_at_position(&self, contact_idx: i32) -> Vector3 {
        unimplemented!()
    }
    fn get_step(&self) -> f32 {
        unimplemented!()
    }
    fn integrate_forces(&mut self) {
        unimplemented!()
    }
    fn get_space_state(&mut self) -> Option<Gd<godot::engine::PhysicsDirectSpaceState3D>> {
        unimplemented!()
    }
}
