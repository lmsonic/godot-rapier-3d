#![allow(unused, non_snake_case)]
use std::cell::RefCell;
use std::rc::Rc;

use godot::engine::PhysicsDirectBodyState3DExtensionVirtual;
use godot::prelude::*;

use crate::body::RapierBody;

#[derive(GodotClass)]
#[class(base=PhysicsDirectBodyState3DExtension)]
pub struct RapierPhysicsDirectBodyState3D {
    body: Rc<RefCell<RapierBody>>,
}

impl RapierPhysicsDirectBodyState3D {
    pub fn new(body: Rc<RefCell<RapierBody>>) -> Self {
        Self { body }
    }
}

#[godot_api]
impl PhysicsDirectBodyState3DExtensionVirtual for RapierPhysicsDirectBodyState3D {
    fn get_total_gravity(&self) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn get_total_linear_damp(&self) -> f32 {
        // TODO
        0.0
    }
    fn get_total_angular_damp(&self) -> f32 {
        // TODO
        0.0
    }
    fn get_center_of_mass(&self) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn get_center_of_mass_local(&self) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn get_principal_inertia_axes(&self) -> Basis {
        // TODO
        Basis::IDENTITY
    }
    fn get_inverse_mass(&self) -> f32 {
        // TODO
        0.0
    }
    fn get_inverse_inertia(&self) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn get_inverse_inertia_tensor(&self) -> Basis {
        // TODO
        Basis::IDENTITY
    }
    fn set_linear_velocity(&mut self, velocity: Vector3) {
        // TODO
    }
    fn get_linear_velocity(&self) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn set_angular_velocity(&mut self, velocity: Vector3) {
        // TODO
    }
    fn get_angular_velocity(&self) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn set_transform(&mut self, transform: Transform3D) {
        // TODO
    }
    fn get_transform(&self) -> Transform3D {
        // TODO
        Transform3D::IDENTITY
    }
    fn get_velocity_at_local_position(&self, local_position: Vector3) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn apply_central_impulse(&mut self, impulse: Vector3) {
        // TODO
    }
    fn apply_impulse(&mut self, impulse: Vector3, position: Vector3) {
        // TODO
    }
    fn apply_torque_impulse(&mut self, impulse: Vector3) {
        // TODO
    }
    fn apply_central_force(&mut self, force: Vector3) {
        // TODO
    }
    fn apply_force(&mut self, force: Vector3, position: Vector3) {
        // TODO
    }
    fn apply_torque(&mut self, torque: Vector3) {
        // TODO
    }
    fn add_constant_central_force(&mut self, force: Vector3) {
        // TODO
    }
    fn add_constant_force(&mut self, force: Vector3, position: Vector3) {
        // TODO
    }
    fn add_constant_torque(&mut self, torque: Vector3) {
        // TODO
    }
    fn set_constant_force(&mut self, force: Vector3) {
        // TODO
    }
    fn get_constant_force(&self) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn set_constant_torque(&mut self, torque: Vector3) {
        // TODO
    }
    fn get_constant_torque(&self) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn set_sleep_state(&mut self, enabled: bool) {
        // TODO
    }
    fn is_sleeping(&self) -> bool {
        // TODO
        false
    }
    fn get_contact_count(&self) -> i32 {
        // TODO
        0
    }
    fn get_contact_local_position(&self, contact_idx: i32) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn get_contact_local_normal(&self, contact_idx: i32) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn get_contact_impulse(&self, contact_idx: i32) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn get_contact_local_shape(&self, contact_idx: i32) -> i32 {
        // TODO
        0
    }
    fn get_contact_local_velocity_at_position(&self, contact_idx: i32) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn get_contact_collider(&self, contact_idx: i32) -> Rid {
        // TODO
        Rid::Invalid
    }
    fn get_contact_collider_position(&self, contact_idx: i32) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn get_contact_collider_id(&self, contact_idx: i32) -> u64 {
        // TODO
        0
    }
    fn get_contact_collider_object(&self, contact_idx: i32) -> Option<Gd<godot::engine::Object>> {
        // TODO
        None
    }
    fn get_contact_collider_shape(&self, contact_idx: i32) -> i32 {
        // TODO
        0
    }
    fn get_contact_collider_velocity_at_position(&self, contact_idx: i32) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn get_step(&self) -> f32 {
        // TODO
        0.0
    }
    fn integrate_forces(&mut self) {
        // TODO
    }
    fn get_space_state(&mut self) -> Option<Gd<godot::engine::PhysicsDirectSpaceState3D>> {
        // TODO
        None
    }
}
