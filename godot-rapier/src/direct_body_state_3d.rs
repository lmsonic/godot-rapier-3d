#![allow(unused, non_snake_case)]
use std::cell::RefCell;
use std::rc::{Rc, Weak};

use godot::engine::PhysicsDirectBodyState3DExtensionVirtual;
use godot::prelude::*;

use crate::body::RapierBody;
use crate::collision_object::RapierCollisionObject;

#[derive(GodotClass)]
#[class(base=PhysicsDirectBodyState3DExtension)]
pub struct RapierPhysicsDirectBodyState3D {
    body: Weak<RefCell<RapierBody>>,
}

impl RapierPhysicsDirectBodyState3D {
    pub fn new(body: Weak<RefCell<RapierBody>>) -> Self {
        Self { body }
    }
}

#[godot_api]
impl PhysicsDirectBodyState3DExtensionVirtual for RapierPhysicsDirectBodyState3D {
    fn get_total_gravity(&self) -> Vector3 {
        self.body.upgrade().unwrap().borrow().total_gravity()
    }
    fn get_total_linear_damp(&self) -> f32 {
        self.body.upgrade().unwrap().borrow().total_linear_damp()
    }
    fn get_total_angular_damp(&self) -> f32 {
        self.body.upgrade().unwrap().borrow().total_angular_damp()
    }
    fn get_center_of_mass(&self) -> Vector3 {
        self.body.upgrade().unwrap().borrow().center_of_mass()
    }
    fn get_center_of_mass_local(&self) -> Vector3 {
        self.body.upgrade().unwrap().borrow().local_center_of_mass()
    }
    fn get_principal_inertia_axes(&self) -> Basis {
        self.body
            .upgrade()
            .unwrap()
            .borrow()
            .principal_inertia_axes()
    }
    fn get_inverse_mass(&self) -> f32 {
        self.body.upgrade().unwrap().borrow().inverse_mass()
    }
    fn get_inverse_inertia(&self) -> Vector3 {
        self.body.upgrade().unwrap().borrow().inverse_inertia()
    }
    fn get_inverse_inertia_tensor(&self) -> Basis {
        self.body
            .upgrade()
            .unwrap()
            .borrow()
            .inverse_inertia_tensor()
    }
    fn set_linear_velocity(&mut self, velocity: Vector3) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .set_linear_velocity(velocity);
    }
    fn get_linear_velocity(&self) -> Vector3 {
        self.body.upgrade().unwrap().borrow().linear_velocity()
    }
    fn set_angular_velocity(&mut self, velocity: Vector3) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .set_angular_velocity(velocity);
    }
    fn get_angular_velocity(&self) -> Vector3 {
        self.body.upgrade().unwrap().borrow().angular_velocity()
    }
    fn set_transform(&mut self, transform: Transform3D) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .set_transform(transform);
    }
    fn get_transform(&self) -> Transform3D {
        self.body.upgrade().unwrap().borrow().transform()
    }
    fn get_velocity_at_local_position(&self, local_position: Vector3) -> Vector3 {
        // TODO
        Vector3::ZERO
    }
    fn apply_central_impulse(&mut self, impulse: Vector3) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .apply_central_impulse(impulse);
    }
    fn apply_impulse(&mut self, impulse: Vector3, position: Vector3) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .apply_impulse(impulse, position);
    }
    fn apply_torque_impulse(&mut self, impulse: Vector3) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .apply_torque_impulse(impulse);
    }
    fn apply_central_force(&mut self, force: Vector3) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .apply_central_force(force);
    }
    fn apply_force(&mut self, force: Vector3, position: Vector3) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .apply_force(force, position);
    }
    fn apply_torque(&mut self, torque: Vector3) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .apply_torque(torque);
    }
    fn add_constant_central_force(&mut self, force: Vector3) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .add_constant_central_force(force);
    }
    fn add_constant_force(&mut self, force: Vector3, position: Vector3) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .add_constant_force(force, position);
    }
    fn add_constant_torque(&mut self, torque: Vector3) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .add_constant_torque(torque);
    }
    fn set_constant_force(&mut self, force: Vector3) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .set_constant_force(force);
    }
    fn get_constant_force(&self) -> Vector3 {
        self.body.upgrade().unwrap().borrow().constant_force_godot()
    }
    fn set_constant_torque(&mut self, torque: Vector3) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .set_constant_torque(torque);
    }
    fn get_constant_torque(&self) -> Vector3 {
        self.body
            .upgrade()
            .unwrap()
            .borrow()
            .constant_torque_godot()
    }
    fn set_sleep_state(&mut self, enabled: bool) {
        self.body
            .upgrade()
            .unwrap()
            .borrow_mut()
            .set_is_sleeping(enabled);
    }
    fn is_sleeping(&self) -> bool {
        self.body.upgrade().unwrap().borrow().is_sleeping()
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
        if let Some(space) = self.body.upgrade().unwrap().borrow().space() {
            return space.borrow().get_step();
        }
        1.0 / 60.0
    }
    fn integrate_forces(&mut self) {
        let step = self.get_step();
        let mut linear_velocity = self.get_linear_velocity();
        let mut angular_velocity = self.get_angular_velocity();

        linear_velocity += self.get_total_gravity() * step;

        linear_velocity *= f32::max(self.get_total_linear_damp().mul_add(-step, 1.0), 0.0);
        angular_velocity *= f32::max(self.get_total_angular_damp().mul_add(-step, 1.0), 0.0);

        self.set_linear_velocity(linear_velocity);
        self.set_angular_velocity(angular_velocity);
    }
    fn get_space_state(&mut self) -> Option<Gd<godot::engine::PhysicsDirectSpaceState3D>> {
        if let Some(space) = self.body.upgrade().unwrap().borrow().space() {
            return space.borrow().direct_state();
        }
        None
    }
}
