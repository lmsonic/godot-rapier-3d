#![allow(
    unused,
    non_snake_case,
    clippy::option_if_let_else,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_precision_loss
)]
use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::f32::consts::E;
use std::rc::Rc;
use std::sync::Arc;

use godot::engine::native::PhysicsServer3DExtensionMotionResult;
use godot::engine::physics_server_3d::{BodyMode, SpaceParameter};
use godot::engine::PhysicsServer3DExtensionVirtual;
use godot::prelude::utilities::{rid_allocate_id, rid_from_int64};
use godot::prelude::*;
use rapier3d::prelude::*;

use crate::area::RapierArea;
use crate::body::RapierBody;
use crate::collision_object::RapierCollisionObject;
use crate::conversions::{isometry_to_transform, transform_to_isometry};
use crate::error::{RapierError, RapierResult};
use crate::joint::RapierJoint;
use crate::physics_server_3d_utils::make_rid;
use crate::shapes::{
    RapierBoxShape, RapierCapsuleShape, RapierConcaveShape, RapierConvexShape, RapierCylinderShape,
    RapierHeightmapShape, RapierSeparationRayShape, RapierShape, RapierSphereShape,
    RapierWorldBoundaryShape,
};
use crate::space::RapierSpace;

#[derive(GodotClass, Default)]
#[class(base=PhysicsServer3DExtension,init)]
pub struct RapierPhysicsServer3D {
    pub(crate) shapes: HashMap<Rid, Rc<RefCell<dyn RapierShape>>>,
    pub(crate) spaces: HashMap<Rid, Rc<RefCell<RapierSpace>>>,
    active_spaces: HashSet<Rid>,
    pub(crate) areas: HashMap<Rid, Rc<RefCell<RapierArea>>>,
    pub(crate) bodies: HashMap<Rid, Rc<RefCell<RapierBody>>>,
    pub(crate) joints: HashMap<Rid, Rc<RefCell<RapierJoint>>>,
    active: bool,
}

#[godot_api]
impl PhysicsServer3DExtensionVirtual for RapierPhysicsServer3D {
    fn world_boundary_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        let shape = RapierWorldBoundaryShape::new(rid);
        self.shapes.insert(rid, Rc::new(RefCell::new(shape)));
        rid
    }
    fn separation_ray_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        let shape = RapierSeparationRayShape::new(rid);
        self.shapes.insert(rid, Rc::new(RefCell::new(shape)));
        rid
    }
    fn sphere_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        let shape = RapierSphereShape::new(rid);
        self.shapes.insert(rid, Rc::new(RefCell::new(shape)));
        rid
    }
    fn box_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        let shape = RapierBoxShape::new(rid);
        self.shapes.insert(rid, Rc::new(RefCell::new(shape)));
        rid
    }
    fn capsule_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        let shape = RapierCapsuleShape::new(rid);
        self.shapes.insert(rid, Rc::new(RefCell::new(shape)));
        rid
    }
    fn cylinder_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        let shape = RapierCylinderShape::new(rid);
        self.shapes.insert(rid, Rc::new(RefCell::new(shape)));
        rid
    }
    fn convex_polygon_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        let shape = RapierConvexShape::new(rid);
        self.shapes.insert(rid, Rc::new(RefCell::new(shape)));
        rid
    }
    fn concave_polygon_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        let shape = RapierConcaveShape::new(rid);
        self.shapes.insert(rid, Rc::new(RefCell::new(shape)));
        rid
    }
    fn heightmap_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        let shape = RapierHeightmapShape::new(rid);
        self.shapes.insert(rid, Rc::new(RefCell::new(shape)));
        rid
    }
    fn custom_shape_create(&mut self) -> Rid {
        Rid::Invalid
    }

    fn shape_set_data(&mut self, shape: Rid, data: Variant) {
        if let Ok(shape) = self.get_shape(shape) {
            shape.borrow_mut().set_data(data);
        }
    }
    fn shape_set_custom_solver_bias(&mut self, shape: Rid, bias: f32) {
        unimplemented!()
    }
    fn shape_set_margin(&mut self, shape: Rid, margin: f32) {
        unimplemented!()
    }
    fn shape_get_margin(&self, shape: Rid) -> f32 {
        unimplemented!()
    }
    fn shape_get_type(&self, shape: Rid) -> godot::engine::physics_server_3d::ShapeType {
        if let Ok(shape) = self.get_shape(shape) {
            return shape.borrow().get_type();
        }
        godot::engine::physics_server_3d::ShapeType::SHAPE_CUSTOM
    }
    fn shape_get_data(&self, shape: Rid) -> Variant {
        if let Ok(shape) = self.get_shape(shape) {
            return shape.borrow().get_data();
        }
        Variant::nil()
    }
    fn shape_get_custom_solver_bias(&self, shape: Rid) -> f32 {
        unimplemented!()
    }
    fn space_create(&mut self) -> Rid {
        let rid = make_rid();
        let space = RapierSpace::new(rid);
        self.spaces.insert(rid, Rc::new(RefCell::new(space)));
        rid
    }
    fn space_set_active(&mut self, space: Rid, active: bool) {
        if self.has_space(space) {
            if active {
                self.active_spaces.insert(space);
            } else {
                self.active_spaces.remove(&space);
            }
        }
    }
    fn space_is_active(&self, space: Rid) -> bool {
        if self.has_space(space) {
            return self.active_spaces.contains(&space);
        }
        false
    }
    fn space_set_param(&mut self, space: Rid, param: SpaceParameter, value: f32) {
        if let Ok(space) = self.get_space(space) {
            space.borrow_mut().set_param(param, value);
        }
    }
    fn space_get_param(&self, space: Rid, param: SpaceParameter) -> f32 {
        if let Ok(space) = self.get_space(space) {
            space.borrow().get_param(param);
        }
        0.0
    }
    fn space_get_direct_state(
        &mut self,
        space: Rid,
    ) -> Option<Gd<godot::engine::PhysicsDirectSpaceState3D>> {
        unimplemented!()
    }
    fn space_set_debug_contacts(&mut self, space: Rid, max_contacts: i32) {
        unimplemented!()
    }
    fn space_get_contacts(&self, space: Rid) -> PackedVector3Array {
        unimplemented!()
    }
    fn space_get_contact_count(&self, space: Rid) -> i32 {
        unimplemented!()
    }
    fn area_create(&mut self) -> Rid {
        let rid = make_rid();
        let area = RapierArea::new(rid);
        self.areas.insert(rid, Rc::new(RefCell::new(area)));
        rid
    }
    fn area_set_space(&mut self, area: Rid, space: Rid) {
        if let Ok(area) = self.get_area(area) {
            if let Ok(space) = self.get_space(space) {
                space.borrow_mut().add_area(area);
                area.borrow_mut().set_space(space.clone());
            }
        }
    }
    fn area_get_space(&self, area: Rid) -> Rid {
        if let Ok(area) = self.get_area(area) {
            if let Some(space) = area.borrow().space() {
                let rid = space.borrow().rid();
                if self.has_space(rid) {
                    return rid;
                }
            }
        }
        Rid::Invalid
    }
    fn area_add_shape(&mut self, area: Rid, shape: Rid, transform: Transform3D, disabled: bool) {
        if let Ok(area) = self.get_area(area) {
            if let Ok(shape) = self.get_shape(shape) {
                area.borrow_mut()
                    .add_shape(shape.clone(), transform, disabled);
            }
        }
    }
    fn area_set_shape(&mut self, area: Rid, shape_idx: i32, shape: Rid) {
        if let Ok(area) = self.get_area(area) {
            if let Ok(shape) = self.get_shape(shape) {
                area.borrow_mut()
                    .set_shape(shape_idx as usize, shape.clone());
            }
        }
    }
    fn area_set_shape_transform(&mut self, area: Rid, shape_idx: i32, transform: Transform3D) {
        if let Ok(area) = self.get_area(area) {
            area.borrow_mut()
                .set_shape_transform(shape_idx as usize, transform);
        }
    }
    fn area_set_shape_disabled(&mut self, area: Rid, shape_idx: i32, disabled: bool) {
        if let Ok(area) = self.get_area(area) {
            area.borrow_mut()
                .set_shape_disabled(shape_idx as usize, disabled);
        }
    }
    fn area_get_shape_count(&self, area: Rid) -> i32 {
        if let Ok(area) = self.get_area(area) {
            return area.borrow().shapes().len() as i32;
        }
        0
    }
    fn area_get_shape(&self, area: Rid, shape_idx: i32) -> Rid {
        if let Ok(area) = self.get_area(area) {
            if let Some(shape_inst) = area.borrow().get_shape_instance(shape_idx as usize) {
                return shape_inst.shape.borrow().rid();
            }
        }
        Rid::Invalid
    }
    fn area_get_shape_transform(&self, area: Rid, shape_idx: i32) -> Transform3D {
        if let Ok(area) = self.get_area(area) {
            if let Some(shape_inst) = area.borrow().get_shape_instance(shape_idx as usize) {
                return isometry_to_transform(&shape_inst.isometry);
            }
        }
        Transform3D::IDENTITY
    }
    fn area_remove_shape(&mut self, area: Rid, shape_idx: i32) {
        if let Ok(area) = self.get_area(area) {
            area.borrow_mut().remove_nth_shape(shape_idx as usize);
        }
    }
    fn area_clear_shapes(&mut self, area: Rid) {
        if let Ok(area) = self.get_area(area) {
            area.borrow_mut().clear_shapes();
        }
    }
    fn area_attach_object_instance_id(&mut self, area: Rid, id: u64) {
        if let Ok(area) = self.get_area(area) {
            area.borrow_mut().set_instance_id(id);
        }
    }
    fn area_get_object_instance_id(&self, area: Rid) -> u64 {
        if let Ok(area) = self.get_area(area) {
            if let Some(id) = area.borrow().instance_id() {
                return id;
            }
        }
        0
    }
    fn area_set_param(
        &mut self,
        area: Rid,
        param: godot::engine::physics_server_3d::AreaParameter,
        value: Variant,
    ) {
        if let Ok(area) = self.get_area(area) {
            area.borrow_mut().set_param(param, value);
        }
    }
    fn area_set_transform(&mut self, area: Rid, transform: Transform3D) {
        if let Ok(area) = self.get_area(area) {
            area.borrow_mut().set_transform(transform);
        }
    }
    fn area_get_param(
        &self,
        area: Rid,
        param: godot::engine::physics_server_3d::AreaParameter,
    ) -> Variant {
        if let Ok(area) = self.get_area(area) {
            return area.borrow().get_param(param);
        }
        Variant::nil()
    }
    fn area_get_transform(&self, area: Rid) -> Transform3D {
        if let Ok(area) = self.get_area(area) {
            if let Some(transform) = area.borrow().get_transform() {
                return transform;
            }
        }
        Transform3D::IDENTITY
    }
    fn area_set_collision_layer(&mut self, area: Rid, layer: u32) {
        unimplemented!()
    }
    fn area_get_collision_layer(&self, area: Rid) -> u32 {
        unimplemented!()
    }
    fn area_set_collision_mask(&mut self, area: Rid, mask: u32) {
        unimplemented!()
    }
    fn area_get_collision_mask(&self, area: Rid) -> u32 {
        unimplemented!()
    }
    fn area_set_monitorable(&mut self, area: Rid, monitorable: bool) {
        if let Ok(area) = self.get_area(area) {
            area.borrow_mut().set_monitorable(monitorable);
        }
    }
    fn area_set_ray_pickable(&mut self, area: Rid, enable: bool) {
        unimplemented!()
    }
    fn area_set_monitor_callback(&mut self, area: Rid, callback: Callable) {
        if let Ok(area) = self.get_area(area) {
            area.borrow_mut().set_body_monitor_callback(callback);
        }
    }
    fn area_set_area_monitor_callback(&mut self, area: Rid, callback: Callable) {
        if let Ok(area) = self.get_area(area) {
            area.borrow_mut().set_area_monitor_callback(callback);
        }
    }
    fn body_create(&mut self) -> Rid {
        let rid = make_rid();
        let body = RapierBody::new(rid);
        self.bodies.insert(rid, Rc::new(RefCell::new(body)));
        rid
    }

    fn body_set_space(&mut self, body: Rid, space: Rid) {
        if let Ok(body) = self.get_body(body) {
            if let Ok(space) = self.get_space(space) {
                space.borrow_mut().add_body(body);
                body.borrow_mut().set_space(space.clone());
            }
        }
    }
    fn body_get_space(&self, body: Rid) -> Rid {
        if let Ok(body) = self.get_body(body) {
            if let Some(space) = body.borrow().space() {
                let rid = space.borrow().rid();
                if self.has_space(rid) {
                    return rid;
                }
            }
        }
        Rid::Invalid
    }
    fn body_set_mode(&mut self, body: Rid, mode: BodyMode) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().set_body_mode(mode);
        }
    }
    fn body_get_mode(&self, body: Rid) -> BodyMode {
        if let Ok(body) = self.get_body(body) {
            return body.borrow().get_body_mode();
        }
        BodyMode::BODY_MODE_STATIC
    }
    fn body_add_shape(&mut self, body: Rid, shape: Rid, transform: Transform3D, disabled: bool) {
        if let Ok(body) = self.get_body(body) {
            if let Ok(shape) = self.get_shape(shape) {
                body.borrow_mut()
                    .add_shape(shape.clone(), transform, disabled);
            }
        }
    }
    fn body_set_shape(&mut self, body: Rid, shape_idx: i32, shape: Rid) {
        if let Ok(body) = self.get_body(body) {
            if let Ok(shape) = self.get_shape(shape) {
                body.borrow_mut()
                    .set_shape(shape_idx as usize, shape.clone());
            }
        }
    }
    fn body_set_shape_transform(&mut self, body: Rid, shape_idx: i32, transform: Transform3D) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut()
                .set_shape_transform(shape_idx as usize, transform);
        }
    }
    fn body_set_shape_disabled(&mut self, body: Rid, shape_idx: i32, disabled: bool) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut()
                .set_shape_disabled(shape_idx as usize, disabled);
        }
    }
    fn body_get_shape_count(&self, body: Rid) -> i32 {
        if let Ok(body) = self.get_body(body) {
            return body.borrow().shapes().len() as i32;
        }
        0
    }
    fn body_get_shape(&self, body: Rid, shape_idx: i32) -> Rid {
        if let Ok(body) = self.get_body(body) {
            if let Some(shape_inst) = body.borrow().get_shape_instance(shape_idx as usize) {
                return shape_inst.shape.borrow().rid();
            }
        }
        Rid::Invalid
    }
    fn body_get_shape_transform(&self, body: Rid, shape_idx: i32) -> Transform3D {
        if let Ok(body) = self.get_body(body) {
            if let Some(shape_inst) = body.borrow().get_shape_instance(shape_idx as usize) {
                return isometry_to_transform(&shape_inst.isometry);
            }
        }
        Transform3D::IDENTITY
    }
    fn body_remove_shape(&mut self, body: Rid, shape_idx: i32) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().remove_nth_shape(shape_idx as usize);
        }
    }
    fn body_clear_shapes(&mut self, body: Rid) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().clear_shapes();
        }
    }
    fn body_attach_object_instance_id(&mut self, body: Rid, id: u64) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().set_instance_id(id);
        }
    }
    fn body_get_object_instance_id(&self, body: Rid) -> u64 {
        if let Ok(body) = self.get_body(body) {
            if let Some(id) = body.borrow().instance_id() {
                return id;
            }
        }
        0
    }
    fn body_set_enable_continuous_collision_detection(&mut self, body: Rid, enable: bool) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().set_enable_ccd(enable);
        }
    }
    fn body_is_continuous_collision_detection_enabled(&self, body: Rid) -> bool {
        if let Ok(body) = self.get_body(body) {
            return body.borrow().is_ccd_enabled();
        }
        false
    }
    fn body_set_collision_layer(&mut self, body: Rid, layer: u32) {
        unimplemented!()
    }
    fn body_get_collision_layer(&self, body: Rid) -> u32 {
        unimplemented!()
    }
    fn body_set_collision_mask(&mut self, body: Rid, mask: u32) {
        unimplemented!()
    }
    fn body_get_collision_mask(&self, body: Rid) -> u32 {
        unimplemented!()
    }
    fn body_set_collision_priority(&mut self, body: Rid, priority: f32) {
        unimplemented!()
    }
    fn body_get_collision_priority(&self, body: Rid) -> f32 {
        unimplemented!()
    }
    fn body_set_user_flags(&mut self, body: Rid, flags: u32) {
        unimplemented!()
    }
    fn body_get_user_flags(&self, body: Rid) -> u32 {
        unimplemented!()
    }
    fn body_set_param(
        &mut self,
        body: Rid,
        param: godot::engine::physics_server_3d::BodyParameter,
        value: Variant,
    ) {
        unimplemented!()
    }
    fn body_get_param(
        &self,
        body: Rid,
        param: godot::engine::physics_server_3d::BodyParameter,
    ) -> Variant {
        unimplemented!()
    }
    fn body_reset_mass_properties(&mut self, body: Rid) {
        unimplemented!()
    }
    fn body_set_state(
        &mut self,
        body: Rid,
        state: godot::engine::physics_server_3d::BodyState,
        value: Variant,
    ) {
        unimplemented!()
    }
    fn body_get_state(
        &self,
        body: Rid,
        state: godot::engine::physics_server_3d::BodyState,
    ) -> Variant {
        unimplemented!()
    }
    fn body_apply_central_impulse(&mut self, body: Rid, impulse: Vector3) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().apply_central_impulse(impulse);
        }
    }
    fn body_apply_impulse(&mut self, body: Rid, impulse: Vector3, position: Vector3) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().apply_impulse(impulse, position);
        }
    }
    fn body_apply_torque_impulse(&mut self, body: Rid, impulse: Vector3) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().apply_torque_impulse(impulse);
        }
    }
    fn body_apply_central_force(&mut self, body: Rid, force: Vector3) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().apply_central_force(force);
        }
    }
    fn body_apply_force(&mut self, body: Rid, force: Vector3, position: Vector3) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().apply_force(force, position);
        }
    }
    fn body_apply_torque(&mut self, body: Rid, torque: Vector3) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().apply_torque(torque);
        }
    }
    fn body_add_constant_central_force(&mut self, body: Rid, force: Vector3) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().add_constant_central_force(force);
        }
    }
    fn body_add_constant_force(&mut self, body: Rid, force: Vector3, position: Vector3) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().add_constant_force(force, position);
        }
    }
    fn body_add_constant_torque(&mut self, body: Rid, torque: Vector3) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().add_constant_torque(torque);
        }
    }
    fn body_set_constant_force(&mut self, body: Rid, force: Vector3) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().set_constant_force(force);
        }
    }
    fn body_get_constant_force(&self, body: Rid) -> Vector3 {
        if let Ok(body) = self.get_body(body) {
            return body.borrow().get_constant_force_godot();
        }
        Vector3::ZERO
    }
    fn body_set_constant_torque(&mut self, body: Rid, torque: Vector3) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().set_constant_torque(torque);
        }
    }
    fn body_get_constant_torque(&self, body: Rid) -> Vector3 {
        if let Ok(body) = self.get_body(body) {
            return body.borrow().get_constant_torque_godot();
        }
        Vector3::ZERO
    }
    fn body_set_axis_velocity(&mut self, body: Rid, axis_velocity: Vector3) {
        unimplemented!()
    }
    fn body_set_axis_lock(
        &mut self,
        body: Rid,
        axis: godot::engine::physics_server_3d::BodyAxis,
        lock: bool,
    ) {
        unimplemented!()
    }
    fn body_is_axis_locked(
        &self,
        body: Rid,
        axis: godot::engine::physics_server_3d::BodyAxis,
    ) -> bool {
        unimplemented!()
    }
    fn body_add_collision_exception(&mut self, body: Rid, excepted_body: Rid) {
        unimplemented!()
    }
    fn body_remove_collision_exception(&mut self, body: Rid, excepted_body: Rid) {
        unimplemented!()
    }
    fn body_get_collision_exceptions(&self, body: Rid) -> Array<Rid> {
        unimplemented!()
    }
    fn body_set_max_contacts_reported(&mut self, body: Rid, amount: i32) {
        unimplemented!()
    }
    fn body_get_max_contacts_reported(&self, body: Rid) -> i32 {
        unimplemented!()
    }
    fn body_set_contacts_reported_depth_threshold(&mut self, body: Rid, threshold: f32) {
        unimplemented!()
    }
    fn body_get_contacts_reported_depth_threshold(&self, body: Rid) -> f32 {
        unimplemented!()
    }
    fn body_set_omit_force_integration(&mut self, body: Rid, enable: bool) {
        unimplemented!()
    }
    fn body_is_omitting_force_integration(&self, body: Rid) -> bool {
        unimplemented!()
    }
    fn body_set_state_sync_callback(&mut self, body: Rid, callable: Callable) {
        if let Ok(body) = self.get_body(body) {
            body.borrow_mut().set_body_state_callback(callable);
        }
    }
    fn body_set_force_integration_callback(
        &mut self,
        body: Rid,
        callable: Callable,
        userdata: Variant,
    ) {
        unimplemented!()
    }
    fn body_set_ray_pickable(&mut self, body: Rid, enable: bool) {
        unimplemented!()
    }
    #[doc = "# Safety"]
    #[doc = ""]
    #[doc = "Godot currently does not document safety requirements on this method. Make sure you understand the underlying semantics."]
    unsafe fn body_test_motion(
        &self,
        body: Rid,
        from: Transform3D,
        motion: Vector3,
        margin: f32,
        max_collisions: i32,
        collide_separation_ray: bool,
        recovery_as_collision: bool,
        result: *mut PhysicsServer3DExtensionMotionResult,
    ) -> bool {
        unimplemented!()
    }
    fn body_get_direct_state(
        &mut self,
        body: Rid,
    ) -> Option<Gd<godot::engine::PhysicsDirectBodyState3D>> {
        unimplemented!()
    }
    fn soft_body_create(&mut self) -> Rid {
        unimplemented!()
    }
    fn soft_body_update_rendering_server(
        &mut self,
        body: Rid,
        rendering_server_handler: Gd<godot::engine::PhysicsServer3DRenderingServerHandler>,
    ) {
        unimplemented!()
    }
    fn soft_body_set_space(&mut self, body: Rid, space: Rid) {
        unimplemented!()
    }
    fn soft_body_get_space(&self, body: Rid) -> Rid {
        unimplemented!()
    }
    fn soft_body_set_ray_pickable(&mut self, body: Rid, enable: bool) {
        unimplemented!()
    }
    fn soft_body_set_collision_layer(&mut self, body: Rid, layer: u32) {
        unimplemented!()
    }
    fn soft_body_get_collision_layer(&self, body: Rid) -> u32 {
        unimplemented!()
    }
    fn soft_body_set_collision_mask(&mut self, body: Rid, mask: u32) {
        unimplemented!()
    }
    fn soft_body_get_collision_mask(&self, body: Rid) -> u32 {
        unimplemented!()
    }
    fn soft_body_add_collision_exception(&mut self, body: Rid, body_b: Rid) {
        unimplemented!()
    }
    fn soft_body_remove_collision_exception(&mut self, body: Rid, body_b: Rid) {
        unimplemented!()
    }
    fn soft_body_get_collision_exceptions(&self, body: Rid) -> Array<Rid> {
        unimplemented!()
    }
    fn soft_body_set_state(
        &mut self,
        body: Rid,
        state: godot::engine::physics_server_3d::BodyState,
        variant: Variant,
    ) {
        unimplemented!()
    }
    fn soft_body_get_state(
        &self,
        body: Rid,
        state: godot::engine::physics_server_3d::BodyState,
    ) -> Variant {
        unimplemented!()
    }
    fn soft_body_set_transform(&mut self, body: Rid, transform: Transform3D) {
        unimplemented!()
    }
    fn soft_body_set_simulation_precision(&mut self, body: Rid, simulation_precision: i32) {
        unimplemented!()
    }
    fn soft_body_get_simulation_precision(&self, body: Rid) -> i32 {
        unimplemented!()
    }
    fn soft_body_set_total_mass(&mut self, body: Rid, total_mass: f32) {
        unimplemented!()
    }
    fn soft_body_get_total_mass(&self, body: Rid) -> f32 {
        unimplemented!()
    }
    fn soft_body_set_linear_stiffness(&mut self, body: Rid, linear_stiffness: f32) {
        unimplemented!()
    }
    fn soft_body_get_linear_stiffness(&self, body: Rid) -> f32 {
        unimplemented!()
    }
    fn soft_body_set_pressure_coefficient(&mut self, body: Rid, pressure_coefficient: f32) {
        unimplemented!()
    }
    fn soft_body_get_pressure_coefficient(&self, body: Rid) -> f32 {
        unimplemented!()
    }
    fn soft_body_set_damping_coefficient(&mut self, body: Rid, damping_coefficient: f32) {
        unimplemented!()
    }
    fn soft_body_get_damping_coefficient(&self, body: Rid) -> f32 {
        unimplemented!()
    }
    fn soft_body_set_drag_coefficient(&mut self, body: Rid, drag_coefficient: f32) {
        unimplemented!()
    }
    fn soft_body_get_drag_coefficient(&self, body: Rid) -> f32 {
        unimplemented!()
    }
    fn soft_body_set_mesh(&mut self, body: Rid, mesh: Rid) {
        unimplemented!()
    }
    fn soft_body_get_bounds(&self, body: Rid) -> godot::builtin::Aabb {
        unimplemented!()
    }
    fn soft_body_move_point(&mut self, body: Rid, point_index: i32, global_position: Vector3) {
        unimplemented!()
    }
    fn soft_body_get_point_global_position(&self, body: Rid, point_index: i32) -> Vector3 {
        unimplemented!()
    }
    fn soft_body_remove_all_pinned_points(&mut self, body: Rid) {
        unimplemented!()
    }
    fn soft_body_pin_point(&mut self, body: Rid, point_index: i32, pin: bool) {
        unimplemented!()
    }
    fn soft_body_is_point_pinned(&self, body: Rid, point_index: i32) -> bool {
        unimplemented!()
    }
    fn joint_create(&mut self) -> Rid {
        let rid = make_rid();
        let joint = RapierJoint::new(rid);
        self.joints.insert(rid, Rc::new(RefCell::new(joint)));
        rid
    }
    fn joint_clear(&mut self, joint_id: Rid) {
        if let Ok(joint) = self.get_joint(joint_id) {
            let empty_joint = RapierJoint::new(joint_id);
            joint.replace(empty_joint);
        }
    }
    fn joint_make_pin(
        &mut self,
        joint: Rid,
        body_A: Rid,
        local_A: Vector3,
        body_B: Rid,
        local_B: Vector3,
    ) {
        unimplemented!()
    }
    fn pin_joint_set_param(
        &mut self,
        joint: Rid,
        param: godot::engine::physics_server_3d::PinJointParam,
        value: f32,
    ) {
        unimplemented!()
    }
    fn pin_joint_get_param(
        &self,
        joint: Rid,
        param: godot::engine::physics_server_3d::PinJointParam,
    ) -> f32 {
        unimplemented!()
    }
    fn pin_joint_set_local_a(&mut self, joint: Rid, local_A: Vector3) {
        unimplemented!()
    }
    fn pin_joint_get_local_a(&self, joint: Rid) -> Vector3 {
        unimplemented!()
    }
    fn pin_joint_set_local_b(&mut self, joint: Rid, local_B: Vector3) {
        unimplemented!()
    }
    fn pin_joint_get_local_b(&self, joint: Rid) -> Vector3 {
        unimplemented!()
    }
    fn joint_make_hinge(
        &mut self,
        joint: Rid,
        body_A: Rid,
        hinge_A: Transform3D,
        body_B: Rid,
        hinge_B: Transform3D,
    ) {
        unimplemented!()
    }
    fn joint_make_hinge_simple(
        &mut self,
        joint: Rid,
        body_A: Rid,
        pivot_A: Vector3,
        axis_A: Vector3,
        body_B: Rid,
        pivot_B: Vector3,
        axis_B: Vector3,
    ) {
        unimplemented!()
    }
    fn hinge_joint_set_param(
        &mut self,
        joint: Rid,
        param: godot::engine::physics_server_3d::HingeJointParam,
        value: f32,
    ) {
        unimplemented!()
    }
    fn hinge_joint_get_param(
        &self,
        joint: Rid,
        param: godot::engine::physics_server_3d::HingeJointParam,
    ) -> f32 {
        unimplemented!()
    }
    fn hinge_joint_set_flag(
        &mut self,
        joint: Rid,
        flag: godot::engine::physics_server_3d::HingeJointFlag,
        enabled: bool,
    ) {
        unimplemented!()
    }
    fn hinge_joint_get_flag(
        &self,
        joint: Rid,
        flag: godot::engine::physics_server_3d::HingeJointFlag,
    ) -> bool {
        unimplemented!()
    }
    fn joint_make_slider(
        &mut self,
        joint: Rid,
        body_A: Rid,
        local_ref_A: Transform3D,
        body_B: Rid,
        local_ref_B: Transform3D,
    ) {
        unimplemented!()
    }
    fn slider_joint_set_param(
        &mut self,
        joint: Rid,
        param: godot::engine::physics_server_3d::SliderJointParam,
        value: f32,
    ) {
        unimplemented!()
    }
    fn slider_joint_get_param(
        &self,
        joint: Rid,
        param: godot::engine::physics_server_3d::SliderJointParam,
    ) -> f32 {
        unimplemented!()
    }
    fn joint_make_cone_twist(
        &mut self,
        joint: Rid,
        body_A: Rid,
        local_ref_A: Transform3D,
        body_B: Rid,
        local_ref_B: Transform3D,
    ) {
        unimplemented!()
    }
    fn cone_twist_joint_set_param(
        &mut self,
        joint: Rid,
        param: godot::engine::physics_server_3d::ConeTwistJointParam,
        value: f32,
    ) {
        unimplemented!()
    }
    fn cone_twist_joint_get_param(
        &self,
        joint: Rid,
        param: godot::engine::physics_server_3d::ConeTwistJointParam,
    ) -> f32 {
        unimplemented!()
    }
    fn joint_make_generic_6dof(
        &mut self,
        joint: Rid,
        body_A: Rid,
        local_ref_A: Transform3D,
        body_B: Rid,
        local_ref_B: Transform3D,
    ) {
        unimplemented!()
    }
    fn generic_6dof_joint_set_param(
        &mut self,
        joint: Rid,
        axis: Vector3Axis,
        param: godot::engine::physics_server_3d::G6DOFJointAxisParam,
        value: f32,
    ) {
        unimplemented!()
    }
    fn generic_6dof_joint_get_param(
        &self,
        joint: Rid,
        axis: Vector3Axis,
        param: godot::engine::physics_server_3d::G6DOFJointAxisParam,
    ) -> f32 {
        unimplemented!()
    }
    fn generic_6dof_joint_set_flag(
        &mut self,
        joint: Rid,
        axis: Vector3Axis,
        flag: godot::engine::physics_server_3d::G6DOFJointAxisFlag,
        enable: bool,
    ) {
        unimplemented!()
    }
    fn generic_6dof_joint_get_flag(
        &self,
        joint: Rid,
        axis: Vector3Axis,
        flag: godot::engine::physics_server_3d::G6DOFJointAxisFlag,
    ) -> bool {
        unimplemented!()
    }
    fn joint_get_type(&self, joint: Rid) -> godot::engine::physics_server_3d::JointType {
        unimplemented!()
    }
    fn joint_set_solver_priority(&mut self, joint: Rid, priority: i32) {
        unimplemented!()
    }
    fn joint_get_solver_priority(&self, joint: Rid) -> i32 {
        unimplemented!()
    }
    fn joint_disable_collisions_between_bodies(&mut self, joint: Rid, disable: bool) {
        unimplemented!()
    }
    fn joint_is_disabled_collisions_between_bodies(&self, joint: Rid) -> bool {
        unimplemented!()
    }
    fn free_rid(&mut self, rid: Rid) {
        if let Some(shape) = self.shapes.remove(&rid) {
            shape.borrow_mut().remove_from_owners();
        } else if let Some(body) = self.bodies.remove(&rid) {
            body.borrow_mut().remove_from_space();
        } else if let Some(area) = self.areas.remove(&rid) {
            area.borrow_mut().remove_from_space();
        } else if let Some(space) = self.spaces.remove(&rid) {
            space.borrow_mut().remove_space_from_bodies_areas();
        } else if self.joints.contains_key(&rid) {
            self.joints.remove(&rid);
        } else {
            godot_error!("Failed to free RID: The specified RID has no owner.");
        }
    }
    fn set_active(&mut self, active: bool) {
        self.active = active;
    }
    fn init_ext(&mut self) {}
    fn step(&mut self, step: f32) {
        if !self.active {
            return;
        }

        for space in &self.active_spaces {
            if let Ok(space) = self.get_space(*space) {
                space.borrow_mut().step();
            };
        }
    }
    fn sync(&mut self) {}
    fn flush_queries(&mut self) {
        if !self.active {
            return;
        }

        for space in &self.active_spaces {
            if let Ok(space) = self.get_space(*space) {
                space.borrow_mut().call_queries();
            }
        }
    }
    fn end_sync(&mut self) {}
    fn finish(&mut self) {
        unimplemented!()
    }
    fn is_flushing_queries(&self) -> bool {
        unimplemented!()
    }
    fn get_process_info(
        &mut self,
        process_info: godot::engine::physics_server_3d::ProcessInfo,
    ) -> i32 {
        unimplemented!()
    }
}
