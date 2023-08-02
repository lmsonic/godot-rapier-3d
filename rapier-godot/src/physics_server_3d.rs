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
use crate::shape::{
    RapierBoxShape, RapierCapsuleShape, RapierCylinderShape, RapierShape, RapierSphereShape,
    SeparationRayShape, WorldBoundaryShape,
};
use crate::space::RapierSpace;
use crate::RapierError;

#[derive(GodotClass, Default)]
#[class(base=PhysicsServer3DExtension,init)]
pub struct RapierPhysicsServer3D {
    shapes: HashMap<Rid, Rc<RefCell<dyn RapierShape>>>,
    spaces: HashMap<Rid, Rc<RefCell<RapierSpace>>>,
    active_spaces: HashSet<Rid>,
    areas: HashMap<Rid, Rc<RefCell<RapierArea>>>,
    bodies: HashMap<Rid, Rc<RefCell<RapierBody>>>,
    active: bool,
}

#[inline]
fn make_rid() -> Rid {
    rid_from_int64(rid_allocate_id())
}

#[godot_api]
impl PhysicsServer3DExtensionVirtual for RapierPhysicsServer3D {
    fn world_boundary_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        let shape = WorldBoundaryShape::new(rid);
        self.shapes.insert(rid, Rc::new(RefCell::new(shape)));
        rid
    }
    fn separation_ray_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        let shape = SeparationRayShape::new(rid);
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
        unimplemented!()
    }
    fn concave_polygon_shape_create(&mut self) -> Rid {
        unimplemented!()
    }
    fn heightmap_shape_create(&mut self) -> Rid {
        unimplemented!()
    }
    fn custom_shape_create(&mut self) -> Rid {
        Rid::Invalid
    }

    fn shape_set_data(&mut self, shape: Rid, data: Variant) {
        if let Some(shape) = self.shapes.get_mut(&shape) {
            shape.borrow_mut().set_data(data);
        } else {
            godot_error!("{}", RapierError::ShapeRidMissing(shape));
        }
    }
    fn shape_set_custom_solver_bias(&mut self, shape: Rid, bias: f32) {
        unimplemented!()
    }
    fn shape_set_margin(&mut self, shape: Rid, margin: f32) {
        unimplemented!()
    }
    fn shape_get_margin(&self, shape_rid: Rid) -> f32 {
        unimplemented!()
    }
    fn shape_get_type(&self, shape: Rid) -> godot::engine::physics_server_3d::ShapeType {
        if let Some(shape) = self.shapes.get(&shape) {
            return shape.borrow().get_type();
        }
        godot_error!("{}", RapierError::ShapeRidMissing(shape));
        godot::engine::physics_server_3d::ShapeType::SHAPE_CUSTOM
    }
    fn shape_get_data(&self, shape: Rid) -> Variant {
        if let Some(shape) = self.shapes.get(&shape) {
            return shape.borrow().get_data();
        }
        godot_error!("{}", RapierError::ShapeRidMissing(shape));
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
        if self.spaces.contains_key(&space) {
            if active {
                self.active_spaces.insert(space);
            } else {
                self.active_spaces.remove(&space);
            }
        } else {
            godot_error!("{}", RapierError::SpaceRidMissing(space));
        }
    }
    fn space_is_active(&self, space: Rid) -> bool {
        if self.spaces.contains_key(&space) {
            return self.active_spaces.contains(&space);
        }
        godot_error!("{}", RapierError::SpaceRidMissing(space));
        false
    }
    fn space_set_param(&mut self, space: Rid, param: SpaceParameter, value: f32) {
        if let Some(space) = self.spaces.get_mut(&space) {
            space.borrow_mut().set_param(param, value);
        } else {
            godot_error!("{}", RapierError::SpaceRidMissing(space));
        }
    }
    fn space_get_param(&self, space: Rid, param: SpaceParameter) -> f32 {
        if let Some(space) = self.spaces.get(&space) {
            space.borrow().get_param(param);
        }
        godot_error!("{}", RapierError::SpaceRidMissing(space));
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
        if let Some(area) = self.areas.get_mut(&area) {
            if let Some(space) = self.spaces.get(&space) {
                space.borrow_mut().add_area(area);
                area.borrow_mut().set_space(space.clone());
            } else {
                godot_error!("{}", RapierError::SpaceRidMissing(space));
            }
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area));
        }
    }
    fn area_get_space(&self, area_id: Rid) -> Rid {
        if let Some(area) = self.areas.get(&area_id) {
            if let Some(space) = area.borrow().get_space() {
                let rid = space.borrow().rid();
                if self.spaces.contains_key(&rid) {
                    return rid;
                }
                godot_error!("{}", RapierError::SpaceRidMissing(rid));
            } else {
                godot_error!("{}", RapierError::ObjectSpaceNotSet(area_id));
            }
        }
        godot_error!("{}", RapierError::AreaRidMissing(area_id));
        Rid::Invalid
    }
    fn area_add_shape(&mut self, area: Rid, shape: Rid, transform: Transform3D, disabled: bool) {
        if let Some(area) = self.areas.get_mut(&area) {
            if let Some(shape) = self.shapes.get(&shape) {
                area.borrow_mut()
                    .add_shape(shape.clone(), transform, disabled);
            } else {
                godot_error!("{}", RapierError::ShapeRidMissing(shape));
            }
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area));
        }
    }
    fn area_set_shape(&mut self, area: Rid, shape_idx: i32, shape: Rid) {
        if let Some(area) = self.areas.get_mut(&area) {
            if let Some(shape) = self.shapes.get(&shape) {
                area.borrow_mut()
                    .set_shape(shape_idx as usize, shape.clone());
            } else {
                godot_error!("{}", RapierError::ShapeRidMissing(shape));
            }
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area));
        }
    }
    fn area_set_shape_transform(&mut self, area: Rid, shape_idx: i32, transform: Transform3D) {
        if let Some(area) = self.areas.get_mut(&area) {
            area.borrow_mut()
                .set_shape_transform(shape_idx as usize, transform);
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area));
        }
    }
    fn area_set_shape_disabled(&mut self, area: Rid, shape_idx: i32, disabled: bool) {
        if let Some(area) = self.areas.get_mut(&area) {
            area.borrow_mut()
                .set_shape_disabled(shape_idx as usize, disabled);
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area));
        }
    }
    fn area_get_shape_count(&self, area: Rid) -> i32 {
        if let Some(area) = self.areas.get(&area) {
            return area.borrow().shapes().len() as i32;
        }
        godot_error!("{}", RapierError::AreaRidMissing(area));
        0
    }
    fn area_get_shape(&self, area_id: Rid, shape_idx: i32) -> Rid {
        if let Some(area) = self.areas.get(&area_id) {
            if let Some(shape_inst) = area.borrow().shapes().get(shape_idx as usize) {
                return shape_inst.shape.borrow().rid();
            }
            godot_error!(
                "{}",
                RapierError::ShapeNotInObject(shape_idx as usize, area_id)
            );
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area_id));
        }
        Rid::Invalid
    }
    fn area_get_shape_transform(&self, area_id: Rid, shape_idx: i32) -> Transform3D {
        if let Some(area) = self.areas.get(&area_id) {
            if let Some(shape_inst) = area.borrow().shapes().get(shape_idx as usize) {
                return isometry_to_transform(&shape_inst.isometry);
            }
            godot_error!(
                "{}",
                RapierError::ShapeNotInObject(shape_idx as usize, area_id)
            );
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area_id));
        }
        Transform3D::default()
    }
    fn area_remove_shape(&mut self, area: Rid, shape_idx: i32) {
        if let Some(area) = self.areas.get_mut(&area) {
            area.borrow_mut().remove_nth_shape(shape_idx as usize);
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area));
        }
    }
    fn area_clear_shapes(&mut self, area: Rid) {
        if let Some(area) = self.areas.get_mut(&area) {
            area.borrow_mut().clear_shapes();
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area));
        }
    }
    fn area_attach_object_instance_id(&mut self, area: Rid, id: u64) {
        if let Some(area) = self.areas.get_mut(&area) {
            area.borrow_mut().set_instance_id(id);
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area));
        }
    }
    fn area_get_object_instance_id(&self, area_id: Rid) -> u64 {
        if let Some(area) = self.areas.get(&area_id) {
            if let Some(id) = area.borrow_mut().get_instance_id() {
                return id;
            }
            godot_error!("{}", RapierError::AreaInstanceIDNotSet(area_id));
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area_id));
        }
        0
    }
    fn area_set_param(
        &mut self,
        area: Rid,
        param: godot::engine::physics_server_3d::AreaParameter,
        value: Variant,
    ) {
        if let Some(area) = self.areas.get(&area) {
            area.borrow_mut().set_param(param, value);
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area));
        }
    }
    fn area_set_transform(&mut self, area: Rid, transform: Transform3D) {
        if let Some(area) = self.areas.get(&area) {
            area.borrow_mut().set_transform(transform);
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area));
        }
    }
    fn area_get_param(
        &self,
        area: Rid,
        param: godot::engine::physics_server_3d::AreaParameter,
    ) -> Variant {
        if let Some(area) = self.areas.get(&area) {
            return area.borrow_mut().get_param(param);
        }
        godot_error!("{}", RapierError::AreaRidMissing(area));
        Variant::nil()
    }
    fn area_get_transform(&self, area: Rid) -> Transform3D {
        if let Some(area) = self.areas.get(&area) {
            if let Some(transform) = area.borrow_mut().get_transform() {
                return transform;
            }
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area));
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
        if let Some(area) = self.areas.get_mut(&area) {
            area.borrow_mut().set_monitorable(monitorable);
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area));
        }
    }
    fn area_set_ray_pickable(&mut self, area: Rid, enable: bool) {
        unimplemented!()
    }
    fn area_set_monitor_callback(&mut self, area: Rid, callback: Callable) {
        if let Some(area) = self.areas.get_mut(&area) {
            area.borrow_mut().set_body_monitor_callback(callback);
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area));
        }
    }
    fn area_set_area_monitor_callback(&mut self, area: Rid, callback: Callable) {
        if let Some(area) = self.areas.get_mut(&area) {
            area.borrow_mut().set_area_monitor_callback(callback);
        } else {
            godot_error!("{}", RapierError::AreaRidMissing(area));
        }
    }
    fn body_create(&mut self) -> Rid {
        let rid = make_rid();
        let body = RapierBody::new(rid);
        self.bodies.insert(rid, Rc::new(RefCell::new(body)));
        rid
    }

    fn body_set_space(&mut self, body: Rid, space: Rid) {
        if let Some(body) = self.bodies.get_mut(&body) {
            if let Some(space) = self.spaces.get(&space) {
                body.borrow_mut().set_space(space.clone());
                space.borrow_mut().add_body(body);
            } else {
                godot_error!("{}", RapierError::SpaceRidMissing(space));
            }
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_get_space(&self, body_id: Rid) -> Rid {
        if let Some(body) = self.bodies.get(&body_id) {
            if let Some(space) = body.borrow().get_space() {
                let rid = space.borrow().rid();
                if self.spaces.contains_key(&rid) {
                    return rid;
                }
                godot_error!("{}", RapierError::SpaceRidMissing(rid));
            } else {
                godot_error!("{}", RapierError::ObjectSpaceNotSet(body_id));
            }
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body_id));
        }
        Rid::Invalid
    }
    fn body_set_mode(&mut self, body: Rid, mode: BodyMode) {
        if let Some(body) = self.bodies.get(&body) {
            body.borrow_mut().set_body_mode(mode);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_get_mode(&self, body: Rid) -> BodyMode {
        if let Some(body) = self.bodies.get(&body) {
            return body.borrow().get_body_mode();
        }
        godot_error!("{}", RapierError::BodyRidMissing(body));
        BodyMode::BODY_MODE_STATIC
    }
    fn body_add_shape(&mut self, body: Rid, shape: Rid, transform: Transform3D, disabled: bool) {
        if let Some(body) = self.bodies.get_mut(&body) {
            if let Some(shape) = self.shapes.get(&shape) {
                body.borrow_mut()
                    .add_shape(shape.clone(), transform, disabled);
            } else {
                godot_error!("{}", RapierError::ShapeRidMissing(shape));
            }
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_set_shape(&mut self, body: Rid, shape_idx: i32, shape: Rid) {
        if let Some(body) = self.bodies.get_mut(&body) {
            if let Some(shape) = self.shapes.get(&shape) {
                body.borrow_mut()
                    .set_shape(shape_idx as usize, shape.clone());
            } else {
                godot_error!("{}", RapierError::ShapeRidMissing(shape));
            }
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_set_shape_transform(&mut self, body: Rid, shape_idx: i32, transform: Transform3D) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut()
                .set_shape_transform(shape_idx as usize, transform);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_set_shape_disabled(&mut self, body: Rid, shape_idx: i32, disabled: bool) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut()
                .set_shape_disabled(shape_idx as usize, disabled);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_get_shape_count(&self, body: Rid) -> i32 {
        if let Some(body) = self.bodies.get(&body) {
            return body.borrow().shapes().len() as i32;
        }
        godot_error!("{}", RapierError::BodyRidMissing(body));
        0
    }
    fn body_get_shape(&self, body_id: Rid, shape_idx: i32) -> Rid {
        if let Some(body) = self.bodies.get(&body_id) {
            if let Some(shape_inst) = body.borrow().shapes().get(shape_idx as usize) {
                return shape_inst.shape.borrow().rid();
            }
            godot_error!(
                "{}",
                RapierError::ShapeNotInObject(shape_idx as usize, body_id)
            );
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body_id));
        }
        Rid::Invalid
    }
    fn body_get_shape_transform(&self, body_id: Rid, shape_idx: i32) -> Transform3D {
        if let Some(body) = self.bodies.get(&body_id) {
            if let Some(shape_inst) = body.borrow().shapes().get(shape_idx as usize) {
                return isometry_to_transform(&shape_inst.isometry);
            }
            godot_error!(
                "{}",
                RapierError::ShapeNotInObject(shape_idx as usize, body_id)
            );
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body_id));
        }
        Transform3D::default()
    }
    fn body_remove_shape(&mut self, body: Rid, shape_idx: i32) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().remove_nth_shape(shape_idx as usize);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_clear_shapes(&mut self, body: Rid) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().clear_shapes();
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_attach_object_instance_id(&mut self, body: Rid, id: u64) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().set_instance_id(id);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_get_object_instance_id(&self, body_id: Rid) -> u64 {
        if let Some(body) = self.bodies.get(&body_id) {
            if let Some(id) = body.borrow_mut().get_instance_id() {
                return id;
            }
            godot_error!("{}", RapierError::BodyInstanceIDNotSet(body_id));
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body_id));
        }
        0
    }
    fn body_set_enable_continuous_collision_detection(&mut self, body: Rid, enable: bool) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().set_enable_ccd(enable);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_is_continuous_collision_detection_enabled(&self, body: Rid) -> bool {
        if let Some(body) = self.bodies.get(&body) {
            return body.borrow_mut().is_ccd_enabled();
        }
        godot_error!("{}", RapierError::BodyRidMissing(body));
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
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().apply_central_impulse(impulse);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_apply_impulse(&mut self, body: Rid, impulse: Vector3, position: Vector3) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().apply_impulse(impulse, position);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_apply_torque_impulse(&mut self, body: Rid, impulse: Vector3) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().apply_torque_impulse(impulse);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_apply_central_force(&mut self, body: Rid, force: Vector3) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().apply_central_force(force);
        }
    }
    fn body_apply_force(&mut self, body: Rid, force: Vector3, position: Vector3) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().apply_force(force, position);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_apply_torque(&mut self, body: Rid, torque: Vector3) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().apply_torque(torque);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_add_constant_central_force(&mut self, body: Rid, force: Vector3) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().add_constant_central_force(force);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_add_constant_force(&mut self, body: Rid, force: Vector3, position: Vector3) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().add_constant_force(force, position);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_add_constant_torque(&mut self, body: Rid, torque: Vector3) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().add_constant_torque(torque);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_set_constant_force(&mut self, body: Rid, force: Vector3) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().set_constant_force(force);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_get_constant_force(&self, body: Rid) -> Vector3 {
        if let Some(body) = self.bodies.get(&body) {
            return body.borrow_mut().get_constant_force_godot();
        }
        godot_error!("{}", RapierError::BodyRidMissing(body));
        Vector3::ZERO
    }
    fn body_set_constant_torque(&mut self, body: Rid, torque: Vector3) {
        if let Some(body) = self.bodies.get_mut(&body) {
            body.borrow_mut().set_constant_torque(torque);
        } else {
            godot_error!("{}", RapierError::BodyRidMissing(body));
        }
    }
    fn body_get_constant_torque(&self, body: Rid) -> Vector3 {
        if let Some(body) = self.bodies.get(&body) {
            return body.borrow_mut().get_constant_torque_godot();
        }
        godot_error!("{}", RapierError::BodyRidMissing(body));
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
        if let Some(body) = self.bodies.get_mut(&body) {
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
        unimplemented!()
    }
    fn joint_clear(&mut self, joint: Rid) {
        unimplemented!()
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
            shape.borrow().remove_from_owners();
            self.shapes.remove(&rid);
        } else if self.bodies.contains_key(&rid) {
            self.bodies.remove(&rid);
        } else if self.areas.contains_key(&rid) {
            self.areas.remove(&rid);
        } else if self.spaces.contains_key(&rid) {
            self.spaces.remove(&rid);
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
            if let Some(space) = self.spaces.get(space) {
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
            if let Some(space) = self.spaces.get(space) {
                space.borrow_mut().call_queries();
            } else {
                godot_error!("{}", RapierError::SpaceRidMissing(*space));
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
