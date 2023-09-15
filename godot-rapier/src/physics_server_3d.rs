#![allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
use std::collections::{HashMap, HashSet};

use godot::engine::native::PhysicsServer3DExtensionMotionResult;
use godot::engine::physics_server_3d::{
    AreaParameter, BodyAxis, BodyMode, BodyParameter, BodyState, SpaceParameter,
};
use godot::engine::{
    PhysicsDirectBodyState3D, PhysicsDirectBodyState3DExtensionVirtual, PhysicsDirectSpaceState3D,
    PhysicsServer3DExtensionVirtual,
};
use godot::prelude::utilities::{rid_allocate_id, rid_from_int64};
use godot::prelude::*;
use rapier3d::prelude::*;

use crate::area::RapierArea;
use crate::body::RapierBody;
use crate::collision_object::RapierCollisionObject;
use crate::shapes::cylinder_shape::CylinderShape;
use crate::shapes::{
    box_shape::BoxShape, capsule_shape::CapsuleShape, concave_polygon_shape::ConcavePolygonShape,
    convex_polygon_shape::ConvexPolygonShape, heightmap_shape::HeightmapShape,
    separation_ray_shape::SeparationRayShape, sphere_shape::SphereShape,
    world_boundary_shape::WorldBoundaryShape, RapierShape,
};
use crate::space::RapierSpace;

#[derive(GodotClass, Default)]
#[class(base=PhysicsServer3DExtension,init)]
pub struct RapierPhysicsServer3D {
    shapes: HashMap<Rid, Box<dyn RapierShape>>,
    spaces: HashMap<Rid, Gd<RapierSpace>>,
    active_spaces: HashSet<Rid>,
    areas: HashMap<Rid, RapierArea>,
    bodies: HashMap<Rid, Gd<RapierBody>>,

    active: bool,
}

#[inline]
fn make_rid() -> Rid {
    rid_from_int64(rid_allocate_id())
}

impl RapierPhysicsServer3D {
    #[allow(clippy::borrowed_box)]
    fn get_shape(&self, rid: Rid) -> Option<&Box<dyn RapierShape>> {
        let shape = self.shapes.get(&rid);
        if shape.is_none() {
            godot_warn!("Could not find shape with {rid}");
        }
        shape
    }
    fn get_shape_mut(&mut self, rid: Rid) -> Option<&mut Box<dyn RapierShape>> {
        let shape = self.shapes.get_mut(&rid);
        if shape.is_none() {
            godot_warn!("Could not find shape with {rid}");
        }
        shape
    }
    fn get_space(&self, rid: Rid) -> Option<&Gd<RapierSpace>> {
        let space = self.spaces.get(&rid);
        if space.is_none() {
            godot_warn!("Could not find space with {rid}");
        }
        space
    }
    fn get_space_mut(&mut self, rid: Rid) -> Option<&mut Gd<RapierSpace>> {
        let space = self.spaces.get_mut(&rid);
        if space.is_none() {
            godot_warn!("Could not find space with {rid}");
        }
        space
    }
    fn get_area(&self, rid: Rid) -> Option<&RapierArea> {
        let area = self.areas.get(&rid);
        if area.is_none() {
            godot_warn!("Could not find area with {rid}");
        }
        area
    }
    fn get_area_mut(&mut self, rid: Rid) -> Option<&mut RapierArea> {
        let area = self.areas.get_mut(&rid);
        if area.is_none() {
            godot_warn!("Could not find area with {rid}");
        }
        area
    }
    fn get_body(&self, rid: Rid) -> Option<&Gd<RapierBody>> {
        let body = self.bodies.get(&rid);
        if body.is_none() {
            godot_warn!("Could not find body with {rid}");
        }
        body
    }
    fn get_body_mut(&mut self, rid: Rid) -> Option<&mut Gd<RapierBody>> {
        let body = self.bodies.get_mut(&rid);
        if body.is_none() {
            godot_warn!("Could not find body with {rid}");
        }
        body
    }

    fn has_space(&self, rid: Rid) -> bool {
        if self.spaces.contains_key(&rid) {
            true
        } else {
            godot_error!("{rid} is not a space RID");
            false
        }
    }
    fn has_shape(&self, rid: Rid) -> bool {
        if self.shapes.contains_key(&rid) {
            true
        } else {
            godot_error!("{rid} is not a shape RID");
            false
        }
    }
    fn has_area(&self, rid: Rid) -> bool {
        if self.areas.contains_key(&rid) {
            true
        } else {
            godot_error!("{rid} is not a area RID");
            false
        }
    }
    fn has_body(&self, rid: Rid) -> bool {
        if self.bodies.contains_key(&rid) {
            true
        } else {
            godot_error!("{rid} is not a body RID");
            false
        }
    }

    fn add_shape_owner(&mut self, shape: Rid, owner: Rid) {
        if let Some(shape) = self.get_shape_mut(shape) {
            shape.add_owner(owner);
        }
    }

    fn remove_shape_owner(&mut self, shape: Rid, owner: Rid) {
        if let Some(shape) = self.get_shape_mut(shape) {
            shape.remove_owner(owner);
        }
    }
}

#[godot_api]
impl PhysicsServer3DExtensionVirtual for RapierPhysicsServer3D {
    fn world_boundary_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        self.shapes
            .insert(rid, Box::new(WorldBoundaryShape::new(rid)));
        godot_print!("WorldBoundaryShape created {rid}");
        rid
    }

    fn separation_ray_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        self.shapes
            .insert(rid, Box::new(SeparationRayShape::new(rid)));
        godot_print!("SeparationRayShape created {rid}");
        rid
    }

    fn sphere_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        self.shapes.insert(rid, Box::new(SphereShape::new(rid)));
        godot_print!("SphereShape created {rid}");
        rid
    }

    fn box_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        self.shapes.insert(rid, Box::new(BoxShape::new(rid)));
        godot_print!("BoxShape created {rid}");
        rid
    }

    fn capsule_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        self.shapes.insert(rid, Box::new(CapsuleShape::new(rid)));
        godot_print!("CapsuleShape created {rid}");
        rid
    }

    fn cylinder_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        self.shapes.insert(rid, Box::new(CylinderShape::new(rid)));
        godot_print!("CylinderShape created {rid}");
        rid
    }

    fn convex_polygon_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        self.shapes
            .insert(rid, Box::new(ConvexPolygonShape::new(rid)));
        godot_print!("ConvexPolygonShape created {rid}");
        rid
    }

    fn concave_polygon_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        self.shapes
            .insert(rid, Box::new(ConcavePolygonShape::new(rid)));
        godot_print!("ConcavePolygonShape created {rid}");
        rid
    }

    fn heightmap_shape_create(&mut self) -> Rid {
        let rid = make_rid();
        self.shapes.insert(rid, Box::new(HeightmapShape::new(rid)));
        godot_print!("HeightMapShape created {rid}");
        rid
    }

    fn custom_shape_create(&mut self) -> Rid {
        godot_warn!("Custom shapes are not supported in Godot Rapier");
        Rid::Invalid
    }

    fn shape_set_data(&mut self, shape: Rid, data: Variant) {
        if let Some(shape) = self.get_shape_mut(shape) {
            shape.set_data(data);
        }
    }

    fn shape_set_custom_solver_bias(&mut self, shape: Rid, bias: f32) {
        godot_warn!("Custom solver bias is unused in Godot Rapier");
    }

    fn shape_set_margin(&mut self, shape: Rid, margin: f32) {
        if let Some(shape) = self.get_shape_mut(shape) {
            shape.set_margin(margin);
        }
    }

    fn shape_get_margin(&self, shape: Rid) -> f32 {
        if let Some(shape) = self.get_shape(shape) {
            return shape.get_margin();
        }
        0.0
    }

    fn shape_get_type(&self, shape: Rid) -> godot::engine::physics_server_3d::ShapeType {
        if let Some(shape) = self.get_shape(shape) {
            return shape.get_shape_type();
        }
        godot::engine::physics_server_3d::ShapeType::SHAPE_CUSTOM
    }

    fn shape_get_data(&self, shape: Rid) -> Variant {
        if let Some(shape) = self.get_shape(shape) {
            return shape.get_data();
        }
        Variant::default()
    }

    fn shape_get_custom_solver_bias(&self, shape: Rid) -> f32 {
        godot_warn!("Custom solver bias is unused in Godot Rapier");
        0.0
    }

    fn space_create(&mut self) -> Rid {
        let rid = make_rid();
        let default_area = RapierArea::new(rid);
        self.areas.insert(rid, default_area);
        let space = RapierSpace::new(rid);
        self.spaces.insert(rid, Gd::new(space));

        godot_print!("Space created {rid}");
        rid
    }

    fn space_set_active(&mut self, space: Rid, active: bool) {
        if !self.has_space(space) {
            return;
        }
        if active {
            self.active_spaces.insert(space);
            godot_print!("Space {space} activated");
        } else {
            self.active_spaces.remove(&space);
            godot_print!("Space {space} deactivated");
        }
    }

    fn space_is_active(&self, space: Rid) -> bool {
        if !self.has_space(space) {
            return false;
        }
        self.active_spaces.contains(&space)
    }

    fn space_set_param(&mut self, space: Rid, param: SpaceParameter, value: f32) {
        if let Some(space) = self.get_space_mut(space) {
            space.bind_mut().set_param(param, value);
        }
    }

    fn space_get_param(&self, space: Rid, param: SpaceParameter) -> f32 {
        if let Some(space) = self.get_space(space) {
            return space.bind().get_param(param);
        }
        0.0
    }

    fn space_get_direct_state(&mut self, space: Rid) -> Option<Gd<PhysicsDirectSpaceState3D>> {
        if let Some(space) = self.get_space(space) {
            return Some(space.share().upcast());
        }
        None
    }

    fn space_set_debug_contacts(&mut self, space: Rid, max_contacts: i32) {
        // TODO
    }

    fn space_get_contacts(&self, space: Rid) -> PackedVector3Array {
        // TODO
        PackedVector3Array::default()
    }

    fn space_get_contact_count(&self, space: Rid) -> i32 {
        // TODO
        0
    }

    fn area_create(&mut self) -> Rid {
        let rid = make_rid();
        self.areas.insert(rid, RapierArea::new(rid));
        godot_print!("Area created {rid}");
        rid
    }

    fn area_set_space(&mut self, area: Rid, space: Rid) {
        if let Some(space) = self.spaces.get_mut(&space) {
            if let Some(area) = self.areas.get_mut(&area) {
                space.bind_mut().add_area(area);
                area.set_space(space.share());
            }
        }
    }

    fn area_get_space(&self, area: Rid) -> Rid {
        if let Some(area) = self.get_area(area) {
            if let Some(space) = area.get_space() {
                return space.bind().rid();
            }
        }
        Rid::Invalid
    }

    fn area_add_shape(
        &mut self,
        area_rid: Rid,
        shape: Rid,
        transform: Transform3D,
        disabled: bool,
    ) {
        if !(self.has_shape(shape) && self.has_area(area_rid)) {
            return;
        }

        if let Some(area) = self.get_area_mut(area_rid) {
            area.add_shape(shape, transform, disabled);
            self.add_shape_owner(shape, area_rid);
        }
    }

    fn area_set_shape(&mut self, area_rid: Rid, shape_idx: i32, shape: Rid) {
        if !(self.has_shape(shape) && self.has_area(area_rid)) {
            return;
        }

        if let Some(area) = self.get_area_mut(area_rid) {
            let old_shape = area.set_shape(shape_idx, shape);
            self.remove_shape_owner(old_shape, area_rid);
            self.add_shape_owner(shape, area_rid);
        }
    }

    fn area_set_shape_transform(&mut self, area: Rid, shape_idx: i32, transform: Transform3D) {
        if !self.has_area(area) {
            return;
        }
        if let Some(area) = self.get_area_mut(area) {
            area.set_shape_transform(shape_idx, transform);
        }
    }

    fn area_set_shape_disabled(&mut self, area: Rid, shape_idx: i32, disabled: bool) {
        if !self.has_area(area) {
            return;
        }
        if let Some(area) = self.get_area_mut(area) {
            area.set_shape_disabled(shape_idx, disabled);
        }
    }

    fn area_get_shape_count(&self, area: Rid) -> i32 {
        if let Some(area) = self.get_area(area) {
            return area.shapes().len() as i32;
        }
        0
    }

    fn area_get_shape(&self, area: Rid, shape_idx: i32) -> Rid {
        if let Some(area) = self.get_area(area) {
            return area.get_shape(shape_idx);
        }
        Rid::Invalid
    }

    fn area_get_shape_transform(&self, area: Rid, shape_idx: i32) -> Transform3D {
        if let Some(area) = self.get_area(area) {
            return area.get_shape_transform(shape_idx);
        }
        Transform3D::default()
    }

    fn area_remove_shape(&mut self, area_rid: Rid, shape_idx: i32) {
        if let Some(area) = self.get_area_mut(area_rid) {
            let old_shape = area.remove_shape(shape_idx);
            self.remove_shape_owner(old_shape, area_rid);
        }
    }

    fn area_clear_shapes(&mut self, area_rid: Rid) {
        if let Some(area) = self.get_area_mut(area_rid) {
            let old_shapes = area.clear_shapes();
            for old_shape in old_shapes {
                self.remove_shape_owner(old_shape, area_rid);
            }
        }
    }

    fn area_attach_object_instance_id(&mut self, area: Rid, id: u64) {
        if let Some(area) = self.get_area_mut(area) {
            area.set_instance_id(id);
        }
    }

    fn area_get_object_instance_id(&self, area: Rid) -> u64 {
        if let Some(area) = self.get_area(area) {
            if let Some(id) = area.get_instance_id() {
                return id;
            }
        }
        0
    }

    fn area_set_param(&mut self, area: Rid, param: AreaParameter, value: Variant) {
        if let Some(area) = self.get_area_mut(area) {
            area.set_param(param, &value);
        }
    }

    fn area_set_transform(&mut self, area: Rid, transform: Transform3D) {
        if let Some(area) = self.get_area_mut(area) {
            area.set_transform(transform);
        }
    }

    fn area_get_param(&self, area: Rid, param: AreaParameter) -> Variant {
        if let Some(area) = self.get_area(area) {
            return area.get_param(param);
        }
        Variant::default()
    }

    fn area_get_transform(&self, area: Rid) -> Transform3D {
        if let Some(area) = self.get_area(area) {
            return area.get_transform();
        }
        Transform3D::default()
    }

    fn area_set_collision_layer(&mut self, area: Rid, layer: u32) {
        if let Some(area) = self.get_area_mut(area) {
            area.set_collision_layer(layer);
        }
    }

    fn area_get_collision_layer(&self, area: Rid) -> u32 {
        if let Some(area) = self.get_area(area) {
            return area.get_collision_layer();
        }
        0
    }

    fn area_set_collision_mask(&mut self, area: Rid, mask: u32) {
        if let Some(area) = self.get_area_mut(area) {
            area.set_collision_mask(mask);
        }
    }

    fn area_get_collision_mask(&self, area: Rid) -> u32 {
        if let Some(area) = self.get_area(area) {
            return area.get_collision_mask();
        }
        0
    }

    fn area_set_monitorable(&mut self, area: Rid, monitorable: bool) {
        if let Some(area) = self.get_area_mut(area) {
            area.set_monitorable(monitorable);
        }
    }

    fn area_set_ray_pickable(&mut self, area: Rid, enable: bool) {
        if let Some(area) = self.get_area_mut(area) {
            area.set_ray_pickable(enable);
        }
    }

    fn area_set_monitor_callback(&mut self, area: Rid, callback: Callable) {
        if let Some(area) = self.get_area_mut(area) {
            area.set_body_monitor_callback(callback);
        }
    }

    fn area_set_area_monitor_callback(&mut self, area: Rid, callback: Callable) {
        if let Some(area) = self.get_area_mut(area) {
            area.set_area_monitor_callback(callback);
        }
    }

    fn body_create(&mut self) -> Rid {
        let rid = make_rid();
        let body = RapierBody::new(rid);
        self.bodies.insert(rid, Gd::new(body));
        godot_print!("Body created {rid}");
        rid
    }

    fn body_set_space(&mut self, body: Rid, space: Rid) {
        if let Some(space) = self.spaces.get_mut(&space) {
            if let Some(body) = self.bodies.get_mut(&body) {
                let handle = space.bind_mut().add_body(body);
                body.bind_mut().set_space(space.share(), handle);
            }
        }
    }

    fn body_get_space(&self, body: Rid) -> Rid {
        if let Some(body) = self.get_body(body) {
            if let Some(space) = body.bind().get_space() {
                return space.bind().rid();
            }
        }
        Rid::Invalid
    }

    fn body_set_mode(&mut self, body: Rid, mode: BodyMode) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().set_body_mode(mode);
        }
    }

    fn body_get_mode(&self, body: Rid) -> BodyMode {
        if let Some(body) = self.get_body(body) {
            return body.bind().get_body_mode();
        }
        BodyMode::BODY_MODE_STATIC
    }

    fn body_add_shape(
        &mut self,
        body_rid: Rid,
        shape: Rid,
        transform: Transform3D,
        disabled: bool,
    ) {
        if !(self.has_shape(shape) && self.has_body(body_rid)) {
            return;
        }
        if let Some(body) = self.get_body_mut(body_rid) {
            body.bind_mut().add_shape(shape, transform, disabled);
            self.add_shape_owner(shape, body_rid);
        }
    }

    fn body_set_shape(&mut self, body_rid: Rid, shape_idx: i32, shape: Rid) {
        if !(self.has_shape(shape) && self.has_body(body_rid)) {
            return;
        }

        if let Some(body) = self.get_body_mut(body_rid) {
            let old_shape = body.bind_mut().set_shape(shape_idx, shape);
            self.remove_shape_owner(old_shape, body_rid);
            self.add_shape_owner(shape, body_rid);
        }
    }

    fn body_set_shape_transform(&mut self, body: Rid, shape_idx: i32, transform: Transform3D) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().set_shape_transform(shape_idx, transform);
        }
    }

    fn body_set_shape_disabled(&mut self, body: Rid, shape_idx: i32, disabled: bool) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().set_shape_disabled(shape_idx, disabled);
        }
    }

    fn body_get_shape_count(&self, body: Rid) -> i32 {
        if let Some(body) = self.get_body(body) {
            return body.bind().shapes().len() as i32;
        }
        0
    }

    fn body_get_shape(&self, body: Rid, shape_idx: i32) -> Rid {
        if let Some(body) = self.get_body(body) {
            return body.bind().get_shape(shape_idx);
        }
        Rid::Invalid
    }

    fn body_get_shape_transform(&self, body: Rid, shape_idx: i32) -> Transform3D {
        if let Some(body) = self.get_body(body) {
            return body.bind().get_shape_transform(shape_idx);
        }
        Transform3D::default()
    }

    fn body_remove_shape(&mut self, body_rid: Rid, shape_idx: i32) {
        if let Some(body) = self.get_body_mut(body_rid) {
            let old_shape = body.bind_mut().remove_shape(shape_idx);
            self.remove_shape_owner(old_shape, body_rid);
        }
    }

    fn body_clear_shapes(&mut self, body_rid: Rid) {
        if let Some(body) = self.get_body_mut(body_rid) {
            let old_shapes = body.bind_mut().clear_shapes();
            for old_shape in old_shapes {
                self.remove_shape_owner(old_shape, body_rid);
            }
        }
    }

    fn body_attach_object_instance_id(&mut self, body: Rid, id: u64) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().set_instance_id(id);
        }
    }

    fn body_get_object_instance_id(&self, body: Rid) -> u64 {
        if let Some(body) = self.get_body(body) {
            if let Some(id) = body.bind().get_instance_id() {
                return id;
            }
        }
        0
    }

    fn body_set_enable_continuous_collision_detection(&mut self, body: Rid, enable: bool) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().set_ccd_enabled(enable);
        }
    }

    fn body_is_continuous_collision_detection_enabled(&self, body: Rid) -> bool {
        if let Some(body) = self.get_body(body) {
            return body.bind().get_ccd_enabled();
        }
        false
    }

    fn body_set_collision_layer(&mut self, body: Rid, layer: u32) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().set_collision_layer(layer);
        }
    }

    fn body_get_collision_layer(&self, body: Rid) -> u32 {
        if let Some(body) = self.get_body(body) {
            return body.bind().get_collision_layer();
        }
        Default::default()
    }

    fn body_set_collision_mask(&mut self, body: Rid, mask: u32) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().set_collision_mask(mask);
        }
    }

    fn body_get_collision_mask(&self, body: Rid) -> u32 {
        if let Some(body) = self.get_body(body) {
            return body.bind().get_collision_mask();
        }
        Default::default()
    }

    fn body_set_collision_priority(&mut self, body: Rid, priority: f32) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().set_priority(priority);
        }
    }

    fn body_get_collision_priority(&self, body: Rid) -> f32 {
        if let Some(body) = self.get_body(body) {
            return body.bind().get_priority();
        }
        Default::default()
    }

    fn body_set_user_flags(&mut self, body: Rid, flags: u32) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().set_user_flags(flags);
        }
    }

    fn body_get_user_flags(&self, body: Rid) -> u32 {
        if let Some(body) = self.get_body(body) {
            return body.bind().get_user_flags();
        }
        Default::default()
    }

    fn body_set_param(&mut self, body: Rid, param: BodyParameter, value: Variant) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().set_param(param, &value);
        }
    }

    fn body_get_param(&self, body: Rid, param: BodyParameter) -> Variant {
        if let Some(body) = self.get_body(body) {
            return body.bind().get_param(param);
        }
        Variant::default()
    }

    fn body_reset_mass_properties(&mut self, body: Rid) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().reset_mass_properties();
        }
    }

    fn body_set_state(&mut self, body: Rid, state: BodyState, value: Variant) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().set_state(state, value);
        }
    }

    fn body_get_state(&self, body: Rid, state: BodyState) -> Variant {
        if let Some(body) = self.get_body(body) {
            return body.bind().get_state(state);
        }
        Variant::default()
    }

    fn body_apply_central_impulse(&mut self, body: Rid, impulse: Vector3) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().apply_central_impulse(impulse);
        }
    }

    fn body_apply_impulse(&mut self, body: Rid, impulse: Vector3, position: Vector3) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().apply_impulse(impulse, position);
        }
    }

    fn body_apply_torque_impulse(&mut self, body: Rid, impulse: Vector3) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().apply_torque_impulse(impulse);
        }
    }

    fn body_apply_central_force(&mut self, body: Rid, force: Vector3) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().apply_central_force(force);
        }
    }

    fn body_apply_force(&mut self, body: Rid, force: Vector3, position: Vector3) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().apply_force(force, position);
        }
    }

    fn body_apply_torque(&mut self, body: Rid, torque: Vector3) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().apply_torque(torque);
        }
    }

    fn body_add_constant_central_force(&mut self, body: Rid, force: Vector3) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().add_constant_central_force(force);
        }
    }

    fn body_add_constant_force(&mut self, body: Rid, force: Vector3, position: Vector3) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().add_constant_force(force, position);
        }
    }

    fn body_add_constant_torque(&mut self, body: Rid, torque: Vector3) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().add_constant_torque(torque);
        }
    }

    fn body_set_constant_force(&mut self, body: Rid, force: Vector3) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().set_constant_force(force);
        }
    }

    fn body_get_constant_force(&self, body: Rid) -> Vector3 {
        if let Some(body) = self.get_body(body) {
            return body.bind().get_constant_force();
        }
        Vector3::default()
    }

    fn body_set_constant_torque(&mut self, body: Rid, torque: Vector3) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().set_constant_torque(torque);
        }
    }

    fn body_get_constant_torque(&self, body: Rid) -> Vector3 {
        if let Some(body) = self.get_body(body) {
            return body.bind().get_constant_torque();
        }
        Vector3::default()
    }

    fn body_set_axis_velocity(&mut self, body: Rid, axis_velocity: Vector3) {}

    fn body_set_axis_lock(&mut self, body: Rid, axis: BodyAxis, lock: bool) {
        if let Some(body) = self.get_body_mut(body) {
            body.bind_mut().set_axis_locked(axis, lock);
        }
    }

    fn body_is_axis_locked(&self, body: Rid, axis: BodyAxis) -> bool {
        if let Some(body) = self.get_body(body) {
            return body.bind().is_axis_locked(axis);
        }
        false
    }

    fn body_add_collision_exception(&mut self, body: Rid, excepted_body: Rid) {}

    fn body_remove_collision_exception(&mut self, body: Rid, excepted_body: Rid) {}

    fn body_get_collision_exceptions(&self, body: Rid) -> Array<Rid> {
        Array::default()
    }

    fn body_set_max_contacts_reported(&mut self, body: Rid, amount: i32) {}

    fn body_get_max_contacts_reported(&self, body: Rid) -> i32 {
        Default::default()
    }

    fn body_set_contacts_reported_depth_threshold(&mut self, body: Rid, threshold: f32) {}

    fn body_get_contacts_reported_depth_threshold(&self, body: Rid) -> f32 {
        Default::default()
    }

    fn body_set_omit_force_integration(&mut self, body: Rid, enable: bool) {}

    fn body_is_omitting_force_integration(&self, body: Rid) -> bool {
        Default::default()
    }

    fn body_set_state_sync_callback(&mut self, body: Rid, callable: Callable) {}

    fn body_set_force_integration_callback(
        &mut self,
        body: Rid,
        callable: Callable,
        userdata: Variant,
    ) {
    }

    fn body_set_ray_pickable(&mut self, body: Rid, enable: bool) {}

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
        Default::default()
    }

    fn body_get_direct_state(&mut self, body: Rid) -> Option<Gd<PhysicsDirectBodyState3D>> {
        if let Some(body) = self.get_body(body) {
            return Some(body.share().upcast());
        }
        None
    }

    fn soft_body_create(&mut self) -> Rid {
        Rid::Invalid
    }

    fn soft_body_update_rendering_server(
        &mut self,
        body: Rid,
        rendering_server_handler: Gd<godot::engine::PhysicsServer3DRenderingServerHandler>,
    ) {
    }

    fn soft_body_set_space(&mut self, body: Rid, space: Rid) {}

    fn soft_body_get_space(&self, body: Rid) -> Rid {
        Rid::Invalid
    }

    fn soft_body_set_ray_pickable(&mut self, body: Rid, enable: bool) {}

    fn soft_body_set_collision_layer(&mut self, body: Rid, layer: u32) {}

    fn soft_body_get_collision_layer(&self, body: Rid) -> u32 {
        Default::default()
    }

    fn soft_body_set_collision_mask(&mut self, body: Rid, mask: u32) {}

    fn soft_body_get_collision_mask(&self, body: Rid) -> u32 {
        Default::default()
    }

    fn soft_body_add_collision_exception(&mut self, body: Rid, body_b: Rid) {}

    fn soft_body_remove_collision_exception(&mut self, body: Rid, body_b: Rid) {}

    fn soft_body_get_collision_exceptions(&self, body: Rid) -> Array<Rid> {
        Array::default()
    }

    fn soft_body_set_state(&mut self, body: Rid, state: BodyState, variant: Variant) {}

    fn soft_body_get_state(&self, body: Rid, state: BodyState) -> Variant {
        Variant::default()
    }

    fn soft_body_set_transform(&mut self, body: Rid, transform: Transform3D) {}

    fn soft_body_set_simulation_precision(&mut self, body: Rid, simulation_precision: i32) {}

    fn soft_body_get_simulation_precision(&self, body: Rid) -> i32 {
        Default::default()
    }

    fn soft_body_set_total_mass(&mut self, body: Rid, total_mass: f32) {}

    fn soft_body_get_total_mass(&self, body: Rid) -> f32 {
        Default::default()
    }

    fn soft_body_set_linear_stiffness(&mut self, body: Rid, linear_stiffness: f32) {}

    fn soft_body_get_linear_stiffness(&self, body: Rid) -> f32 {
        Default::default()
    }

    fn soft_body_set_pressure_coefficient(&mut self, body: Rid, pressure_coefficient: f32) {}

    fn soft_body_get_pressure_coefficient(&self, body: Rid) -> f32 {
        Default::default()
    }

    fn soft_body_set_damping_coefficient(&mut self, body: Rid, damping_coefficient: f32) {}

    fn soft_body_get_damping_coefficient(&self, body: Rid) -> f32 {
        Default::default()
    }

    fn soft_body_set_drag_coefficient(&mut self, body: Rid, drag_coefficient: f32) {}

    fn soft_body_get_drag_coefficient(&self, body: Rid) -> f32 {
        Default::default()
    }

    fn soft_body_set_mesh(&mut self, body: Rid, mesh: Rid) {}

    fn soft_body_get_bounds(&self, body: Rid) -> godot::builtin::Aabb {
        godot::builtin::Aabb::default()
    }

    fn soft_body_move_point(&mut self, body: Rid, point_index: i32, global_position: Vector3) {}

    fn soft_body_get_point_global_position(&self, body: Rid, point_index: i32) -> Vector3 {
        Vector3::default()
    }

    fn soft_body_remove_all_pinned_points(&mut self, body: Rid) {}

    fn soft_body_pin_point(&mut self, body: Rid, point_index: i32, pin: bool) {}

    fn soft_body_is_point_pinned(&self, body: Rid, point_index: i32) -> bool {
        Default::default()
    }

    fn joint_create(&mut self) -> Rid {
        Rid::Invalid
    }

    fn joint_clear(&mut self, joint: Rid) {}

    fn joint_make_pin(
        &mut self,
        joint: Rid,
        body_a: Rid,
        local_a: Vector3,
        body_b: Rid,
        local_b: Vector3,
    ) {
    }

    fn pin_joint_set_param(
        &mut self,
        joint: Rid,
        param: godot::engine::physics_server_3d::PinJointParam,
        value: f32,
    ) {
    }

    fn pin_joint_get_param(
        &self,
        joint: Rid,
        param: godot::engine::physics_server_3d::PinJointParam,
    ) -> f32 {
        Default::default()
    }

    fn pin_joint_set_local_a(&mut self, joint: Rid, local_a: Vector3) {}

    fn pin_joint_get_local_a(&self, joint: Rid) -> Vector3 {
        Vector3::default()
    }

    fn pin_joint_set_local_b(&mut self, joint: Rid, local_b: Vector3) {}

    fn pin_joint_get_local_b(&self, joint: Rid) -> Vector3 {
        Vector3::default()
    }

    fn joint_make_hinge(
        &mut self,
        joint: Rid,
        body_a: Rid,
        hinge_a: Transform3D,
        body_b: Rid,
        hinge_b: Transform3D,
    ) {
    }

    fn joint_make_hinge_simple(
        &mut self,
        joint: Rid,
        body_a: Rid,
        pivot_a: Vector3,
        axis_a: Vector3,
        body_b: Rid,
        pivot_b: Vector3,
        axis_b: Vector3,
    ) {
    }

    fn hinge_joint_set_param(
        &mut self,
        joint: Rid,
        param: godot::engine::physics_server_3d::HingeJointParam,
        value: f32,
    ) {
    }

    fn hinge_joint_get_param(
        &self,
        joint: Rid,
        param: godot::engine::physics_server_3d::HingeJointParam,
    ) -> f32 {
        Default::default()
    }

    fn hinge_joint_set_flag(
        &mut self,
        joint: Rid,
        flag: godot::engine::physics_server_3d::HingeJointFlag,
        enabled: bool,
    ) {
    }

    fn hinge_joint_get_flag(
        &self,
        joint: Rid,
        flag: godot::engine::physics_server_3d::HingeJointFlag,
    ) -> bool {
        Default::default()
    }

    fn joint_make_slider(
        &mut self,
        joint: Rid,
        body_a: Rid,
        local_ref_a: Transform3D,
        body_b: Rid,
        local_ref_b: Transform3D,
    ) {
    }

    fn slider_joint_set_param(
        &mut self,
        joint: Rid,
        param: godot::engine::physics_server_3d::SliderJointParam,
        value: f32,
    ) {
    }

    fn slider_joint_get_param(
        &self,
        joint: Rid,
        param: godot::engine::physics_server_3d::SliderJointParam,
    ) -> f32 {
        Default::default()
    }

    fn joint_make_cone_twist(
        &mut self,
        joint: Rid,
        body_a: Rid,
        local_ref_a: Transform3D,
        body_b: Rid,
        local_ref_b: Transform3D,
    ) {
    }

    fn cone_twist_joint_set_param(
        &mut self,
        joint: Rid,
        param: godot::engine::physics_server_3d::ConeTwistJointParam,
        value: f32,
    ) {
    }

    fn cone_twist_joint_get_param(
        &self,
        joint: Rid,
        param: godot::engine::physics_server_3d::ConeTwistJointParam,
    ) -> f32 {
        Default::default()
    }

    fn joint_make_generic_6dof(
        &mut self,
        joint: Rid,
        body_a: Rid,
        local_ref_a: Transform3D,
        body_b: Rid,
        local_ref_b: Transform3D,
    ) {
    }

    fn generic_6dof_joint_set_param(
        &mut self,
        joint: Rid,
        axis: Vector3Axis,
        param: godot::engine::physics_server_3d::G6DOFJointAxisParam,
        value: f32,
    ) {
    }

    fn generic_6dof_joint_get_param(
        &self,
        joint: Rid,
        axis: Vector3Axis,
        param: godot::engine::physics_server_3d::G6DOFJointAxisParam,
    ) -> f32 {
        Default::default()
    }

    fn generic_6dof_joint_set_flag(
        &mut self,
        joint: Rid,
        axis: Vector3Axis,
        flag: godot::engine::physics_server_3d::G6DOFJointAxisFlag,
        enable: bool,
    ) {
    }

    fn generic_6dof_joint_get_flag(
        &self,
        joint: Rid,
        axis: Vector3Axis,
        flag: godot::engine::physics_server_3d::G6DOFJointAxisFlag,
    ) -> bool {
        Default::default()
    }

    fn joint_get_type(&self, joint: Rid) -> godot::engine::physics_server_3d::JointType {
        godot::engine::physics_server_3d::JointType::JOINT_TYPE_PIN
    }

    fn joint_set_solver_priority(&mut self, joint: Rid, priority: i32) {}

    fn joint_get_solver_priority(&self, joint: Rid) -> i32 {
        Default::default()
    }

    fn joint_disable_collisions_between_bodies(&mut self, joint: Rid, disable: bool) {}

    fn joint_is_disabled_collisions_between_bodies(&self, joint: Rid) -> bool {
        Default::default()
    }

    fn free_rid(&mut self, rid: Rid) {
        if let Some(shape) = self.shapes.remove(&rid) {
            for body in self.bodies.values_mut() {
                body.bind_mut().remove_shape_rid(rid);
            }
            for area in self.areas.values_mut() {
                area.remove_shape_rid(rid);
            }
        } else if let Some(body) = self.bodies.remove(&rid) {
            body.free();
        } else if let Some(area) = self.areas.remove(&rid) {
        } else if let Some(space) = self.spaces.remove(&rid) {
            space.free();
            self.active_spaces.remove(&rid);
        } else {
            godot_error!("Failed to free RID: The specified {} has no owner.", rid);
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
            if let Some(space) = self.spaces.get_mut(space) {
                space.bind_mut().step();
            }
        }
    }

    fn sync(&mut self) {}

    fn flush_queries(&mut self) {}

    fn end_sync(&mut self) {}

    fn finish(&mut self) {}

    fn is_flushing_queries(&self) -> bool {
        Default::default()
    }

    fn get_process_info(
        &mut self,
        process_info: godot::engine::physics_server_3d::ProcessInfo,
    ) -> i32 {
        Default::default()
    }
}
