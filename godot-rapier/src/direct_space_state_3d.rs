#![allow(unused, non_snake_case)]
use std::ffi::c_void;

use godot::engine::native::{
    PhysicsServer3DExtensionRayResult, PhysicsServer3DExtensionShapeRestInfo,
    PhysicsServer3DExtensionShapeResult,
};
use godot::engine::PhysicsDirectSpaceState3DExtensionVirtual;
use godot::prelude::*;

#[derive(GodotClass)]
#[class(base=PhysicsDirectSpaceState3DExtension)]
struct RapierPhysicsDirectSpaceState3D {}
#[godot_api]
impl PhysicsDirectSpaceState3DExtensionVirtual for RapierPhysicsDirectSpaceState3D {
    #[doc = "# Safety"]
    #[doc = ""]
    #[doc = "Godot currently does not document safety requirements on this method. Make sure you understand the underlying semantics."]
    unsafe fn intersect_ray(
        &mut self,
        from: Vector3,
        to: Vector3,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        hit_from_inside: bool,
        hit_back_faces: bool,
        pick_ray: bool,
        result: *mut PhysicsServer3DExtensionRayResult,
    ) -> bool {
        false
    }
    #[doc = "# Safety"]
    #[doc = ""]
    #[doc = "Godot currently does not document safety requirements on this method. Make sure you understand the underlying semantics."]
    unsafe fn intersect_point(
        &mut self,
        position: Vector3,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut PhysicsServer3DExtensionShapeResult,
        max_results: i32,
    ) -> i32 {
        0
    }
    #[doc = "# Safety"]
    #[doc = ""]
    #[doc = "Godot currently does not document safety requirements on this method. Make sure you understand the underlying semantics."]
    unsafe fn intersect_shape(
        &mut self,
        shape_rid: Rid,
        transform: Transform3D,
        motion: Vector3,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        result_count: *mut PhysicsServer3DExtensionShapeResult,
        max_results: i32,
    ) -> i32 {
        0
    }
    #[doc = "# Safety"]
    #[doc = ""]
    #[doc = "Godot currently does not document safety requirements on this method. Make sure you understand the underlying semantics."]
    unsafe fn cast_motion(
        &mut self,
        shape_rid: Rid,
        transform: Transform3D,
        motion: Vector3,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        closest_safe: *mut f64,
        closest_unsafe: *mut f64,
        info: *mut PhysicsServer3DExtensionShapeRestInfo,
    ) -> bool {
        false
    }
    #[doc = "# Safety"]
    #[doc = ""]
    #[doc = "Godot currently does not document safety requirements on this method. Make sure you understand the underlying semantics."]
    unsafe fn collide_shape(
        &mut self,
        shape_rid: Rid,
        transform: Transform3D,
        motion: Vector3,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        results: *mut c_void,
        max_results: i32,
        result_count: *mut i32,
    ) -> bool {
        false
    }
    #[doc = "# Safety"]
    #[doc = ""]
    #[doc = "Godot currently does not document safety requirements on this method. Make sure you understand the underlying semantics."]
    unsafe fn rest_info(
        &mut self,
        shape_rid: Rid,
        transform: Transform3D,
        motion: Vector3,
        margin: f32,
        collision_mask: u32,
        collide_with_bodies: bool,
        collide_with_areas: bool,
        rest_info: *mut PhysicsServer3DExtensionShapeRestInfo,
    ) -> bool {
        false
    }
    fn get_closest_point_to_object_volume(&self, object: Rid, point: Vector3) -> Vector3 {
        Vector3::ZERO
    }
}
