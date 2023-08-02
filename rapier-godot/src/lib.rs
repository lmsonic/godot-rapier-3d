/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

use godot::{
    engine::PhysicsServer3DManager, prelude::*, private::class_macros::auto_register_classes,
};
use physics_server_3d::RapierPhysicsServer3D;

struct RapierPhysics;

mod area;
mod body;
mod collision_object;
mod conversions;
mod direct_body_state_3d;
mod direct_space_state_3d;
mod joint;
mod physics_server_3d;
mod shape;
mod space;

use thiserror::Error;

type RapierResult<T> = Result<T, RapierError>;

#[derive(Error, Debug)]
enum RapierError {
    #[error("RID {0} doesn't correspond to any shape")]
    ShapeRidMissing(Rid),
    #[error("RID {0} doesn't correspond to any area")]
    AreaRidMissing(Rid),
    #[error("RID {0} doesn't correspond to any body")]
    BodyRidMissing(Rid),
    #[error("RID {0} doesn't correspond to any space")]
    SpaceRidMissing(Rid),
    #[error("RID {0} doesn't correspond to any joint")]
    JointRidMissing(Rid),
    #[error("Object with RID {0} doesn't have any space set")]
    ObjectSpaceNotSet(Rid),
    #[error("Area with RID {0} doesn't have any rapier collider handle set")]
    AreaHandleNotSet(Rid),
    #[error("Area with RID {0} has invalid collider handle")]
    AreaHandleInvalid(Rid),
    #[error("Body with RID {0} doesn't have any rapier rigid body handle set")]
    BodyHandleNotSet(Rid),
    #[error("Body with RID {0} has invalid rigid body handle")]
    BodyHandleInvalid(Rid),
    #[error("Area with RID {0} doesn't have any instance ID set")]
    AreaInstanceIDNotSet(Rid),
    #[error("Body with RID {0} doesn't have any instance ID set")]
    BodyInstanceIDNotSet(Rid),
    #[error("Shape with index {0} isn't present in object with RID {1}")]
    ShapeNotInObject(usize, Rid),
    #[error(
        "Object with RID {0} is being build without shape (will use a disabled sphere as a stub)"
    )]
    BuildingObjectWithNoShapes(Rid),
}
#[derive(GodotClass)]
#[class(base=Object,init)]
pub struct ServerInitializer {}

#[godot_api]
impl ServerInitializer {
    #[func]
    fn create_server() -> Gd<RapierPhysicsServer3D> {
        Gd::<RapierPhysicsServer3D>::new_default()
    }
}

struct ServerLayer;
impl ExtensionLayer for ServerLayer {
    fn initialize(&mut self) {
        crate::auto_register_classes();
        let mut manager = PhysicsServer3DManager::singleton();
        let initializer = Gd::<ServerInitializer>::new_default();
        manager.register_server("Rapier3D".into(), initializer.callable("create_server"));
    }

    fn deinitialize(&mut self) {}
}

#[gdextension]
unsafe impl ExtensionLibrary for RapierPhysics {
    fn load_library(handle: &mut InitHandle) -> bool {
        handle.register_layer(InitLevel::Servers, ServerLayer);
        true
    }
}
