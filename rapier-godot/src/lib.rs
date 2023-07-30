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

mod direct_body_state_3d;
mod direct_space_state_3d;
mod physics_server_3d;
mod shape;

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
