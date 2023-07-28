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

#[derive(GodotClass)]
#[class(base=Object,init)]
pub struct RapierPhysicsServerFactory {}

#[godot_api]
impl RapierPhysicsServerFactory {
    fn create_server() -> Gd<RapierPhysicsServer3D> {
        Gd::new_default()
    }
}

impl ServerLayer {
    fn new() -> Self {
        Self {
            server_factory: Gd::<RapierPhysicsServerFactory>::new_default(),
        }
    }
}

struct ServerLayer {
    server_factory: Gd<RapierPhysicsServerFactory>,
}
impl ExtensionLayer for ServerLayer {
    fn initialize(&mut self) {
        let mut manager = PhysicsServer3DManager::singleton();
        manager.register_server(
            "Rapier3D".into(),
            self.server_factory.callable("create_server"),
        );
    }

    fn deinitialize(&mut self) {}
}

struct DefaultLayer;

impl ExtensionLayer for DefaultLayer {
    fn initialize(&mut self) {
        crate::auto_register_classes();
    }

    fn deinitialize(&mut self) {
        // Nothing -- note that any cleanup task should be performed outside of this method,
        // as the user is free to use a different impl, so cleanup code may not be run.
    }
}

#[gdextension]
unsafe impl ExtensionLibrary for RapierPhysics {
    fn load_library(handle: &mut InitHandle) -> bool {
        handle.register_layer(InitLevel::Scene, DefaultLayer);
        handle.register_layer(InitLevel::Servers, ServerLayer::new());
        true
    }
}
