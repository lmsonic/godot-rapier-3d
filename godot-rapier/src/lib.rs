#![allow(clippy::module_name_repetitions)]
use godot::{
    engine::PhysicsServer3DManager, init::EditorRunBehavior, prelude::*,
    private::class_macros::auto_register_classes,
};
use physics_server_3d::RapierPhysicsServer3D;

struct RapierPhysics;

mod area;
mod body;
mod collision_object;
mod conversions;
mod direct_body_state_3d;
mod direct_space_state_3d;
mod error;
mod joint;
mod physics_server_3d;
mod physics_server_3d_utils;
mod shapes;
mod space;

#[derive(GodotClass)]
#[class(base=Object,init)]
pub struct ServerInitializer;

#[godot_api]
impl ServerInitializer {
    #[func]
    fn create_server() -> Gd<RapierPhysicsServer3D> {
        Gd::<RapierPhysicsServer3D>::new_default()
    }
}

struct ServerLayer {
    initializer: Option<Gd<ServerInitializer>>,
}

impl ServerLayer {
    const fn new() -> Self {
        Self { initializer: None }
    }
}

impl ExtensionLayer for ServerLayer {
    fn initialize(&mut self) {
        crate::auto_register_classes();
        let mut manager = PhysicsServer3DManager::singleton();
        let initializer = Gd::<ServerInitializer>::new_default();
        manager.register_server("Rapier3D".into(), initializer.callable("create_server"));
        self.initializer = Some(initializer);
    }

    fn deinitialize(&mut self) {
        if let Some(initializer) = self.initializer.take() {
            initializer.free();
        }
    }
}

#[gdextension]
unsafe impl ExtensionLibrary for RapierPhysics {
    fn load_library(handle: &mut InitHandle) -> bool {
        handle.register_layer(InitLevel::Servers, ServerLayer::new());
        true
    }
    fn editor_run_behavior() -> EditorRunBehavior {
        EditorRunBehavior::AllClasses
    }
}
