#![warn(
    clippy::pedantic,
    clippy::nursery,
    clippy::dbg_macro,
    clippy::empty_structs_with_brackets,
    clippy::float_cmp_const,
    clippy::print_stderr,
    clippy::print_stdout,
    clippy::shadow_unrelated,
    clippy::use_debug,
    clippy::wildcard_dependencies,
    clippy::exit,
    clippy::todo,
    clippy::mem_forget,
    clippy::rc_mutex,
    clippy::rest_pat_in_fully_bound_structs,
    clippy::string_add,
    clippy::string_to_string,
    clippy::unimplemented,
    clippy::verbose_file_reads,
    future_incompatible,
    nonstandard_style,
    rust_2018_idioms,
    unused_crate_dependencies,
    unused_extern_crates,
    unused_import_braces,
    missing_copy_implementations,
    missing_debug_implementations,
    non_ascii_idents,
    noop_method_call,
    //missing_docs,
    //clippy::missing_docs_in_private_items
)]
#![allow(clippy::module_name_repetitions)]

use godot::{
    engine::PhysicsServer3DManager, init::EditorRunBehavior, prelude::*,
    private::class_macros::auto_register_classes,
};
use physics_server_3d::RapierPhysicsServer3D;

mod physics_server_3d;

mod area;
mod body;
mod collision_object;
mod shapes;
mod space;

#[derive(GodotClass)]
#[class(base=Object,init)]
struct ServerInitializer;

#[godot_api]
impl ServerInitializer {
    #[func]
    fn create_server() -> Gd<RapierPhysicsServer3D> {
        Gd::<RapierPhysicsServer3D>::new_default()
    }
}
struct RapierPhysics;
#[gdextension]
unsafe impl ExtensionLibrary for RapierPhysics {
    fn editor_run_behavior() -> EditorRunBehavior {
        EditorRunBehavior::AllClasses
    }

    fn min_level() -> InitLevel {
        InitLevel::Servers
    }

    fn on_level_init(level: InitLevel) {
        if level == InitLevel::Servers {
            let mut manager = PhysicsServer3DManager::singleton();
            let initializer = Gd::<ServerInitializer>::new_default();
            manager.register_server("Rapier3D".into(), initializer.callable("create_server"));
        }
    }

    fn on_level_deinit(_level: InitLevel) {
        // Nothing by default.
    }
}
