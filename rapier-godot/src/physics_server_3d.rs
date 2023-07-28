use godot::engine::{PhysicsServer3DExtension, PhysicsServer3DExtensionVirtual};
use godot::prelude::*;

#[derive(GodotClass)]
#[class(base=PhysicsServer3DExtension)]
struct RapierPhysicsServer3DExtension {
    #[base]
    base: Base<PhysicsServer3DExtension>,
}
#[godot_api]
impl PhysicsServer3DExtensionVirtual for RapierPhysicsServer3DExtension {}
