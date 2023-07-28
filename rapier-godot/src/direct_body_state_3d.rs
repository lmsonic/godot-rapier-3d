use godot::engine::{PhysicsDirectBodyState3DExtension, PhysicsDirectBodyState3DExtensionVirtual};
use godot::prelude::*;

#[derive(GodotClass)]
#[class(base=PhysicsDirectBodyState3DExtension)]
struct RapierPhysicsDirectBodyState3D {
    #[base]
    base: Base<PhysicsDirectBodyState3DExtension>,
}
#[godot_api]
impl PhysicsDirectBodyState3DExtensionVirtual for RapierPhysicsDirectBodyState3D {}
