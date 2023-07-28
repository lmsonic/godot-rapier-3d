use godot::engine::{
    PhysicsDirectSpaceState3DExtension, PhysicsDirectSpaceState3DExtensionVirtual,
};
use godot::prelude::*;

#[derive(GodotClass)]
#[class(base=PhysicsDirectSpaceState3DExtension)]
struct RapierPhysicsDirectSpaceState3D {
    #[base]
    base: Base<PhysicsDirectSpaceState3DExtension>,
}
#[godot_api]
impl PhysicsDirectSpaceState3DExtensionVirtual for RapierPhysicsDirectSpaceState3D {}
