use godot::prelude::Rid;

pub struct RapierJoint {
    rid: Rid,
}

impl RapierJoint {
    pub const fn new(rid: Rid) -> Self {
        Self { rid }
    }
}
