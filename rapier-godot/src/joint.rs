use godot::prelude::Rid;

pub struct RapierJoint {
    rid: Rid,
}

impl RapierJoint {
    pub fn new(rid: Rid) -> Self {
        Self { rid }
    }
}
