use godot::prelude::*;

use crate::shape::RapierCollisionObject;

impl RapierCollisionObject for RapierBody {}
#[allow(clippy::module_name_repetitions)]
pub struct RapierBody {
    rid: Rid,
}

impl RapierBody {
    pub const fn new(rid: Rid) -> Self {
        Self { rid }
    }
}
