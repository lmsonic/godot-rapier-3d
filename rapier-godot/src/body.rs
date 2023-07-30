use godot::prelude::*;

use crate::shape::RapierObject;

impl RapierObject for RapierBody {}
#[allow(clippy::module_name_repetitions)]
pub struct RapierBody {
    rid: Rid,
}

impl RapierBody {
    pub fn new(rid: Rid) -> Self {
        Self { rid }
    }
}
