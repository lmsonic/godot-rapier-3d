/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

use godot::prelude::*;

struct RapierPhysics;

mod direct_body_state_3d;
mod direct_space_state_3d;
mod physics_server_3d;

#[gdextension]
unsafe impl ExtensionLibrary for RapierPhysics {}
