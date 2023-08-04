use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::prelude::*;

use crate::collision_object::RapierCollisionObject;

use super::RapierShape;
pub struct RapierWorldBoundaryShape {
    shape: HalfSpace,
    owners: Vec<Rc<RefCell<dyn RapierCollisionObject>>>,
    rid: Rid,
}

impl RapierWorldBoundaryShape {
    pub fn new(rid: Rid) -> Self {
        Self {
            shape: HalfSpace::new(UnitVector::new_normalize(vector![0.0, 1.0, 0.0])),
            owners: vec![],
            rid,
        }
    }
}
impl RapierShape for RapierWorldBoundaryShape {
    fn rid(&self) -> Rid {
        self.rid
    }

    fn get_data(&self) -> Variant {
        let normal = Vector3::new(
            self.shape.normal.x,
            self.shape.normal.y,
            self.shape.normal.z,
        );
        Variant::from(Plane::new(normal, 0.0))
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to::<Plane>() {
            Ok(plane) => {
                let normal = UnitVector::new_normalize(vector![
                    plane.normal.x,
                    plane.normal.y,
                    plane.normal.z
                ]);
                if normal != self.shape.normal {
                    self.shape = HalfSpace::new(normal);
                    self.update_owners();
                }
                if plane.d != 0.0 {
                    godot_warn!(
                        "Godot Rapier doesn't support WorldBoundaryShapes not set in the origin"
                    );
                }
            }
            Err(err) => godot_error!("{:?}", err),
        }
    }

    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape)
    }

    fn get_type(&self) -> godot::engine::physics_server_3d::ShapeType {
        godot::engine::physics_server_3d::ShapeType::SHAPE_WORLD_BOUNDARY
    }

    fn owners(&self) -> &Vec<Rc<RefCell<dyn RapierCollisionObject>>> {
        &self.owners
    }
}
