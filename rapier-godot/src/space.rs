use std::{borrow, cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::prelude::*;

use crate::{area::RapierArea, body::RapierBody};

#[derive(Default)]
pub struct RapierSpace {
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    gravity: Vector<Real>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    physics_hooks: (),
    event_handler: (),
}

impl RapierSpace {
    pub fn new() -> Self {
        Self::default()
    }

    fn step(&mut self) {
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,
            &self.physics_hooks,
            &self.event_handler,
        );
    }

    pub fn add_area(&mut self, area: &Rc<RefCell<RapierArea>>) {
        let collider = if area.borrow().shapes.len() == 1 {
            let shape_instance = &area.borrow().shapes[0];
            let shape = shape_instance.shape.borrow().get_shape();

            ColliderBuilder::new(shape)
                .position(shape_instance.isometry)
                .sensor(true)
                .enabled(!shape_instance.disabled)
                .build()
        } else {
            let compound_shapes: Vec<(Isometry<f32>, SharedShape)> = area
                .borrow()
                .shapes
                .iter()
                .filter(|shape_instance| !shape_instance.disabled)
                .map(|shape_instance| {
                    (
                        shape_instance.isometry,
                        shape_instance.shape.borrow().get_shape(),
                    )
                })
                .collect();
            ColliderBuilder::compound(compound_shapes).build()
        };

        self.collider_set.insert(collider);
    }

    pub fn add_body(&mut self, body: &Rc<RefCell<RapierBody>>) {
        let collider = if body.borrow().shapes.len() == 1 {
            let shape_instance = &body.borrow().shapes[0];
            let shape = shape_instance.shape.borrow().get_shape();

            ColliderBuilder::new(shape)
                .position(shape_instance.isometry)
                .sensor(true)
                .enabled(!shape_instance.disabled)
                .build()
        } else {
            let compound_shapes: Vec<(Isometry<f32>, SharedShape)> = body
                .borrow()
                .shapes
                .iter()
                .filter(|shape_instance| !shape_instance.disabled)
                .map(|shape_instance| {
                    (
                        shape_instance.isometry,
                        shape_instance.shape.borrow().get_shape(),
                    )
                })
                .collect();
            ColliderBuilder::compound(compound_shapes).build()
        };
    }
}
