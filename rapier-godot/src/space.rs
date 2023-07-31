use std::{cell::RefCell, rc::Rc};

use godot::{engine::physics_server_3d::BodyMode, prelude::*};
use rapier3d::prelude::*;

use crate::{
    area::RapierArea, body::RapierBody, collision_object::RapierCollisionObject,
    math::isometry_to_transform,
};

#[derive(Default)]
#[allow(clippy::module_name_repetitions)]
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

const fn body_mode_to_body_type(mode: BodyMode) -> RigidBodyType {
    match mode {
        BodyMode::BODY_MODE_RIGID | BodyMode::BODY_MODE_RIGID_LINEAR => RigidBodyType::Dynamic,
        BodyMode::BODY_MODE_KINEMATIC => RigidBodyType::KinematicVelocityBased,
        _ => RigidBodyType::Fixed,
    }
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

    pub fn get_area_transform(&self, handle: ColliderHandle) -> Option<Transform3D> {
        let collider = self.collider_set.get(handle)?;
        Some(isometry_to_transform(collider.position()))
    }

    pub fn add_area(&mut self, area: &Rc<RefCell<RapierArea>>) {
        let binding = area.borrow();
        let shapes = binding.get_shapes();
        let collider = if shapes.len() == 1 {
            let shape_instance = &shapes[0];
            let shape = shape_instance.shape.borrow().get_shape();

            ColliderBuilder::new(shape)
                .position(shape_instance.isometry)
                .sensor(true)
                .enabled(!shape_instance.disabled)
                .build()
        } else {
            let compound_shapes: Vec<(Isometry<f32>, SharedShape)> = shapes
                .iter()
                .filter(|shape_instance| !shape_instance.disabled)
                .map(|shape_instance| {
                    (
                        shape_instance.isometry,
                        shape_instance.shape.borrow().get_shape(),
                    )
                })
                .collect();
            ColliderBuilder::compound(compound_shapes)
                .sensor(true)
                .build()
        };

        let handle = self.collider_set.insert(collider);
        area.borrow_mut().set_handle(handle);
    }

    pub fn add_body(&mut self, body: &Rc<RefCell<RapierBody>>) {
        let binding = body.borrow();
        let shapes = binding.get_shapes();
        let collider = if shapes.len() == 1 {
            let shape_instance = &shapes[0];
            let shape = shape_instance.shape.borrow().get_shape();

            ColliderBuilder::new(shape)
                .position(shape_instance.isometry)
                .sensor(true)
                .enabled(!shape_instance.disabled)
                .build()
        } else {
            let compound_shapes: Vec<(Isometry<f32>, SharedShape)> = shapes
                .iter()
                .filter(|shape_instance| !shape_instance.disabled)
                .map(|shape_instance| {
                    (
                        shape_instance.isometry,
                        shape_instance.shape.borrow().get_shape(),
                    )
                })
                .collect();
            ColliderBuilder::compound(compound_shapes)
                .sensor(true)
                .build()
        };
        let body_type = body_mode_to_body_type(body.borrow().get_body_mode());
        let handle = self.rigid_body_set.insert(RigidBodyBuilder::new(body_type));
        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        body.borrow_mut().set_handle(handle);
    }
}
