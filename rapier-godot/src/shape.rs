#![allow(clippy::module_name_repetitions)]

use std::{cell::RefCell, rc::Rc};

use godot::prelude::*;
use rapier3d::prelude::*;

pub trait RapierShape {
    fn get_data(&self) -> Variant;
    fn set_data(&mut self, data: Variant);
    fn get_shape(&self) -> SharedShape;
}

pub struct RapierShapeInstance {
    pub shape: Rc<RefCell<dyn RapierShape>>,
    pub isometry: Isometry<f32>,
    pub disabled: bool,
}

impl RapierShapeInstance {
    pub fn new(
        shape: Rc<RefCell<dyn RapierShape>>,
        isometry: Isometry<f32>,
        disabled: bool,
    ) -> Self {
        Self {
            shape,
            isometry,
            disabled,
        }
    }
}

pub struct RapierSphereShape {
    shape: Ball,
}

impl RapierSphereShape {
    pub fn new() -> Self {
        Self {
            shape: Ball::new(0.5),
        }
    }
}

impl RapierShape for RapierSphereShape {
    fn get_data(&self) -> Variant {
        Variant::from(self.shape.radius)
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to() {
            Ok(radius) => {
                self.shape.radius = radius;
            }
            Err(err) => godot_error!("{:?}", err),
        };
    }

    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape)
    }
}

pub struct RapierBoxShape {
    shape: Cuboid,
}

impl RapierBoxShape {
    pub fn new() -> Self {
        Self {
            shape: Cuboid::new(vector![0.5, 0.5, 0.5]),
        }
    }
}

impl RapierShape for RapierBoxShape {
    fn get_data(&self) -> Variant {
        Variant::from(Vector3::new(
            self.shape.half_extents.x,
            self.shape.half_extents.y,
            self.shape.half_extents.z,
        ))
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to::<Vector3>() {
            Ok(half_extents) => {
                self.shape.half_extents = vector![half_extents.x, half_extents.y, half_extents.z];
            }
            Err(err) => godot_error!("{:?}", err),
        };
    }
    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape)
    }
}

pub struct RapierCapsuleShape {
    shape: Capsule,
}

impl RapierCapsuleShape {
    pub fn new() -> Self {
        Self {
            shape: Capsule::new_y(0.5, 0.2),
        }
    }
}

impl RapierShape for RapierCapsuleShape {
    fn get_data(&self) -> Variant {
        Variant::from(dict! {"radius": self.shape.radius,"height":self.shape.height()})
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to::<Dictionary>() {
            Ok(d) => {
                match d.get_or_nil("radius").try_to() {
                    Ok(radius) => self.shape.radius = radius,
                    Err(e) => godot_error!("{:?}", e),
                };
                match d.get_or_nil("height").try_to::<f32>() {
                    Ok(height) => {
                        self.shape.segment.b = self.shape.segment.a + Vector::y() * height;
                    }
                    Err(e) => godot_error!("{:?}", e),
                };
            }
            Err(e) => godot_error!("{:?}", e),
        };
    }
    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape)
    }
}

pub struct RapierCylinderShape {
    shape: Cylinder,
}

impl RapierCylinderShape {
    pub fn new() -> Self {
        Self {
            shape: Cylinder::new(0.5, 0.2),
        }
    }
}

impl RapierShape for RapierCylinderShape {
    fn get_data(&self) -> Variant {
        Variant::from(dict! {"radius": self.shape.radius,"height":self.shape.half_height*2.0})
    }

    fn set_data(&mut self, data: Variant) {
        match data.try_to::<Dictionary>() {
            Ok(d) => {
                match d.get_or_nil("radius").try_to() {
                    Ok(radius) => self.shape.radius = radius,
                    Err(e) => godot_error!("{:?}", e),
                };
                match d.get_or_nil("height").try_to::<f32>() {
                    Ok(height) => {
                        self.shape.half_height = height * 0.5;
                    }
                    Err(e) => godot_error!("{:?}", e),
                };
            }
            Err(e) => godot_error!("{:?}", e),
        };
    }
    fn get_shape(&self) -> SharedShape {
        SharedShape::new(self.shape)
    }
}
