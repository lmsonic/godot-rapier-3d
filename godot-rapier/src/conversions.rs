use godot::{engine::physics_server_3d::BodyMode, prelude::*};
use rapier3d::{
    na::{Translation3, UnitQuaternion},
    prelude::*,
};

type RapierQuaternion = rapier3d::na::Quaternion<f32>;
#[inline]
pub fn rapier_vector_to_godot_vector(v: Vector<f32>) -> Vector3 {
    Vector3::new(v.x, v.y, v.z)
}

#[inline]
pub fn rapier_point_to_godot_vector(v: Point<f32>) -> Vector3 {
    Vector3::new(v.x, v.y, v.z)
}
#[inline]
pub const fn godot_vector_to_rapier_vector(v: Vector3) -> Vector<f32> {
    vector![v.x, v.y, v.z]
}
#[inline]
pub const fn godot_vector_to_rapier_point(v: Vector3) -> Point<f32> {
    point![v.x, v.y, v.z]
}

pub fn transform_to_isometry(transform: &Transform3D) -> Isometry<f32> {
    let translation = Translation3::new(transform.origin.x, transform.origin.y, transform.origin.z);
    let godot_quat = transform.basis.orthonormalized().to_quat();
    let rapier_quat = RapierQuaternion::new(godot_quat.x, godot_quat.y, godot_quat.z, godot_quat.w);

    Isometry::from_parts(translation, UnitQuaternion::from_quaternion(rapier_quat))
}
pub fn isometry_to_transform(isometry: &Isometry<f32>) -> Transform3D {
    let origin = Vector3::new(
        isometry.translation.x,
        isometry.translation.y,
        isometry.translation.z,
    );
    let rapier_quat = isometry.rotation.coords;
    let godot_quat = Quaternion::new(rapier_quat.x, rapier_quat.y, rapier_quat.z, rapier_quat.w);
    Transform3D {
        basis: Basis::from_quat(godot_quat),
        origin,
    }
}

#[inline]
pub const fn body_mode_to_body_type(mode: BodyMode) -> RigidBodyType {
    match mode {
        BodyMode::BODY_MODE_RIGID | BodyMode::BODY_MODE_RIGID_LINEAR => RigidBodyType::Dynamic,
        BodyMode::BODY_MODE_KINEMATIC => RigidBodyType::KinematicVelocityBased,
        _ => RigidBodyType::Fixed,
    }
}
