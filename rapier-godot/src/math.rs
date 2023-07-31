use godot::prelude::*;
use rapier3d::{
    na::{Translation3, UnitQuaternion},
    prelude::*,
};

type RapierQuaternion = rapier3d::na::Quaternion<f32>;

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
