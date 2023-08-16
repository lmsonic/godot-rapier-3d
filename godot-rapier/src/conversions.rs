use godot::{engine::physics_server_3d::BodyMode, prelude::*};
use rapier3d::{
    na::{Translation3, UnitQuaternion},
    prelude::*,
};

// TODO: bypass orphan rule for From using Newtype pattern

pub(crate) trait FromExt<T>: Sized {
    /// Converts to this type from the input type.
    #[must_use]
    fn from_ext(value: T) -> Self;
}

pub(crate) trait IntoExt<T>: Sized {
    /// Converts this type into the (usually inferred) input type.
    #[must_use]
    fn into_ext(self) -> T;
}

impl<T, U> IntoExt<U> for T
where
    U: FromExt<T>,
{
    #[inline]
    fn into_ext(self) -> U {
        U::from_ext(self)
    }
}
impl<T> FromExt<T> for T {
    #[inline(always)]
    fn from_ext(t: T) -> T {
        t
    }
}

impl FromExt<Vector3> for Vector<f32> {
    fn from_ext(v: Vector3) -> Self {
        vector![v.x, v.y, v.z]
    }
}

impl FromExt<Vector<f32>> for Vector3 {
    fn from_ext(v: Vector<f32>) -> Self {
        Self::new(v.x, v.y, v.z)
    }
}

impl FromExt<Vector3> for Point<f32> {
    fn from_ext(v: Vector3) -> Self {
        point![v.x, v.y, v.z]
    }
}

impl FromExt<Point<f32>> for Vector3 {
    fn from_ext(v: Point<f32>) -> Self {
        Self::new(v.x, v.y, v.z)
    }
}
type RapierQuaternion = rapier3d::na::Quaternion<f32>;

impl FromExt<Transform3D> for (Isometry<f32>, Vector<f32>) {
    fn from_ext(transform: Transform3D) -> Self {
        let translation =
            Translation3::new(transform.origin.x, transform.origin.y, transform.origin.z);
        let scale = transform.basis.scale();
        let godot_quat = transform.basis.orthonormalized().to_quat();
        let rapier_quat =
            RapierQuaternion::new(godot_quat.x, godot_quat.y, godot_quat.z, godot_quat.w);

        (
            Isometry::from_parts(translation, UnitQuaternion::from_quaternion(rapier_quat)),
            scale.into_ext(),
        )
    }
}

impl FromExt<Isometry<f32>> for Transform3D {
    fn from_ext(isometry: Isometry<f32>) -> Self {
        let origin = Vector3::new(
            isometry.translation.x,
            isometry.translation.y,
            isometry.translation.z,
        );
        let rapier_quat = isometry.rotation.coords;
        let godot_quat =
            Quaternion::new(rapier_quat.x, rapier_quat.y, rapier_quat.z, rapier_quat.w);
        Transform3D {
            basis: Basis::from_quat(godot_quat),
            origin,
        }
    }
}

impl FromExt<BodyMode> for RigidBodyType {
    #[inline]
    fn from_ext(mode: BodyMode) -> Self {
        match mode {
            BodyMode::BODY_MODE_RIGID | BodyMode::BODY_MODE_RIGID_LINEAR => RigidBodyType::Dynamic,
            BodyMode::BODY_MODE_KINEMATIC => RigidBodyType::KinematicPositionBased,
            _ => RigidBodyType::Fixed,
        }
    }
}
