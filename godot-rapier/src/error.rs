use godot::prelude::Rid;
use thiserror::Error;

pub type RapierResult<T> = Result<T, RapierError>;

#[derive(Error, Debug)]
pub enum RapierError {
    #[error("{0} doesn't correspond to any shape")]
    ShapeRidMissing(Rid),
    #[error("{0} doesn't correspond to any area")]
    AreaRidMissing(Rid),
    #[error("{0} doesn't correspond to any body")]
    BodyRidMissing(Rid),
    #[error("{0} doesn't correspond to any space")]
    SpaceRidMissing(Rid),
    #[error("{0} doesn't correspond to any joint")]
    JointRidMissing(Rid),
    // TODO: Change these error to print the necessary missing info: AREA, BODY
    #[error("Object with {0} doesn't have any space set")]
    ObjectSpaceNotSet(Rid),
    #[error("Area with {0} doesn't have any rapier collider handle set")]
    AreaHandleNotSet(Rid),
    // TODO: Change these error to print the necessary missing info
    #[error("Area with {0} has invalid collider handle")]
    AreaHandleInvalid(Rid),
    #[error("Body with {0} doesn't have any rapier rigid body handle set")]
    BodyHandleNotSet(Rid),
    // TODO: Change these error to print the necessary missing info
    #[error("Body with {0} has invalid rigid body handle")]
    BodyHandleInvalid(Rid),
    #[error("Area with {0} doesn't have any instance ID set")]
    AreaInstanceIDNotSet(Rid),
    #[error("Body with {0} doesn't have any instance ID set")]
    BodyInstanceIDNotSet(Rid),
    #[error("Shape with index {0} isn't present in object with {1}")]
    ShapeNotInObject(usize, Rid),
    // TODO: Warning?
    #[error("Object with {0} is being build without shape (will use a disabled sphere as a stub)")]
    BuildingObjectWithNoShapes(Rid),
}
