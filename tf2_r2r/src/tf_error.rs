use std::collections::{HashMap, HashSet};

use r2r::{builtin_interfaces::msg::Time, geometry_msgs::msg::TransformStamped};
use thiserror::Error;

/// Enumerates the different types of errors
#[derive(Clone, Debug, Error)]
#[non_exhaustive]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    #[error("tf2_r2r: AttemptedLookupInPast {:?} < {:?}",.0, .1)]
    AttemptedLookupInPast(Time, Box<TransformStamped>),
    /// Error due to the transform not yet being available.
    #[error("tf2_r2r: AttemptedLookupInFuture {:?} < {:?}",.0, .1)]
    AttemptedLookUpInFuture(Box<TransformStamped>, Time),
    /// There is no path between the from and to frame.
    #[error("tf2_r2r: CouldNotFindTransform {} -> {} ({:?})", .0, .1, .2)]
    CouldNotFindTransform(String, String, HashMap<String, HashSet<String>>),
    /// In the event that a write is simultaneously happening with a read of the same tf buffer
    #[error("tf2_r2r: CouldNotAcquireLock")]
    CouldNotAcquireLock,
    /// Error of r2r
    #[error("tf2_r2r: r2r error {:?}", .0)]
    R2r(String),
}
