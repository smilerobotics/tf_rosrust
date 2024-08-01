use std::collections::{HashMap, HashSet};

use roslibrust_codegen::Time;
use thiserror::Error;

use crate::transforms::geometry_msgs::TransformStamped;

/// Enumerates the different types of errors
#[derive(Clone, Debug, Error)]
#[non_exhaustive]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    #[error("tf_rosrust: AttemptedLookupInPast {:?} < {:?}",.0, .1)]
    AttemptedLookupInPast(Time, Box<TransformStamped>),
    /// Error due to the transform not yet being available.
    #[error("tf_rosrust: AttemptedLookupInFuture {:?} < {:?}",.0, .1)]
    AttemptedLookUpInFuture(Box<TransformStamped>, Time),
    /// There is a loop in the path between frames.
    #[error("tf_rosrust: LoopDetected between {} and root -> ({:?})", .0, .1)]
    LoopDetected(String, HashMap<String, HashSet<String>>),
    /// There is no path between the from and to frame (disconnected trees).
    #[error("tf_rosrust: CouldNotFindTransform {} -> {} ({:?})", .0, .1, .2)]
    CouldNotFindTransform(String, String, HashMap<String, HashSet<String>>),
    /// In the event that a write is simultaneously happening with a read of the same tf buffer
    #[error("tf_rosrust: CouldNotAcquireLock")]
    CouldNotAcquireLock,
    /// Error of rosrust
    #[error("tf_rosrust: rosrust error {:?}", .0)]
    Rosrust(String),
}
