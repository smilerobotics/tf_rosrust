use std::collections::{HashMap, HashSet};

use ros2_client::builtin_interfaces::Time;
use thiserror::Error;

use crate::msg::geometry_msgs::TransformStamped;

/// Enumerates the different types of errors
#[derive(Clone, Debug, Error)]
#[non_exhaustive]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    #[error("tf_ros2: AttemptedLookupInPast {:?} < {:?}",.0, .1)]
    AttemptedLookupInPast(Time, Box<TransformStamped>),
    /// Error due to the transform not yet being available.
    #[error("tf_ros2: AttemptedLookupInFuture {:?} < {:?}",.0, .1)]
    AttemptedLookUpInFuture(Box<TransformStamped>, Time),
    /// There is no path between the from and to frame.
    #[error("tf_ros2: CouldNotFindTransform {} -> {} ({:?})", .0, .1, .2)]
    CouldNotFindTransform(String, String, HashMap<String, HashSet<String>>),
    /// In the event that a write is simultaneously happening with a read of the same tf buffer
    #[error("tf_ros2: CouldNotAcquireLock")]
    CouldNotAcquireLock,
    /// Error of ros2-client
    #[error("tf_ros2: ros2-client error {:?}", .0)]
    Ros2(String),
}
