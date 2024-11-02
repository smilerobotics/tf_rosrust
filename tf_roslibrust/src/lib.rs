//! This is a rust port of the [ROS tf library](http://wiki.ros.org/tf). It is intended for being used in robots to help keep track of
//! multiple coordinate frames and is part of a larger suite of rust libraries that provide support for various robotics related functionality.
//!
//! Example usage:
//!
//! ```no_run
//! use tf_roslibrust::TfListener;
//!
//! // TODO(lucasw) make this code valid
//! /*
//! rosrust::init("listener");
//! let listener = TfListener::new();
//!
//! let rate = rosrust::rate(1.0);
//! while rosrust::is_ok() {
//!     let tf = listener.lookup_transform("camera", "base_link", rosrust::Time::new());
//!     println!("{tf:?}");
//!     rate.sleep();
//! }
//! */
//!```

use roslibrust_util::geometry_msgs;

pub trait LookupTransform {
    fn lookup_transform(
        &self,
        from: &str,
        to: &str,
        stamp0: Option<roslibrust_codegen::Time>,
    ) -> Result<geometry_msgs::TransformStamped, tf_error::TfError>;

    fn lookup_transform_with_time_travel(
        &self,
        to: &str,
        time2: roslibrust_codegen::Time,
        from: &str,
        time1: roslibrust_codegen::Time,
        fixed_frame: &str,
    ) -> Result<geometry_msgs::TransformStamped, tf_error::TfError>;
}

mod tf_broadcaster;
mod tf_buffer;
mod tf_error;
mod tf_graph_node;
mod tf_individual_transform_chain;
mod tf_listener;
pub mod tf_util;
pub mod transforms;
pub use tf_broadcaster::TfBroadcaster;
pub use tf_buffer::TfBuffer;
pub use tf_error::TfError;
pub use tf_listener::TfListener;
