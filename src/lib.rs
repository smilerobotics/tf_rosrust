//! This is a rust port of the [ROS tf library](http://wiki.ros.org/tf). It is intended for being used in robots to help keep track of
//! multiple coordinate frames and is part of a larger suite of rust libraries that provide support for various robotics related functionality.
//!
//! Example usage:
//!
//! ```no_run
//! use tf_rosrust::TfListener;
//!
//! rosrust::init("listener");
//! let listener = TfListener::new();
//!
//! let rate = rosrust::rate(1.0);
//! while rosrust::is_ok() {
//!     let tf = listener.lookup_transform("camera", "base_link", rosrust::Time::new());
//!     println!("{tf:?}");
//!     rate.sleep();
//! }
//!```

mod tf_broadcaster;
mod tf_buffer;
mod tf_error;
mod tf_graph_node;
mod tf_individual_transform_chain;
pub mod transforms;
pub use transforms::geometry_msgs::TransformStamped;
mod tf_listener;
pub use tf_broadcaster::TfBroadcaster;
pub use tf_buffer::TfBuffer;
pub use tf_error::TfError;
pub use tf_listener::TfListener;
