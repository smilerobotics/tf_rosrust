use crate::{
    tf_error::TfError,
    transforms::{geometry_msgs::TransformStamped, tf2_msgs::TFMessage},
};

/// Broadcast tf messages
///
/// Example usage:
///
/// ```no_run
/// use tf_rosrust::{TfBroadcaster, TransformStamped};
///
/// rosrust::init("broadcaster");
/// let broadcaster = TfBroadcaster::new();
///
/// let rate = rosrust::rate(100.0);
/// let mut tf = TransformStamped::default();
/// tf.header.frame_id = "map".to_string();
/// tf.child_frame_id = "tf_rosrust".to_string();
/// tf.transform.rotation.w = 1.0;
/// let mut theta = 0.01_f64;
/// while rosrust::is_ok() {
///     theta += 0.01;
///     tf.header.stamp = rosrust::now();
///     tf.transform.translation.x = theta.sin();
///     tf.transform.translation.y = theta.cos();
///     broadcaster.send_transform(tf.clone()).unwrap();
///     println!("{tf:?}");
///     rate.sleep();
/// }
/// ```
pub struct TfBroadcaster {
    publisher: rosrust::Publisher<TFMessage>,
}

impl TfBroadcaster {
    /// Create a new TfBroadcaster
    pub fn new() -> Self {
        Self {
            publisher: rosrust::publish("/tf", 1000).unwrap(),
        }
    }

    /// Broadcast transform
    pub fn send_transform(&self, tf: TransformStamped) -> Result<(), TfError> {
        let tf_message = TFMessage {
            transforms: vec![tf],
        };
        // TODO: handle error correctly
        self.publisher
            .send(tf_message)
            .map_err(|err| TfError::Rosrust(err.description().to_string()))
    }
}

impl Default for TfBroadcaster {
    fn default() -> Self {
        TfBroadcaster::new()
    }
}
