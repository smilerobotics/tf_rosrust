use crate::tf_error::TfError;
use roslibrust::ros1::NodeHandle;
use roslibrust::ros1::Publisher;
use roslibrust_util::{geometry_msgs, tf2_msgs};

/// Broadcast tf messages
///
/// Example usage:
///
/// TODO(lucasw) update this to use roslibrust
///
/// ```no_run
/// /*
/// use tf_rosrust::{TfBroadcaster, TransformStamped};
///
/// rosrust::init("broadcaster");
/// let broadcaster = TfBroadcaster::new(nh);
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
/// */
/// ```
pub struct TfBroadcaster {
    publisher: Publisher<tf2_msgs::TFMessage>,
}

impl TfBroadcaster {
    /// Create a new TfBroadcaster
    pub async fn new(nh: &NodeHandle) -> Self {
        let latching = false;
        Self {
            publisher: nh.advertise("/tf", 1000, latching).await.unwrap(),
        }
    }

    // TODO(lucasw) need be able to send list of transforms
    /// Broadcast transform
    pub async fn send_transform(&self, tf: geometry_msgs::TransformStamped) -> Result<(), TfError> {
        let tf_message = tf2_msgs::TFMessage {
            transforms: vec![tf],
        };
        // TODO: handle error correctly
        self.publisher
            .publish(&tf_message)
            .await
            // .map_err(|err| TfError::Rosrust(err.description().to_string()))
            .map_err(|_err| TfError::Rosrust("TODO failed publish".to_string()))
    }
}

/*
impl Default for TfBroadcaster {
    fn default(nh: &NodeHandle) -> Self {
        TfBroadcaster::new(nh)
    }
}
*/
