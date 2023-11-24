use r2r::{geometry_msgs::msg::TransformStamped, tf2_msgs::msg::TFMessage, QosProfile};

use crate::tf_error::TfError;

pub struct TfBroadcaster {
    publisher: r2r::Publisher<TFMessage>,
}

impl TfBroadcaster {
    /// Create a new TfBroadcaster
    #[track_caller]
    pub fn new(node: &mut r2r::Node) -> Self {
        Self {
            publisher: node.create_publisher("/tf", QosProfile::default()).unwrap(),
        }
    }

    /// Broadcast transform
    pub fn send_transform(&self, tf: TransformStamped) -> Result<(), TfError> {
        let tf_message = TFMessage {
            transforms: vec![tf],
        };
        // TODO: handle error correctly
        self.publisher
            .publish(&tf_message)
            .map_err(|err| TfError::R2r(err.to_string()))
    }
}
