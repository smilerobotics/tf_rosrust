use crate::{
    msg::{geometry_msgs::TransformStamped, tf2_msgs::TFMessage},
    tf_error::TfError,
};

pub struct TfBroadcaster {
    publisher: ros2_client::Publisher<TFMessage>,
}

impl TfBroadcaster {
    /// Create a new TfBroadcaster
    #[track_caller]
    pub fn new(node: &mut ros2_client::Node, tf_topic: &rustdds::Topic) -> Self {
        Self {
            publisher: node.create_publisher(tf_topic, None).unwrap(),
        }
    }

    /// Broadcast transform
    pub fn send_transform(&self, tf: TransformStamped) -> Result<(), TfError> {
        let tf_message = TFMessage {
            transforms: vec![tf],
        };
        // TODO: handle error correctly
        self.publisher
            .publish(tf_message)
            .map_err(|err| TfError::Ros2(err.to_string()))
    }
}
