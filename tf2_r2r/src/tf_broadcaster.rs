use crate::tf_error::TfError;
use r2r::{geometry_msgs::msg::TransformStamped, tf2_msgs::msg::TFMessage, QosProfile};

pub struct TfBroadcaster {
    publisher: r2r::Publisher<TFMessage>,
    _node: r2r::Node,
}

impl TfBroadcaster {
    /// Create a new TfBroadcaster
    pub fn new(ctx: r2r::Context) -> Self {
        let mut node = r2r::Node::create(ctx, "tf_broadcaster", "tf2_r2r").unwrap();
        Self {
            publisher: node.create_publisher("/tf", QosProfile::default()).unwrap(),
            _node: node,
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
