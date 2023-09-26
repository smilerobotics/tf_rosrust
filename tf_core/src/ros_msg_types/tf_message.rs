use crate::TransformStamped;

#[cfg(feature = "ros")]
ros_nalgebra::rosmsg_include!(tf2_msgs / TFMessage);

#[derive(Debug, Clone)]
pub struct TFMessage {
    pub transforms: Vec<TransformStamped>,
}

#[cfg(feature = "ros")]
impl From<TFMessage> for tf2_msgs::TFMessage {
    fn from(value: TFMessage) -> Self {
        let mut tf_message_inner = vec![];
        for t in value.transforms {
            tf_message_inner.push(t.into());
        }
        tf2_msgs::TFMessage {
            transforms: tf_message_inner,
        }
    }
}

#[cfg(feature = "ros")]
impl From<tf2_msgs::TFMessage> for TFMessage {
    fn from(value: tf2_msgs::TFMessage) -> Self {
        let mut tf_message_inner = vec![];
        for t in value.transforms {
            tf_message_inner.push(t.into());
        }
        Self {
            transforms: tf_message_inner,
        }
    }
}

#[cfg(feature = "ros2")]
impl From<TFMessage> for r2r::tf2_msgs::msg::TFMessage {
    fn from(value: TFMessage) -> Self {
        let mut tf_message_inner = vec![];
        for t in value.transforms {
            tf_message_inner.push(t.into());
        }
        r2r::tf2_msgs::msg::TFMessage {
            transforms: tf_message_inner,
        }
    }
}

#[cfg(feature = "ros2")]
impl From<r2r::tf2_msgs::msg::TFMessage> for TFMessage {
    fn from(value: r2r::tf2_msgs::msg::TFMessage) -> Self {
        let mut tf_message_inner = vec![];
        for t in value.transforms {
            tf_message_inner.push(t.into());
        }
        Self {
            transforms: tf_message_inner,
        }
    }
}
