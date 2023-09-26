use std::sync::{Arc, RwLock};

use crate::{tf_buffer::TfBuffer, tf_error::TfError};
use futures::StreamExt;
use r2r::{
    builtin_interfaces::msg::Time, geometry_msgs::msg::TransformStamped, tf2_msgs::msg::TFMessage,
    QosProfile,
};

pub struct TfListener {
    buffer: Arc<RwLock<TfBuffer>>,
    _node: r2r::Node,
}

impl TfListener {
    /// Create a new TfListener
    pub fn new(node: r2r::Node) -> Self {
        Self::new_with_buffer(node, TfBuffer::new())
    }

    pub fn new_with_buffer(mut node: r2r::Node, tf_buffer: TfBuffer) -> Self {
        let buff = Arc::new(RwLock::new(tf_buffer));

        let mut dynamic_subscriber = node
            .subscribe::<TFMessage>("tf", QosProfile::default())
            .unwrap();

        let buff_for_dynamic_sub = buff.clone();
        tokio::spawn(async move {
            loop {
                if let Some(tf) = dynamic_subscriber.next().await {
                    buff_for_dynamic_sub
                        .write()
                        .unwrap()
                        .handle_incoming_transforms(tf, false);
                }
            }
        });

        let mut static_subscriber = node
            .subscribe::<TFMessage>("tf_static", QosProfile::default())
            .unwrap();

        let buff_for_static_sub = buff.clone();
        tokio::spawn(async move {
            loop {
                if let Some(tf) = static_subscriber.next().await {
                    buff_for_static_sub
                        .write()
                        .unwrap()
                        .handle_incoming_transforms(tf, true);
                }
            }
        });

        TfListener {
            buffer: buff,
            _node: node,
        }
    }

    /// Looks up a transform within the tree at a given time.
    pub fn lookup_transform(
        &self,
        from: &str,
        to: &str,
        time: Time,
    ) -> Result<TransformStamped, TfError> {
        self.buffer
            .read()
            .unwrap()
            .lookup_transform(from, to, &time)
    }

    /// Looks up a transform within the tree at a given time.
    pub fn lookup_transform_with_time_travel(
        &self,
        from: &str,
        time1: Time,
        to: &str,
        time2: Time,
        fixed_frame: &str,
    ) -> Result<TransformStamped, TfError> {
        self.buffer
            .read()
            .unwrap()
            .lookup_transform_with_time_travel(from, time1, to, time2, fixed_frame)
    }
}
