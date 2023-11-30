use std::{
    sync::{Arc, RwLock},
    time::Duration,
};

use futures::StreamExt;
use r2r::{
    builtin_interfaces::msg::Time, geometry_msgs::msg::TransformStamped, tf2_msgs::msg::TFMessage,
    QosProfile,
};

use crate::{tf_buffer::TfBuffer, tf_error::TfError};

pub struct TfListener {
    buffer: Arc<RwLock<TfBuffer>>,
}

impl TfListener {
    /// Create a new TfListener
    #[track_caller]
    pub fn new(node: &mut r2r::Node) -> Self {
        Self::new_with_buffer(node, TfBuffer::new())
    }

    #[track_caller]
    pub fn new_with_buffer(node: &mut r2r::Node, tf_buffer: TfBuffer) -> Self {
        let buff = Arc::new(RwLock::new(tf_buffer));

        let mut dynamic_subscriber = node
            .subscribe::<TFMessage>("/tf", QosProfile::default())
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
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
        });

        let mut static_subscriber = node
            .subscribe::<TFMessage>("/tf_static", QosProfile::default())
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
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
        });

        TfListener { buffer: buff }
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
