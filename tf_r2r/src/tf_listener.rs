use std::{
    sync::{Arc, RwLock},
    time::Duration,
};

use ros2_client::builtin_interfaces::Time;

use crate::{
    msg::{geometry_msgs::TransformStamped, tf2_msgs::TFMessage},
    tf_buffer::TfBuffer,
    tf_error::TfError,
};

pub struct TfListener {
    buffer: Arc<RwLock<TfBuffer>>,
}

impl TfListener {
    /// Create a new TfListener
    #[track_caller]
    pub fn new(
        node: &mut ros2_client::Node,
        tf_topic: &rustdds::Topic,
        tf_static_topic: &rustdds::Topic,
    ) -> Self {
        Self::new_with_buffer(node, tf_topic, tf_static_topic, TfBuffer::new())
    }

    #[track_caller]
    pub fn new_with_buffer(
        node: &mut ros2_client::Node,
        tf_topic: &rustdds::Topic,
        tf_static_topic: &rustdds::Topic,
        tf_buffer: TfBuffer,
    ) -> Self {
        let buff = Arc::new(RwLock::new(tf_buffer));

        let dynamic_subscriber = node
            .create_subscription::<TFMessage>(tf_topic, None)
            .unwrap();

        let buff_for_dynamic_sub = buff.clone();
        tokio::spawn(async move {
            while Arc::strong_count(&buff_for_dynamic_sub) > 1 {
                if let Ok((tf, _info)) = dynamic_subscriber.async_take().await {
                    buff_for_dynamic_sub
                        .write()
                        .unwrap()
                        .handle_incoming_transforms(tf, false);
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
        });

        let static_subscriber = node
            .create_subscription::<TFMessage>(tf_static_topic, None)
            .unwrap();

        let buff_for_static_sub = buff.clone();
        tokio::spawn(async move {
            while Arc::strong_count(&buff_for_static_sub) > 1 {
                if let Ok((tf, _info)) = static_subscriber.async_take().await {
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
