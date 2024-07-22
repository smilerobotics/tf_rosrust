use std::sync::{Arc, RwLock};

use crate::{
    tf_buffer::TfBuffer,
    tf_error::TfError,
    transforms::{geometry_msgs::TransformStamped, tf2_msgs::TFMessage},
};

use roslibrust::ros1::NodeHandle;
use roslibrust::ros1::Subscriber;
use roslibrust_codegen::Time;


///This struct tries to be the same as the C++ version of `TransformListener`. Use this struct to lookup transforms.
///
/// Example usage:
///
/// ```no_run
/// use tf_rosrust::TfListener;
///
/// rosrust::init("listener");
/// let listener = TfListener::new();
///
/// let rate = rosrust::rate(1.0);
/// while rosrust::is_ok() {
///     let tf = listener.lookup_transform("camera", "base_link", rosrust::Time::new());
///     println!("{tf:?}");
///     rate.sleep();
/// }
/// ```
/// Do note that unlike the C++ variant of the TfListener, only one TfListener can be created at a time. Like its C++ counterpart,
/// it must be scoped to exist through the lifetime of the program. One way to do this is using an `Arc` or `RwLock`.
pub struct TfListener {
    buffer: Arc<RwLock<TfBuffer>>,
    _static_subscriber: Subscriber<TFMessage>,
    pub _dynamic_subscriber: Subscriber<TFMessage>,
}

impl TfListener {
    /// Create a new TfListener
    pub async fn new(nh: &NodeHandle) -> Self {
        Self::new_with_buffer(nh, TfBuffer::new()).await
    }

    pub async fn new_with_buffer(nh: &NodeHandle, tf_buffer: TfBuffer) -> Self {
        let buff = RwLock::new(tf_buffer);
        let arc = Arc::new(buff);
        // let r1 = arc.clone();
        let _dynamic_subscriber = nh.subscribe::<TFMessage>("/tf", 100).await.unwrap();
        // , move |v: TFMessage| {
        //     r1.write().unwrap().handle_incoming_transforms(v, false);
        // })

        // let r2 = arc.clone();
        let _static_subscriber = nh.subscribe::<TFMessage>("/tf_static", 100).await.unwrap();
        // , move |v: TFMessage| {
        //     r2.write().unwrap().handle_incoming_transforms(v, true);
        // })

        TfListener {
            buffer: arc,
            _static_subscriber,
            _dynamic_subscriber,
        }
    }

    pub async fn update_tf(&mut self, tfm: TFMessage) {
        // TODO(lucasw) handle SubscriberError instead of last unwrap
        // let new_tfm = self._dynamic_subscriber.next().await.unwrap().unwrap();
        // println!("{tfm:?}");
        let r1 = self.buffer.clone();
        r1.write().unwrap().handle_incoming_transforms(tfm, false);
    }

    pub async fn update_tf_static(&mut self) {
        let new_tfm = self._static_subscriber.next().await.unwrap().unwrap();
        let r1 = self.buffer.clone();
        r1.write().unwrap().handle_incoming_transforms(new_tfm, true);
    }

    /// Looks up a transform within the tree at a given time.
    pub fn lookup_transform(
        &self,
        from: &str,
        to: &str,
        time: Time,
    ) -> Result<TransformStamped, TfError> {
        self.buffer.read().unwrap().lookup_transform(from, to, time)
    }

    /// Looks up a transform within the tree at a given time for each frame with
    /// respect to a fixed frame.
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

/*
impl Default for TfListener {
    fn default(nh: &NodeHandle) -> Self {
        TfListener::new(nh)
    }
}
*/
