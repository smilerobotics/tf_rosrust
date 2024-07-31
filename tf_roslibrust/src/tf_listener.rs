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
/// // TODO(lucasw) make this code work
/// /*
/// use tf_roslibrust::TfListener;
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
/// */
/// ```
pub struct TfListener {
    buffer: TfBuffer,
    _static_subscriber: Subscriber<TFMessage>,
    _dynamic_subscriber: Subscriber<TFMessage>,
}

impl TfListener {
    /// Create a new TfListener
    pub async fn new(nh: &NodeHandle) -> Self {
        Self::new_with_buffer(nh, TfBuffer::new()).await
    }

    pub async fn new_with_buffer(nh: &NodeHandle, tf_buffer: TfBuffer) -> Self {
        // let buff = RwLock::new(tf_buffer);
        let buffer = tf_buffer;

        let _dynamic_subscriber = nh.subscribe::<TFMessage>("/tf", 100).await.unwrap();
        let _static_subscriber = nh.subscribe::<TFMessage>("/tf_static", 100).await.unwrap();

        TfListener {
            // buffer: arc,
            buffer,
            _static_subscriber,
            _dynamic_subscriber,
        }
    }

    pub async fn update(&mut self) {
        tokio::select! {
            rv = self._dynamic_subscriber.next() => {
                print!(".");
                match rv {
                    Some(Ok(tfm)) => {
                        self.update_tf(tfm);
                    },
                    Some(Err(error)) => {
                        panic!("{error}");
                    },
                    None => (),
                }
            },
            rv = self._static_subscriber.next() => {
                print!("+");
                match rv {
                    Some(Ok(tfm)) => {
                        self.update_tf_static(tfm);
                    },
                    Some(Err(error)) => {
                        panic!("{error}");
                    },
                    None => (),
                }
            },
        }
    }

    fn update_tf(&mut self, tfm: TFMessage) {
        // println!("{tfm:?}");
        // let r1 = self.buffer.clone();
        // r1.write().unwrap().handle_incoming_transforms(tfm, false);
        self.buffer.handle_incoming_transforms(tfm, false);
    }

    fn update_tf_static(&mut self, tfm: TFMessage) {
        // println!("static {tfm:?}");
        // let r1 = self.buffer.clone();
        // r1.write().unwrap().handle_incoming_transforms(tfm, true);
        self.buffer.handle_incoming_transforms(tfm, true);
    }

    /// Looks up a transform within the tree at a given time.
    pub fn lookup_transform(
        &self,
        from: &str,
        to: &str,
        time: Option<Time>,
    ) -> Result<TransformStamped, TfError> {
        // self.buffer.read().unwrap().lookup_transform(from, to, time)
        self.buffer.lookup_transform(from, to, time)
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
        // self.buffer
        //    .read()
        //    .unwrap()
        self.buffer
            .lookup_transform_with_time_travel(from, time1, to, time2, fixed_frame)
    }
}
