use crate::{
    tf_buffer::TfBuffer,
    tf_error::TfError,
    transforms::{geometry_msgs::TransformStamped, tf2_msgs::TFMessage},
};

use roslibrust::ros1::NodeHandle;
use roslibrust_codegen::Time;
use std::sync::{mpsc, Arc, RwLock};

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
    buffer: Arc<RwLock<TfBuffer>>,
    buffer_handle: tokio::task::JoinHandle<()>,
    tf_handle: tokio::task::JoinHandle<()>,
    tf_static_handle: tokio::task::JoinHandle<()>,
}

impl TfListener {
    /// Create a new TfListener
    pub async fn new(nh: &NodeHandle) -> Self {
        let buffer = Arc::new(RwLock::new(TfBuffer::new()));

        let (static_tfm_sender, tfm_receiver) = mpsc::sync_channel(4000);

        let dyn_tfm_sender = static_tfm_sender.clone();
        // let dyn_nh = nh.clone();
        let mut dynamic_subscriber = nh.subscribe::<TFMessage>("/tf", 200).await.unwrap();
        let tf_handle = tokio::spawn(async move {
            while let Some(rv) = dynamic_subscriber.next().await {
                print!(".");
                match rv {
                    Ok(tfm) => {
                        let _ = dyn_tfm_sender.send((tfm, false));
                    }
                    Err(error) => {
                        panic!("{error}");
                    }
                }
            }
        });

        let mut static_subscriber = nh.subscribe::<TFMessage>("/tf_static", 200).await.unwrap();
        let tf_static_handle = tokio::spawn(async move {
            while let Some(rv) = static_subscriber.next().await {
                print!(".");
                match rv {
                    Ok(tfm) => {
                        let _ = static_tfm_sender.send((tfm, true));
                    }
                    Err(error) => {
                        panic!("{error}");
                    }
                }
            }
        });

        let buffer_for_writing = buffer.clone();
        let buffer_handle = tokio::spawn(async move {
            loop {
                let rv = tfm_receiver.recv();
                match rv {
                    Ok((tfm, is_static)) => {
                        // r1.write().unwrap().handle_incoming_transforms(tfm, true);
                        buffer_for_writing
                            .write()
                            .unwrap()
                            .handle_incoming_transforms(tfm, is_static);
                    }
                    Err(e) => {
                        // TODO(lucasw) the panic only shuts down this thread, not the whole
                        // application
                        panic!("{e:?} - did this node get killed?");
                    }
                }
            }
        });

        TfListener {
            buffer,
            buffer_handle,
            tf_handle,
            tf_static_handle,
        }
    }

    pub fn is_finished(&self) -> bool {
        self.buffer_handle.is_finished()
            || self.tf_handle.is_finished()
            || self.tf_static_handle.is_finished()
    }

    /// Looks up a transform within the tree at a given time.
    pub fn lookup_transform(
        &self,
        from: &str,
        to: &str,
        time: Option<Time>,
    ) -> Result<TransformStamped, TfError> {
        // self.buffer.read().unwrap().lookup_transform(from, to, time)
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
