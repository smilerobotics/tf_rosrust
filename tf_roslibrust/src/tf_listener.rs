use crate::{tf_buffer::TfBuffer, tf_error::TfError};
use roslibrust_util::{geometry_msgs, tf2_msgs};

use roslibrust::codegen::integral_types::Time;
use roslibrust::ros1::NodeHandle;
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
    pub buffer: Arc<RwLock<TfBuffer>>,
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
        let mut dynamic_subscriber = nh
            .subscribe::<tf2_msgs::TFMessage>("/tf", 200)
            .await
            .unwrap();
        let tf_handle = tokio::spawn(async move {
            while let Some(rv) = dynamic_subscriber.next().await {
                // print!(".");
                match rv {
                    Ok(tfm) => {
                        let _ = dyn_tfm_sender.send((tfm, false));
                    }
                    Err(error) => {
                        log::info!("dynamic tf sub error: {error}");
                        break;
                    }
                }
            }
        });

        let mut static_subscriber = nh
            .subscribe::<tf2_msgs::TFMessage>("/tf_static", 200)
            .await
            .unwrap();
        let tf_static_handle = tokio::spawn(async move {
            while let Some(rv) = static_subscriber.next().await {
                // print!(".");
                match rv {
                    Ok(tfm) => {
                        let _ = static_tfm_sender.send((tfm, true));
                    }
                    Err(error) => {
                        log::info!("static tf sub error: {error}");
                        break;
                    }
                }
            }
        });

        let buffer_for_writing = buffer.clone();
        let buffer_handle = tokio::spawn(async move {
            loop {
                // This is more complicated than blocking and waiting for the next
                // tfm but is maybe more efficient
                // TODO(lucasw) possibly there is a 10-20% cpu usage reduction when tf
                // arriving at 500 Hz
                let mut tfms = Vec::new();
                let max_num_tfm = 50;
                {
                    let mut tfm_iter = tfm_receiver.try_iter();
                    // take in as many as 50 transform messages
                    for _ in 0..max_num_tfm {
                        let rv = tfm_iter.next();
                        match rv {
                            Some(tfm_static) => {
                                tfms.push(tfm_static);
                            }
                            None => {
                                break;
                            }
                        }
                    }
                }

                let num_tfms = tfms.len();
                if num_tfms > 0 {
                    // write batches of up to 50 transforms
                    let mut buffer_writer = buffer_for_writing.write().unwrap();
                    for (tfm, is_static) in tfms {
                        let rv = buffer_writer.handle_incoming_transforms(tfm, is_static);
                        if rv.is_err() {
                            // TODO(lucasw) may want to throttle this down
                            log::info!("{rv:?}");
                        }
                    }
                }

                // if tfms is full don't sleep, maybe more are queue
                if num_tfms < max_num_tfm {
                    tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
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

    // TODO(lucasw) really need a integrated test for this
    pub fn force_finish(&self) {
        log::info!("tf listener force finish");
        self.buffer_handle.abort();
        self.tf_handle.abort();
        self.tf_static_handle.abort();
        log::info!("tf listener force finish done");
    }
}

impl crate::LookupTransform for TfListener {
    /// Looks up a transform within the tree at a given time.
    fn lookup_transform(
        &self,
        from: &str,
        to: &str,
        time: Option<Time>,
    ) -> Result<geometry_msgs::TransformStamped, TfError> {
        // self.buffer.read().unwrap().lookup_transform(from, to, time)
        self.buffer.read().unwrap().lookup_transform(from, to, time)
    }

    /// Looks up a transform within the tree at a given time for each frame with
    /// respect to a fixed frame.
    fn lookup_transform_with_time_travel(
        &self,
        from: &str,
        time1: Time,
        to: &str,
        time2: Time,
        fixed_frame: &str,
    ) -> Result<geometry_msgs::TransformStamped, TfError> {
        self.buffer
            .read()
            .unwrap()
            .lookup_transform_with_time_travel(from, time1, to, time2, fixed_frame)
    }
}
