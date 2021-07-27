use rosrust::{Duration, Time};

use crate::{
    ordered_tf::OrderedTF,
    tf_error::TfError,
    transforms::{geometry_msgs::TransformStamped, interpolate, to_transform_stamped},
};

fn get_nanos(dur: rosrust::Duration) -> i64 {
    i64::from(dur.sec) * 1_000_000_000 + i64::from(dur.nsec)
}

#[derive(Clone, Debug)]
pub(crate) struct TfIndividualTransformChain {
    cache_duration: Duration,
    static_tf: bool,
    //TODO:  Implement a circular buffer. Current method is slowww.
    pub(crate) transform_chain: Vec<OrderedTF>,
    latest_stamp: Time,
}

impl TfIndividualTransformChain {
    pub fn new(static_tf: bool, cache_duration: Duration) -> Self {
        Self {
            cache_duration,
            transform_chain: Vec::new(),
            static_tf,
            latest_stamp: Time::from_nanos(0),
        }
    }

    pub fn add_to_buffer(&mut self, msg: TransformStamped) {
        if msg.header.stamp > self.latest_stamp {
            self.latest_stamp = msg.header.stamp;
        }
        let res = self
            .transform_chain
            .binary_search(&OrderedTF { tf: msg.clone() });

        match res {
            Ok(x) => self.transform_chain.insert(x, OrderedTF { tf: msg }),
            Err(x) => self.transform_chain.insert(x, OrderedTF { tf: msg }),
        }

        let time_to_keep = if self.latest_stamp > Time::from_nanos(0) + self.cache_duration {
            self.latest_stamp - self.cache_duration
        } else {
            Time::from_nanos(0)
        };
        while !self.transform_chain.is_empty() {
            if let Some(first) = self.transform_chain.first() {
                if first.tf.header.stamp < time_to_keep {
                    self.transform_chain.remove(0);
                } else {
                    break;
                }
            }
        }
    }

    pub fn get_closest_transform(&self, time: rosrust::Time) -> Result<TransformStamped, TfError> {
        if self.static_tf {
            return Ok(self.transform_chain.last().unwrap().tf.clone());
        }

        let mut res = TransformStamped::default();
        res.header.stamp = time;
        res.transform.rotation.w = 1f64;

        let res = self.transform_chain.binary_search(&OrderedTF { tf: res });

        match res {
            Ok(x) => return Ok(self.transform_chain.get(x).unwrap().tf.clone()),
            Err(x) => {
                if x == 0 {
                    return Err(TfError::AttemptedLookupInPast(
                        time,
                        self.transform_chain.first().unwrap().tf.clone(),
                    ));
                }
                if x >= self.transform_chain.len() {
                    return Err(TfError::AttemptedLookUpInFuture(
                        self.transform_chain.last().unwrap().tf.clone(),
                        time,
                    ));
                }
                let tf1 = self
                    .transform_chain
                    .get(x - 1)
                    .unwrap()
                    .clone()
                    .tf
                    .transform;
                let tf2 = self.transform_chain.get(x).unwrap().clone().tf.transform;
                let time1 = self.transform_chain.get(x - 1).unwrap().tf.header.stamp;
                let time2 = self.transform_chain.get(x).unwrap().tf.header.stamp;
                let header = self.transform_chain.get(x).unwrap().tf.header.clone();
                let child_frame = self
                    .transform_chain
                    .get(x)
                    .unwrap()
                    .tf
                    .child_frame_id
                    .clone();
                let total_duration = get_nanos(time2 - time1) as f64;
                let desired_duration = get_nanos(time - time1) as f64;
                let weight = 1.0 - desired_duration / total_duration;
                let final_tf = interpolate(tf1, tf2, weight);
                let ros_msg = to_transform_stamped(final_tf, header.frame_id, child_frame, time);
                Ok(ros_msg)
            }
        }
    }
}
