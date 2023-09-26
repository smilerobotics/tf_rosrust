use r2r::{
    builtin_interfaces::msg::{Duration, Time},
    geometry_msgs::msg::TransformStamped,
};

use crate::{
    tf_error::TfError,
    transforms::{interpolate, to_transform_stamped},
    utils::*,
};

fn get_nanos(dur: Duration) -> i64 {
    i64::from(dur.sec) * 1_000_000_000 + i64::from(dur.nanosec)
}

fn binary_search_time(chain: &[TransformStamped], time: &Time) -> Result<usize, usize> {
    chain.binary_search_by(|element| {
        time_as_ns_i64(&element.header.stamp).cmp(&time_as_ns_i64(&time))
    })
}

#[derive(Clone, Debug)]
pub(crate) struct TfIndividualTransformChain {
    cache_duration: Duration,
    static_tf: bool,
    //TODO:  Implement a circular buffer. Current method is slow.
    pub(crate) transform_chain: Vec<TransformStamped>,
}

impl TfIndividualTransformChain {
    pub fn new(static_tf: bool, cache_duration: Duration) -> Self {
        Self {
            cache_duration,
            transform_chain: Vec::new(),
            static_tf,
        }
    }

    pub fn newest_stamp(&self) -> Option<Time> {
        self.transform_chain.last().map(|x| x.header.stamp.clone())
    }

    pub fn add_to_buffer(&mut self, msg: TransformStamped) {
        let index = binary_search_time(&self.transform_chain, &msg.header.stamp)
            .unwrap_or_else(|index| index);
        self.transform_chain.insert(index, msg.clone());

        if let Some(newest_stamp) = self.newest_stamp() {
            if is_time_later(
                &newest_stamp,
                &add_time_and_duration(&time_from_nanosec(0), &self.cache_duration),
            ) {
                let time_to_keep = sub_duration_from_time(&newest_stamp, &self.cache_duration);
                let index =
                    binary_search_time(&self.transform_chain, &time_to_keep).unwrap_or_else(|x| x);
                self.transform_chain.drain(..index);
            }
        }
    }

    /// If timestamp is zero, return the latest transform.
    pub fn get_closest_transform(&self, time: &Time) -> Result<TransformStamped, TfError> {
        if time_as_ns_i64(time) == 0 {
            return Ok(self.transform_chain.last().unwrap().clone());
        }

        if self.static_tf {
            return Ok(self.transform_chain.last().unwrap().clone());
        }

        match binary_search_time(&self.transform_chain, &time) {
            Ok(x) => return Ok(self.transform_chain.get(x).unwrap().clone()),
            Err(x) => {
                if x == 0 {
                    return Err(TfError::AttemptedLookupInPast(
                        time.clone(),
                        Box::new(self.transform_chain.first().unwrap().clone()),
                    ));
                }
                if x >= self.transform_chain.len() {
                    return Err(TfError::AttemptedLookUpInFuture(
                        Box::new(self.transform_chain.last().unwrap().clone()),
                        time.clone(),
                    ));
                }
                let tf1 = self.transform_chain.get(x - 1).unwrap().clone().transform;
                let tf2 = self.transform_chain.get(x).unwrap().clone().transform;
                let time1 = self
                    .transform_chain
                    .get(x - 1)
                    .unwrap()
                    .header
                    .stamp
                    .clone();
                let time2 = self.transform_chain.get(x).unwrap().header.stamp.clone();
                let header = self.transform_chain.get(x).unwrap().header.clone();
                let child_frame = self.transform_chain.get(x).unwrap().child_frame_id.clone();
                let total_duration = get_nanos(sub_time_and_time(&time2, &time1)) as f64;
                let desired_duration = get_nanos(sub_time_and_time(&time, &time1)) as f64;
                let weight = 1.0 - desired_duration / total_duration;
                let final_tf = interpolate(tf1, tf2, weight);
                let ros_msg = to_transform_stamped(final_tf, header.frame_id, child_frame, &time);
                Ok(ros_msg)
            }
        }
    }

    pub fn has_valid_transform(&self, time: &Time) -> bool {
        if self.transform_chain.is_empty() {
            return false;
        }

        if self.static_tf {
            return true;
        }

        let first = self.transform_chain.first().unwrap();
        let last = self.transform_chain.last().unwrap();

        time_as_ns_i64(time) == 0
            || is_time_in_range_eq(&time, &first.header.stamp, &last.header.stamp)
    }
}
