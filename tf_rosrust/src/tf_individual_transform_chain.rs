use rosrust::{Duration, Time};

use crate::{
    tf_error::TfError,
    transforms::{geometry_msgs::TransformStamped, interpolate, to_transform_stamped},
};

fn get_nanos(dur: rosrust::Duration) -> i64 {
    i64::from(dur.sec) * 1_000_000_000 + i64::from(dur.nsec)
}

fn binary_search_time(chain: &[TransformStamped], time: Time) -> Result<usize, usize> {
    chain.binary_search_by(|element| element.header.stamp.cmp(&time))
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
        self.transform_chain.last().map(|x| x.header.stamp)
    }

    pub fn add_to_buffer(&mut self, msg: TransformStamped) {
        let index = binary_search_time(&self.transform_chain, msg.header.stamp)
            .unwrap_or_else(|index| index);
        self.transform_chain.insert(index, msg);

        if let Some(newest_stamp) = self.newest_stamp() {
            if newest_stamp > Time::from_nanos(0) + self.cache_duration {
                let time_to_keep = newest_stamp - self.cache_duration;
                let index =
                    binary_search_time(&self.transform_chain, time_to_keep).unwrap_or_else(|x| x);
                self.transform_chain.drain(..index);
            }
        }
    }

    /// If timestamp is zero, return the latest transform.
    pub fn get_closest_transform(&self, time: rosrust::Time) -> Result<TransformStamped, TfError> {
        if time.nanos() == 0 {
            return Ok(self.transform_chain.last().unwrap().clone());
        }

        if self.static_tf {
            return Ok(self.transform_chain.last().unwrap().clone());
        }

        match binary_search_time(&self.transform_chain, time) {
            Ok(x) => return Ok(self.transform_chain.get(x).unwrap().clone()),
            Err(x) => {
                if x == 0 {
                    return Err(TfError::AttemptedLookupInPast(
                        time,
                        Box::new(self.transform_chain.first().unwrap().clone()),
                    ));
                }
                if x >= self.transform_chain.len() {
                    return Err(TfError::AttemptedLookUpInFuture(
                        Box::new(self.transform_chain.last().unwrap().clone()),
                        time,
                    ));
                }
                let tf1 = self.transform_chain.get(x - 1).unwrap().clone().transform;
                let tf2 = self.transform_chain.get(x).unwrap().clone().transform;
                let time1 = self.transform_chain.get(x - 1).unwrap().header.stamp;
                let time2 = self.transform_chain.get(x).unwrap().header.stamp;
                let header = self.transform_chain.get(x).unwrap().header.clone();
                let child_frame = self.transform_chain.get(x).unwrap().child_frame_id.clone();
                let total_duration = get_nanos(time2 - time1) as f64;
                let desired_duration = get_nanos(time - time1) as f64;
                let weight = 1.0 - desired_duration / total_duration;
                let final_tf = interpolate(tf1, tf2, weight);
                let ros_msg = to_transform_stamped(final_tf, header.frame_id, child_frame, time);
                Ok(ros_msg)
            }
        }
    }

    pub fn has_valid_transform(&self, time: rosrust::Time) -> bool {
        if self.transform_chain.is_empty() {
            return false;
        }

        if self.static_tf {
            return true;
        }

        let first = self.transform_chain.first().unwrap();
        let last = self.transform_chain.last().unwrap();

        time.nanos() == 0 || (time >= first.header.stamp && time <= last.header.stamp)
    }
}
