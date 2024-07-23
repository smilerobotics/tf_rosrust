use roslibrust_codegen::Time;
use chrono::TimeDelta;

use crate::{
    tf_error::TfError,
    transforms::{geometry_msgs::TransformStamped, interpolate, to_transform_stamped},
};

pub fn stamp_to_duration(stamp: Time) -> TimeDelta
{
    TimeDelta::new(stamp.secs.into(), stamp.nsecs).unwrap()
}

fn binary_search_time(chain: &[TransformStamped], time: TimeDelta) -> Result<usize, usize> {
    chain.binary_search_by(|element| stamp_to_duration(element.header.stamp.clone()).cmp(&time))
}

#[derive(Clone, Debug)]
pub(crate) struct TfIndividualTransformChain {
    cache_duration: TimeDelta,
    static_tf: bool,
    // TODO: Implement a circular buffer. Current method is slow.
    pub(crate) transform_chain: Vec<TransformStamped>,
}

impl TfIndividualTransformChain {
    pub(crate) fn new(static_tf: bool, cache_duration: TimeDelta) -> Self {
        Self {
            cache_duration,
            transform_chain: Vec::new(),
            static_tf,
        }
    }

    fn newest_stamp(&self) -> Option<TimeDelta> {
        self.transform_chain.last().map(|x| stamp_to_duration(x.header.stamp.clone()))
    }

    pub(crate) fn add_to_buffer(&mut self, msg: TransformStamped) {
        let index = binary_search_time(&self.transform_chain, stamp_to_duration(msg.header.stamp.clone()))
            .unwrap_or_else(|index| index);
        self.transform_chain.insert(index, msg);

        if let Some(newest_stamp) = self.newest_stamp() {
            if newest_stamp > self.cache_duration {
                let time_to_keep = newest_stamp - self.cache_duration;
                let index =
                    binary_search_time(&self.transform_chain, time_to_keep).unwrap_or_else(|x| x);
                self.transform_chain.drain(..index);
            }
        }
    }

    /// If timestamp is None, return the latest transform.
    pub(crate) fn get_closest_transform(
        &self,
        stamp: Option<Time>,
    ) -> Result<TransformStamped, TfError> {
        // TODO(lucasw) or just have a get_most_recent_transform()
        if stamp.is_none() || self.static_tf {
            println!("return latest");
            // TODO(lucasw) don't really want to use the timestamp of this if it is static
            return Ok(self.transform_chain.last().unwrap().clone());
        }

        let stamp = stamp.unwrap();
        let time = stamp_to_duration(stamp.clone());

        match binary_search_time(&self.transform_chain, time) {
            Ok(x) => return Ok(self.transform_chain.get(x).unwrap().clone()),
            Err(x) => {
                if x == 0 {
                    return Err(TfError::AttemptedLookupInPast(
                        stamp,
                        Box::new(self.transform_chain.first().unwrap().clone()),
                    ));
                }
                if x >= self.transform_chain.len() {
                    return Err(TfError::AttemptedLookUpInFuture(
                        Box::new(self.transform_chain.last().unwrap().clone()),
                        stamp,
                    ));
                }
                let tf1 = self.transform_chain.get(x - 1).unwrap().clone().transform;
                let tf2 = self.transform_chain.get(x).unwrap().clone().transform;
                let time1 = stamp_to_duration(self.transform_chain.get(x - 1).unwrap().header.stamp.clone());
                let time2 = stamp_to_duration(self.transform_chain.get(x).unwrap().header.stamp.clone());
                let header = self.transform_chain.get(x).unwrap().header.clone();
                let child_frame = self.transform_chain.get(x).unwrap().child_frame_id.clone();
                // interpolate between the timestamps that bracket the desired time
                let total_duration = (time2 - time1).num_milliseconds() as f64;
                let desired_duration = (time - time1).num_milliseconds() as f64;
                let weight = 1.0 - desired_duration / total_duration;
                let final_tf = interpolate(tf1, tf2, weight);
                let ros_msg = to_transform_stamped(final_tf, header.frame_id, child_frame, stamp);
                Ok(ros_msg)
            }
        }
    }

    pub(crate) fn has_valid_transform(&self, time: Option<TimeDelta>) -> bool {
        if self.transform_chain.is_empty() {
            return false;
        }

        if self.static_tf {
            return true;
        }

        match time {
            None => {
                return true;
            },
            Some(time) => {
                let first = self.transform_chain.first().unwrap();
                let last = self.transform_chain.last().unwrap();

                let first_header_stamp = stamp_to_duration(first.header.stamp.clone());
                let last_header_stamp = stamp_to_duration(last.header.stamp.clone());
                time >= first_header_stamp && time <= last_header_stamp
            },
        }
    }
}
