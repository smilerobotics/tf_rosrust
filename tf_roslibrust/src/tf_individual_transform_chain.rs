use chrono::TimeDelta;
use roslibrust_codegen::Time;
use std::collections::BTreeMap;

use crate::{
    tf_error::TfError,
    tf_util::stamp_to_duration,
    transforms::{geometry_msgs::TransformStamped, interpolate, to_transform_stamped},
};

#[derive(Clone, Debug)]
pub(crate) struct TfIndividualTransformChain {
    // TODO(lucasw) it looks like every individual transform has a cache duration,
    // so old transforms could hang around indefintely if no new ones are received with that
    // parent-child?
    cache_duration: TimeDelta,
    static_tf: bool,
    // TODO: Implement a circular buffer. Current method is slow.
    pub(crate) transform_chain: BTreeMap<TimeDelta, TransformStamped>,
}

impl TfIndividualTransformChain {
    pub(crate) fn new(static_tf: bool, cache_duration: TimeDelta) -> Self {
        // TODO(lucasw) have an enum type for static and non-static?
        Self {
            cache_duration,
            transform_chain: BTreeMap::new(),
            static_tf,
        }
    }

    fn newest_stamp(&self) -> Option<TimeDelta> {
        let key_value = self.transform_chain.last_key_value();
        match key_value {
            Some(key_value) => {
                let (key, _) = key_value;
                Some(key.clone())
            },
            None => None,
        }
    }

    // TODO(lucasw) pass in a TimeDelta which is latest - cache_duration across entire tf buffer
    // then won't have old transforms linger
    pub(crate) fn add_to_buffer(&mut self, msg: TransformStamped) {
        if self.static_tf {
            let mut tfs = msg.clone();
            // TODO(lucasw) tried to get rid of the magic 0, 0 static value but here it is again
            tfs.header.stamp.secs = 0;
            tfs.header.stamp.nsecs = 0;
            self.transform_chain.insert(stamp_to_duration(tfs.header.stamp), tfs);
            // TODO(lucasw) is there any way for other keys to get into this map, and need to clear
            // them out?
            return;
        }

        // insert the new new transform then check if first value is too old
        let latest_time = stamp_to_duration(msg.header.stamp);
        self.transform_chain.insert(latest_time, msg);

        // could do a while loop but should be a huge problem if too old values linger
        let (oldest_time, _) = self.transform_chain.first_key_value().unwrap();
        if (latest_time - *oldest_time) > self.cache_duration {
            // _ = self.transform_chain.remove(oldest_time);
            _ = self.transform_chain.pop_first();
        }
    }

    /// If timestamp is None, return the latest transform.
    pub(crate) fn get_closest_transform(
        &self,
        stamp: Option<Time>,
    ) -> Result<TransformStamped, TfError> {
        // TODO(lucasw) or just have a get_most_recent_transform()
        if stamp.is_none() || self.static_tf {
            // println!("return latest");
            // TODO(lucasw) don't really want to use the timestamp of this if it is static
            let key_value = self.transform_chain.first_key_value();
            match key_value {
                Some ((time, tf)) => {
                    return Ok(*tf);
                },
                // TODO(lucasw) probably this isn't possible, the check is already done?
                // None => return Err(TfError::CouldNotFindTransform()),
                None => { panic!("couldn't get static transform"); }
            }
        }

        let stamp = stamp.unwrap();
        let time = stamp_to_duration(stamp.clone());

        // the bound here result in not returning a two transforms if the key
        // is equal to the earliest time in the chain, but since it is on the verge
        // of getting removed that isn't bad.

        // This is unstable

        let header;
        let child_frame_id;
        let (time1, tf1);

        /*
        let cursor = self.transform_chain.upper_bound(std::ops::Bound::Included(&time));
        match cursor.peek_prev() {
            Some((time, tf)) => {
                time1 = time;
                tf1 = tf.transform;
                header = tf.header;
                child_frame_id = tf.child_frame_id;
            },
            None => {
                return Err(TfError::AttemptedLookupInPast(
                    stamp,
                    Box::new(self.transform_chain.first_key_value().unwrap().1.clone()),
                ));
            },
        }

        let (time2, tf2);
        match cursor.peek_next() {
            Some((time, tf)) => {
                time2 = time;
                tf2 = tf.transform;
            },
            None => {
                return Err(TfError::AttemptedLookUpInFuture(
                    Box::new(self.transform_chain.last_key_value().unwrap().1.clone()),
                    stamp,
                ));
            },
        }
        */
        {
            let keys = self.transform_chain.keys().copied().collect();
            if time <= keys[0] {
                return Err(TfError::AttemptedLookupInPast(
                    stamp,
                    Box::new(self.transform_chain.first_key_value().unwrap().1.clone()),
                ));
            } else if time > keys.last()] {
                return Err(TfError::AttemptedLookUpInFuture(
                    Box::new(self.transform_chain.last_key_value().unwrap().1.clone()),
                    stamp,
                ));
            }

            let ind = keys.partition_point(|&x| x < time);
        }

        // interpolate between the timestamps that bracket the desired time
        let total_duration = (*time2 - *time1).num_milliseconds() as f64;
        let desired_duration = (time - *time1).num_milliseconds() as f64;
        let weight = 1.0 - desired_duration / total_duration;
        let final_tf = interpolate(tf1, tf2, weight);
        let ros_msg = to_transform_stamped(final_tf, header.frame_id, child_frame_id, stamp);
        Ok(ros_msg)
    }

    // TODO(lucasw) not currently using this
    /*
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
    */
}
