use std::collections::{hash_map::Entry, HashMap, HashSet};

use chrono::TimeDelta;
use roslibrust_codegen::Time;

use crate::{
    tf_error::TfError,
    tf_graph_node::TfGraphNode,
    tf_individual_transform_chain::TfIndividualTransformChain,
    tf_util::{stamp_to_duration, to_stamp},
    transforms::{
        chain_transforms, geometry_msgs::TransformStamped, get_inverse, std_msgs::Header,
        tf2_msgs::TFMessage, to_transform_stamped,
    },
};

#[derive(Clone, Debug)]
pub struct TfBuffer {
    parent_transform_index: HashMap<String, String>,
    child_transform_index: HashMap<String, HashSet<String>>,
    transform_data: HashMap<TfGraphNode, TfIndividualTransformChain>,
    cache_duration: TimeDelta,
}

impl Default for TfBuffer {
    fn default() -> Self {
        Self::new()
    }
}

const DEFAULT_CACHE_DURATION_SECONDS: u16 = 10;

impl TfBuffer {
    pub fn new() -> Self {
        Self::new_with_duration(TimeDelta::new(DEFAULT_CACHE_DURATION_SECONDS.into(), 0).unwrap())
    }

    pub fn new_with_duration(cache_duration: TimeDelta) -> Self {
        TfBuffer {
            parent_transform_index: HashMap::new(),
            child_transform_index: HashMap::new(),
            transform_data: HashMap::new(),
            cache_duration,
        }
    }

    pub fn handle_incoming_transforms(
        &mut self,
        transforms: TFMessage,
        is_static: bool,
    ) -> Result<(), TfError> {
        for transform in transforms.transforms {
            self.add_transform(&transform, is_static)?;
        }
        Ok(())
    }

    // don't detect loops here on the assumption that the transforms are arriving
    // much faster than lookup are occuring, so only detect loops in a lookup
    pub fn add_transform(
        &mut self,
        transform: &TransformStamped,
        is_static_tf: bool,
    ) -> Result<(), TfError> {
        // TODO(lucasw) need to retire transforms from this index when they expire
        // TODO(lucasw) if this child has a different parent should error or warn
        let parent = &transform.header.frame_id;
        let child = &transform.child_frame_id;

        let existing_parent = self.parent_transform_index.get(child);
        match existing_parent {
            None => {
                self.parent_transform_index
                    .insert(child.clone(), parent.clone());
            }
            Some(existing_parent) => {
                // TODO(lucasw) will this comparison slow everything down much when there are lots
                // of transforms?
                if *existing_parent != *parent {
                    return Err(TfError::ChangingParent(
                        child.clone(),
                        parent.clone(),
                        existing_parent.clone(),
                    ));
                }
            }
        }

        self.child_transform_index
            .entry(parent.clone())
            .or_default()
            .insert(child.clone());

        let key = TfGraphNode {
            child: child.clone(),
            parent: parent.clone(),
        };

        // TODO(lucasw) return error if the existing chain static != is_static_tf
        match self.transform_data.entry(key) {
            Entry::Occupied(e) => e.into_mut(),
            Entry::Vacant(e) => e.insert(TfIndividualTransformChain::new(
                parent.clone(),
                child.clone(),
                is_static_tf,
                self.cache_duration,
            )),
        }
        .add_to_buffer(transform.clone());

        Ok(())
    }

    pub fn get_parent_to_children(&self) -> HashMap<String, HashSet<String>> {
        self.child_transform_index.clone()
    }

    /// traverse tf tree straight upwards until there are no more parents
    fn get_path_to_root(&self, frame: &str) -> Result<(Vec<String>, HashSet<String>), TfError> {
        let frame = frame.to_string();
        // TODO(lucasw) use an IndexMap
        let mut frame_lineage = Vec::new();
        let mut frame_lineage_visited = HashSet::<String>::new();
        frame_lineage_visited.insert(frame.clone().to_string());
        frame_lineage.push(frame.clone());

        let mut cur_frame = frame.clone();
        // println!("get frame lineage '{frame}'");

        // TODO(lucasw) could this be done in one line?
        while self.parent_transform_index.contains_key(&cur_frame) {
            // println!("-- {cur_frame}");
            cur_frame = self
                .parent_transform_index
                .get(&cur_frame)
                .unwrap()
                .to_string();
            // println!("  |-> {cur_frame}");
            // detect loops
            // TODO(lucasw) much more advanced loop detection would allow loops that don't overlap
            // in time, but it's much easier to prohibit those (which means loops can exist
            // that don't overlap within cache duration of each other)
            if frame_lineage_visited.contains(&cur_frame) {
                return Err(TfError::LoopDetected(
                    frame.to_string(),
                    self.child_transform_index.clone(),
                ));
            }
            frame_lineage_visited.insert(cur_frame.clone());
            frame_lineage.push(cur_frame.clone());
        }
        Ok((frame_lineage, frame_lineage_visited))
    }

    pub fn get_root(&self) -> Result<String, TfError> {
        let parent = self
            .child_transform_index
            .keys()
            .next()
            .ok_or(TfError::EmptyTree)?;
        let (lineage, _) = self.get_path_to_root(parent)?;
        lineage.last().cloned().ok_or(TfError::EmptyTree)
    }

    /// Retrieves the transform path
    fn retrieve_transform_path(&self, from: &str, to: &str) -> Result<Vec<String>, TfError> {
        // for (key, value) in &self.parent_transform_index {
        //     println!("{key}: {value}");
        // }
        // return Err(TfError::CouldNotFindTransform(from, to, self.child_transform_index.clone()));

        // Find the common parent of from and to, get the path all the way to root of each
        // then iterate through one to find find the earliest frame that is also in the other path
        let (from_lineage, from_lineage_visited) = self.get_path_to_root(from)?;
        // println!("from lineage {from_lineage:?}");
        let (to_lineage, _) = self.get_path_to_root(to)?;
        // println!("to lineage {to_lineage:?}");

        // TODO(lucasw) could try to terminate above get_path_to_root by looking for the common frame there
        // make it take an optional frame to stop at, if None go to root
        let common_parent = {
            let mut common_parent = None;
            for frame in &to_lineage {
                if from_lineage_visited.contains(frame) {
                    common_parent = Some(frame);
                    // println!("common parent found: {frame}");
                    break;
                }
            }

            if common_parent.is_none() {
                // TODO(lucasw) could also delineate where one transform or the other isn't in the tree
                // at all
                return Err(TfError::CouldNotFindTransform(
                    from.to_string(),
                    to.to_string(),
                    "disconnected trees, no common parent".to_string(),
                ));
            }
            common_parent.unwrap()
        };

        // Now join the path up from 'from' and path down to 'to' together
        // TODO(lucasw) return two separate paths so the caller can apply
        // an inverse transform to the 'up' path instead of the stored forward transform
        let mut res = vec![];

        for frame in &from_lineage {
            // println!("frame in 'from' lineage {frame}");
            res.push(frame.clone());
            if frame == common_parent {
                break;
            }
        }

        {
            let mut found_common_parent = false;
            for frame in (to_lineage).iter().rev() {
                // println!("frame in 'to' lineage {frame}");
                if found_common_parent {
                    res.push(frame.clone());
                }
                if frame == common_parent {
                    found_common_parent = true;
                }
            }
        }

        Ok(res)
    }

    // get tf chain from one frame to another
    fn get_tf_list(
        &self,
        from: &str,
        to: &str,
        stamp: Option<Time>,
    ) -> Result<Vec<TransformStamped>, TfError> {
        let mut tf_list = Vec::<TransformStamped>::new();
        let path0 = self.retrieve_transform_path(from, to)?;
        // TODO(lucasw) how to iterate through path0 to get adjacent elements like
        // this?
        for ind in 0..path0.len() - 1 {
            let frame0 = path0[ind].clone();
            let frame1 = path0[ind + 1].clone();
            let node = TfGraphNode {
                child: frame1.clone(),
                parent: frame0.clone(),
            };
            let inverse_node = TfGraphNode {
                child: frame0.clone(),
                parent: frame1.clone(),
            };

            // get inverse of inverse_node if this fails
            // TODO(lucasw) it would be better if the path returned had the information on which
            // direction the transforms need to go
            let mut invert_transform = false;
            let time_cache = {
                let rv = self.transform_data.get(&node);
                match rv {
                    Some(time_cache) => time_cache,
                    None => {
                        invert_transform = true;
                        self.transform_data
                            .get(&inverse_node)
                            .unwrap_or_else(|| panic!("{inverse_node:?}"))
                    }
                }
            };

            // TODO(lucasw) this doesn't get a coherent set of transforms when
            // wanting the most recent- need to find the earliest and latest
            // transform for each of these and get the overlap of all of them,
            // then use the latest of those, then use that as the header stamp
            // for the result as well (don't use time 0)
            let transform = {
                let transform = time_cache.get_closest_transform(stamp.clone())?;
                if invert_transform {
                    get_inverse(&transform)
                } else {
                    transform
                }
            };
            tf_list.push(transform);
        }
        Ok(tf_list)
    }

    /// Looks up a transform within the tree at a given time.
    pub fn lookup_transform(
        &self,
        from: &str,
        to: &str,
        stamp0: Option<Time>,
    ) -> Result<TransformStamped, TfError> {
        if from == to {
            // TODO(lucasw) use default syntax
            let mut tfs = TransformStamped::default();
            tfs.header.frame_id = from.to_string();
            tfs.child_frame_id = to.to_string();
            match stamp0 {
                Some(stamp) => {
                    tfs.header.stamp = stamp;
                }
                // TODO(lucasw) is time zero correct?
                None => {
                    tfs.header.stamp = to_stamp(0, 0);
                }
            }
            tfs.transform.rotation.w = 1.0;
            return Ok(tfs);
        }

        let stamp = {
            match stamp0 {
                Some(stamp) => Some(stamp),
                None => {
                    // println!("getting most recent transform");
                    let tf_list = self.get_tf_list(from, to, stamp0)?;
                    // search for time that is before or equal to every transform in the chain,
                    // unless the frame was static, then ignore the timestamp
                    let min_time = {
                        let mut min_time = None;
                        for tf in tf_list {
                            let tf_time = stamp_to_duration(&tf.header.stamp);
                            // println!("{0} tf time {tf_time:?}", tf.child_frame_id);

                            // TODO(lucasw) wouldn't need this is_zero if the tf list contents
                            // could have a bool to show whether transform was static
                            if !tf_time.is_zero() {
                                if min_time.is_none() {
                                    // TODO(lucasw) if the tf transform is static
                                    // is this correct- does it return the asked for time
                                    // instead of whatever the original static frame was
                                    min_time = Some(tf_time);
                                } else {
                                    min_time = Some(std::cmp::min(tf_time, min_time.unwrap()));
                                }
                            }
                        }
                        // if min_time is still None after this the chain was pure static,
                        // then the later get_tf_list still gets a good chain
                        min_time
                    };

                    // TODO(lucasw) will a static transform return a bad stamp?
                    match min_time {
                        Some(min_time) => {
                            let stamp = to_stamp(
                                min_time.num_seconds() as u32,
                                min_time.subsec_nanos() as u32,
                            );
                            // println!("most recent stamp {stamp:?} {min_time:?}");
                            Some(stamp)
                        }
                        None => None,
                    }
                } // get most recent stamp
            }
        };

        // println!("getting transform at stamp {stamp:?}");
        // TODO(lucasw) str vs. String here- can it all be made consistent?
        let tf_list = self.get_tf_list(from, to, stamp.clone())?;
        let final_tf = chain_transforms(&tf_list);
        let msg = TransformStamped {
            child_frame_id: to.to_string(),
            header: Header {
                frame_id: from.to_string(),
                stamp: {
                    match stamp {
                        Some(stamp) => stamp,
                        // the chain was pure static and there are no meaningful values
                        // TODO(lucasw) does this match rospy/roscpp behavior?
                        None => to_stamp(0, 0),
                    }
                },
                seq: 1,
            },
            transform: final_tf,
        };
        Ok(msg)
    }

    pub(crate) fn lookup_transform_with_time_travel(
        &self,
        to: &str,
        time2: Time,
        from: &str,
        time1: Time,
        fixed_frame: &str,
    ) -> Result<TransformStamped, TfError> {
        let tf1 = self.lookup_transform(from, fixed_frame, Some(time1.clone()))?;
        let tf2 = self.lookup_transform(to, fixed_frame, Some(time2))?;
        let transforms = get_inverse(&tf1);
        let result = chain_transforms(&[tf2, transforms]);
        Ok(to_transform_stamped(
            result,
            from.to_string(),
            to.to_string(),
            // TODO(lucasw) does using this stamp as opposed to time2 match roscpp/rospy?
            time1,
        ))
    }
}

#[cfg(test)]
mod test {
    use Time;

    use super::*;
    use crate::transforms::geometry_msgs::{Quaternion, Transform, Vector3};

    const PARENT: &str = "parent";
    const CHILD0: &str = "child0";
    const CHILD1: &str = "child1";

    /// This function builds a tree consisting of the following items:
    /// * a world coordinate frame
    /// * an item in the world frame at (1,0,0)
    /// * base_link of a robot starting at (0,0,0) and progressing at (0,t,0) where t is time in seconds
    /// * a camera which is (0.5, 0, 0) from the base_link
    fn build_test_tree(buffer: &mut TfBuffer, time: f64) {
        let nsecs = ((time - ((time.floor() as i64) as f64)) * 1E9) as u32;

        let world_to_item = TransformStamped {
            child_frame_id: "item".to_string(),
            header: Header {
                frame_id: "world".to_string(),
                stamp: to_stamp(time.floor() as u32, nsecs),
                seq: 1,
            },
            transform: Transform {
                rotation: Quaternion {
                    x: 0f64,
                    y: 0f64,
                    z: 0f64,
                    w: 1f64,
                },
                translation: Vector3 {
                    x: 1f64,
                    y: 0f64,
                    z: 0f64,
                },
            },
        };
        let _ = buffer.add_transform(&world_to_item, true);
        // buffer.add_transform(&get_inverse(&world_to_item), true);

        let world_to_base_link = TransformStamped {
            child_frame_id: "base_link".to_string(),
            header: Header {
                frame_id: "world".to_string(),
                stamp: Time {
                    secs: time.floor() as u32,
                    nsecs: nsecs,
                },
                seq: 1,
            },
            transform: Transform {
                rotation: Quaternion {
                    x: 0f64,
                    y: 0f64,
                    z: 0f64,
                    w: 1f64,
                },
                translation: Vector3 {
                    x: 0f64,
                    y: time,
                    z: 0f64,
                },
            },
        };
        let _ = buffer.add_transform(&world_to_base_link, false);
        // buffer.add_transform(&get_inverse(&world_to_base_link), false);

        let base_link_to_camera = TransformStamped {
            child_frame_id: "camera".to_string(),
            header: Header {
                frame_id: "base_link".to_string(),
                stamp: Time {
                    secs: time.floor() as u32,
                    nsecs: nsecs,
                },
                seq: 1,
            },
            transform: Transform {
                rotation: Quaternion {
                    x: 0f64,
                    y: 0f64,
                    z: 0f64,
                    w: 1f64,
                },
                translation: Vector3 {
                    x: 0.5f64,
                    y: 0f64,
                    z: 0f64,
                },
            },
        };
        let _ = buffer.add_transform(&base_link_to_camera, true);
        // buffer.add_transform(&get_inverse(&base_link_to_camera), true);
    }

    /// Tests a basic lookup
    #[test]
    fn test_basic_tf_lookup() {
        let mut tf_buffer = TfBuffer::new();
        build_test_tree(&mut tf_buffer, 0f64);
        let res = tf_buffer.lookup_transform("camera", "item", None);
        let expected = TransformStamped {
            child_frame_id: "item".to_string(),
            header: Header {
                frame_id: "camera".to_string(),
                stamp: Time { secs: 0, nsecs: 0 },
                seq: 1,
            },
            transform: Transform {
                rotation: Quaternion {
                    x: 0f64,
                    y: 0f64,
                    z: 0f64,
                    w: 1f64,
                },
                translation: Vector3 {
                    x: 0.5f64,
                    y: 0f64,
                    z: 0f64,
                },
            },
        };
        assert_eq!(res.unwrap(), expected);
    }

    /// Tests an interpolated lookup.
    #[test]
    fn test_basic_tf_interpolation() {
        let mut tf_buffer = TfBuffer::new();
        build_test_tree(&mut tf_buffer, 0f64);
        build_test_tree(&mut tf_buffer, 1f64);
        let res = tf_buffer.lookup_transform(
            "camera",
            "item",
            Some(Time {
                secs: 0,
                nsecs: 700_000_000,
            }),
        );
        let expected = TransformStamped {
            child_frame_id: "item".to_string(),
            header: Header {
                frame_id: "camera".to_string(),
                stamp: Time {
                    secs: 0,
                    nsecs: 700_000_000,
                },
                seq: 1,
            },
            transform: Transform {
                rotation: Quaternion {
                    x: 0f64,
                    y: 0f64,
                    z: 0f64,
                    w: 1f64,
                },
                translation: Vector3 {
                    x: 0.5f64,
                    y: -0.7f64,
                    z: 0f64,
                },
            },
        };
        assert_eq!(res.unwrap(), expected);
    }

    /// Tests an interpolated lookup.
    #[test]
    fn test_basic_tf_time_travel() {
        let mut tf_buffer = TfBuffer::new();
        build_test_tree(&mut tf_buffer, 0f64);
        build_test_tree(&mut tf_buffer, 1f64);
        let res = tf_buffer.lookup_transform_with_time_travel(
            "camera",
            Time {
                secs: 0,
                nsecs: 400_000_000,
            },
            "camera",
            Time {
                secs: 0,
                nsecs: 700_000_000,
            },
            "item",
        );
        let expected = TransformStamped {
            child_frame_id: "camera".to_string(),
            header: Header {
                frame_id: "camera".to_string(),
                stamp: Time {
                    secs: 0,
                    nsecs: 700_000_000,
                },
                seq: 1,
            },
            transform: Transform {
                rotation: Quaternion {
                    x: 0f64,
                    y: 0f64,
                    z: 0f64,
                    w: 1f64,
                },
                translation: Vector3 {
                    x: 0f64,
                    y: 0.3f64,
                    z: 0f64,
                },
            },
        };
        assert_approx_eq(res.unwrap(), expected);
    }

    #[test]
    fn test_add_transform() {
        let mut tf_buffer = TfBuffer::new();

        let transform00 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                // TODO(lucasw) can't use 0, 0 as a normal stamp because it's still special to
                // denote static, need tof fix that
                stamp: Time { secs: 1, nsecs: 0 },
                ..Default::default()
            },
            child_frame_id: CHILD0.to_string(),
            ..Default::default()
        };
        let transform0_key = TfGraphNode {
            parent: PARENT.to_owned(),
            child: CHILD0.to_owned(),
        };

        let transform01 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                stamp: Time { secs: 2, nsecs: 0 },
                ..Default::default()
            },
            child_frame_id: CHILD0.to_string(),
            ..Default::default()
        };

        let transform10 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                stamp: Time { secs: 3, nsecs: 0 },
                ..Default::default()
            },
            child_frame_id: CHILD1.to_string(),
            ..Default::default()
        };
        let transform11 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                stamp: Time { secs: 4, nsecs: 0 },
                ..Default::default()
            },
            child_frame_id: CHILD1.to_string(),
            ..Default::default()
        };

        let transform1_key = TfGraphNode {
            parent: PARENT.to_owned(),
            child: CHILD1.to_owned(),
        };

        let is_static = false;
        assert!(tf_buffer.add_transform(&transform00, is_static).is_ok());
        assert_eq!(tf_buffer.child_transform_index.len(), 1);
        assert!(tf_buffer.child_transform_index.contains_key(PARENT));

        let children = tf_buffer.child_transform_index.get(PARENT).unwrap();
        assert_eq!(children.len(), 1);
        assert!(children.contains(CHILD0));
        assert_eq!(tf_buffer.transform_data.len(), 1);
        assert!(tf_buffer.transform_data.contains_key(&transform0_key));

        let data = tf_buffer.transform_data.get(&transform0_key);
        assert!(data.is_some());
        assert_eq!(data.unwrap().transform_chain.len(), 1);

        assert!(tf_buffer.add_transform(&transform01, is_static).is_ok());
        assert_eq!(tf_buffer.child_transform_index.len(), 1);
        assert!(tf_buffer.child_transform_index.contains_key(PARENT));

        let children = tf_buffer.child_transform_index.get(PARENT).unwrap();
        assert_eq!(children.len(), 1);
        assert!(children.contains(CHILD0));
        assert_eq!(tf_buffer.transform_data.len(), 1);
        assert!(tf_buffer.transform_data.contains_key(&transform0_key));

        let data = tf_buffer.transform_data.get(&transform0_key);
        assert!(data.is_some());
        // dbg!(&data);
        assert_eq!(data.unwrap().transform_chain.len(), 2);

        let is_static = true;
        // a static transform chain will always be length 0, the latest overwrites the earlier one
        // (regardless of timestamp in the transform, it's set to zero internally anyhow)
        assert!(tf_buffer.add_transform(&transform10, is_static).is_ok());
        assert!(tf_buffer.add_transform(&transform11, is_static).is_ok());
        assert_eq!(tf_buffer.child_transform_index.len(), 1);
        assert!(tf_buffer.child_transform_index.contains_key(PARENT));

        let children = tf_buffer.child_transform_index.get(PARENT).unwrap();
        assert_eq!(children.len(), 2);
        assert!(children.contains(CHILD0));
        assert!(children.contains(CHILD1));
        assert_eq!(tf_buffer.transform_data.len(), 2);
        assert!(tf_buffer.transform_data.contains_key(&transform0_key));
        assert!(tf_buffer.transform_data.contains_key(&transform1_key));

        let data = tf_buffer.transform_data.get(&transform0_key);
        assert!(data.is_some());
        assert_eq!(data.unwrap().transform_chain.len(), 2);

        let data = tf_buffer.transform_data.get(&transform1_key);
        assert!(data.is_some());
        assert_eq!(data.unwrap().transform_chain.len(), 1);
    }

    #[test]
    fn test_cache_duration() {
        let mut tf_buffer = TfBuffer::new_with_duration(TimeDelta::new(1, 0).unwrap());
        let transform00 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                stamp: Time { secs: 1, nsecs: 0 },
                ..Default::default()
            },
            child_frame_id: CHILD0.to_string(),
            ..Default::default()
        };
        let transform01 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                stamp: Time { secs: 2, nsecs: 0 },
                ..Default::default()
            },
            child_frame_id: CHILD0.to_string(),
            ..Default::default()
        };
        let transform02 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                stamp: Time { secs: 3, nsecs: 0 },
                ..Default::default()
            },
            child_frame_id: CHILD0.to_string(),
            ..Default::default()
        };
        let transform0_key = TfGraphNode {
            child: CHILD0.to_owned(),
            parent: PARENT.to_owned(),
        };

        let is_static = false;
        assert!(tf_buffer.add_transform(&transform00, is_static).is_ok());
        assert_eq!(tf_buffer.child_transform_index.len(), 1);
        assert_eq!(tf_buffer.transform_data.len(), 1);
        assert!(tf_buffer.transform_data.contains_key(&transform0_key));
        let data = tf_buffer.transform_data.get(&transform0_key);
        assert!(data.is_some());
        assert_eq!(data.unwrap().transform_chain.len(), 1);
        assert_eq!(
            data.unwrap()
                .transform_chain
                .first_key_value()
                .unwrap()
                .1
                .header
                .stamp,
            to_stamp(1, 0),
        );

        assert!(tf_buffer.add_transform(&transform01, is_static).is_ok());
        assert_eq!(tf_buffer.child_transform_index.len(), 1);
        assert_eq!(tf_buffer.transform_data.len(), 1);
        assert!(tf_buffer.transform_data.contains_key(&transform0_key));
        let data = tf_buffer.transform_data.get(&transform0_key);
        assert!(data.is_some());
        assert_eq!(data.unwrap().transform_chain.len(), 2);
        assert_eq!(
            data.unwrap()
                .transform_chain
                .first_key_value()
                .unwrap()
                .1
                .header
                .stamp,
            to_stamp(1, 0),
        );
        let keys: Vec<TimeDelta> = data.unwrap().transform_chain.keys().copied().collect();
        assert_eq!(
            data.unwrap().transform_chain[&keys[1]].header.stamp,
            to_stamp(2, 0),
        );

        assert!(tf_buffer.add_transform(&transform02, is_static).is_ok());
        assert_eq!(tf_buffer.child_transform_index.len(), 1);
        assert_eq!(tf_buffer.transform_data.len(), 1);
        assert!(tf_buffer.transform_data.contains_key(&transform0_key));
        let data = tf_buffer.transform_data.get(&transform0_key);
        assert!(data.is_some());
        assert_eq!(data.unwrap().transform_chain.len(), 2);
        assert_eq!(
            data.unwrap()
                .transform_chain
                .first_key_value()
                .unwrap()
                .1
                .header
                .stamp,
            to_stamp(2, 0),
        );
        let keys: Vec<TimeDelta> = data.unwrap().transform_chain.keys().copied().collect();
        assert_eq!(
            data.unwrap().transform_chain[&keys[1]].header.stamp,
            to_stamp(3, 0),
        );
    }

    fn assert_approx_eq(msg1: TransformStamped, msg2: TransformStamped) {
        assert_eq!(msg1.header, msg2.header);
        assert_eq!(msg1.child_frame_id, msg2.child_frame_id);

        assert!((msg1.transform.rotation.x - msg2.transform.rotation.x).abs() < 1e-9);
        assert!((msg1.transform.rotation.y - msg2.transform.rotation.y).abs() < 1e-9);
        assert!((msg1.transform.rotation.z - msg2.transform.rotation.z).abs() < 1e-9);
        assert!((msg1.transform.rotation.w - msg2.transform.rotation.w).abs() < 1e-9);

        assert!((msg1.transform.translation.x - msg2.transform.translation.x).abs() < 1e-9);
        assert!((msg1.transform.translation.y - msg2.transform.translation.y).abs() < 1e-9);
        assert!((msg1.transform.translation.z - msg2.transform.translation.z).abs() < 1e-9);
    }

    /// TODO(lucasw) this reparenting isn't supported currently, have to wait for old
    /// transforms to fall out of buffer
    /// TODO(lucasw) update- I've made this an error
    /// Tests a case in which the tree structure changes dynamically
    /// time 1-2(sec): [base] -> [camera1] -> [marker] -> [target]
    /// time 3-4(sec): [base] -> [camera2] -> [marker] -> [target]
    /// time 5-6(sec): [base] -> [camera1] -> [marker] -> [target]
    #[test]
    fn test_dynamic_tree() {
        let mut tf_buffer = TfBuffer::new();

        let base_to_camera1 = TransformStamped {
            header: Header {
                frame_id: "base".to_string(),
                stamp: Time { secs: 1, nsecs: 0 },
                seq: 1,
            },
            child_frame_id: "camera1".to_string(),
            transform: Transform {
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                translation: Vector3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
            },
        };
        assert!(tf_buffer.add_transform(&base_to_camera1, true).is_ok());

        let base_to_camera2 = TransformStamped {
            header: Header {
                frame_id: "base".to_string(),
                stamp: Time { secs: 1, nsecs: 0 },
                seq: 1,
            },
            child_frame_id: "camera2".to_string(),
            transform: Transform {
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 1.0,
                    w: 0.0,
                },
                translation: Vector3 {
                    x: -1.0,
                    y: 0.0,
                    z: 0.0,
                },
            },
        };
        assert!(tf_buffer.add_transform(&base_to_camera2, true).is_ok());

        let marker_to_target = TransformStamped {
            header: Header {
                frame_id: "marker".to_string(),
                stamp: Time { secs: 1, nsecs: 0 },
                seq: 1,
            },
            child_frame_id: "target".to_string(),
            transform: Transform {
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                translation: Vector3 {
                    x: -0.5,
                    y: 0.0,
                    z: 0.0,
                },
            },
        };
        assert!(tf_buffer.add_transform(&marker_to_target, true).is_ok());

        let mut camera1_to_marker = TransformStamped {
            header: Header {
                frame_id: "camera1".to_string(),
                stamp: Time { secs: 4, nsecs: 0 },
                seq: 1,
            },
            child_frame_id: "marker".to_string(),
            transform: Transform {
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                translation: Vector3 {
                    x: 1.0,
                    y: 1.0,
                    z: 0.0,
                },
            },
        };
        assert!(tf_buffer.add_transform(&camera1_to_marker, false).is_ok());

        camera1_to_marker.header.stamp.secs = 30;
        camera1_to_marker.header.seq += 1;
        camera1_to_marker.transform.translation.y = -1.0;
        assert!(tf_buffer.add_transform(&camera1_to_marker, false).is_ok());

        let mut camera2_to_marker = TransformStamped {
            header: Header {
                frame_id: "camera2".to_string(),
                stamp: Time { secs: 30, nsecs: 0 },
                seq: 1,
            },
            child_frame_id: "marker".to_string(),
            transform: Transform {
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                translation: Vector3 {
                    x: 1.0,
                    y: 10.0,
                    z: 0.0,
                },
            },
        };
        assert!(tf_buffer.add_transform(&camera2_to_marker, false).is_err());

        camera2_to_marker.header.stamp.secs = 40;
        camera2_to_marker.header.seq += 1;
        camera2_to_marker.transform.translation.y = -10.0;
        assert!(tf_buffer.add_transform(&camera2_to_marker, false).is_err());

        // TODO(lucasw) there is a rotation here that flips the y direction
        let rv = tf_buffer.lookup_transform("base", "target", Some(Time { secs: 37, nsecs: 0 }));
        //.unwrap();
        // assert!((tf.transform.translation.y - -1.0).abs() < 0.01, "base -> target {:?}", tf.transform.translation);
        // TODO(lucasw)
        match rv {
            // this is what is expected
            Err(TfError::AttemptedLookupInFuture(_, _, _)) => assert!(true),
            _ => assert!(false),
        }

        let tf = tf_buffer
            .lookup_transform("camera1", "marker", Some(Time { secs: 30, nsecs: 0 }))
            .unwrap();
        assert!(
            (tf.transform.translation.y - -1.0).abs() < 0.01,
            "{:?}",
            tf.transform.translation
        );

        // TODO(lucasw) update these lookups to be valid
        /*
        let result = tf_buffer.lookup_transform(
            "base",
            "target",
            Some(to_stamp(37, 0)),
        );
        assert_eq!(
            result.unwrap().transform.translation,
            Vector3 {
                x: 1.5,
                y: 0.0,
                z: 0.0
            }
        );

        let result =
            tf_buffer.lookup_transform("base", "target", Some(Time { secs: 2, nsecs: 0 }));
        assert_eq!(
            result.unwrap().transform.translation,
            Vector3 {
                x: 1.5,
                y: -1.0,
                z: 0.0
            }
        );
        */

        let result = tf_buffer.lookup_transform(
            "base",
            "target",
            Some(Time {
                secs: 2,
                nsecs: 500_000_000,
            }),
        );
        assert!(result.is_err());

        /*
        let result =
            tf_buffer.lookup_transform("base", "target", Some(Time { secs: 3, nsecs: 0 }));
        assert_eq!(
            result.unwrap().transform.translation,
            Vector3 {
                x: -1.5,
                y: -1.0,
                z: 0.0
            }
        );

        let result = tf_buffer.lookup_transform(
            "base",
            "target",
            Some(Time {
                secs: 3,
                nsecs: 500_000_000,
            }),
        );
        assert_eq!(
            result.unwrap().transform.translation,
            Vector3 {
                x: -1.5,
                y: -0.0,
                z: 0.0
            }
        );

        let result =
            tf_buffer.lookup_transform("base", "target", Some(Time { secs: 4, nsecs: 0 }));
        assert_eq!(
            result.unwrap().transform.translation,
            Vector3 {
                x: -1.5,
                y: 1.0,
                z: 0.0
            }
        );
        */

        let result = tf_buffer.lookup_transform(
            "base",
            "target",
            Some(Time {
                secs: 4,
                nsecs: 500_000_000,
            }),
        );
        assert!(result.is_err());

        camera1_to_marker.header.stamp.secs = 50;
        camera1_to_marker.header.seq += 1;
        camera1_to_marker.transform.translation.x = 0.5;
        camera1_to_marker.transform.translation.y = 1.0;
        assert!(tf_buffer.add_transform(&camera1_to_marker, false).is_ok());

        camera1_to_marker.header.stamp.secs += 10;
        camera1_to_marker.header.seq += 1;
        camera1_to_marker.transform.translation.y = -1.0;
        assert!(tf_buffer.add_transform(&camera1_to_marker, false).is_ok());

        let result =
            tf_buffer.lookup_transform("base", "target", Some(Time { secs: 55, nsecs: 0 }));
        assert_eq!(
            result.unwrap().transform.translation,
            Vector3 {
                x: 1.0,
                y: 0.0,
                z: 0.0
            }
        );

        let result = tf_buffer.lookup_transform(
            "base",
            "target",
            Some(Time {
                secs: 59,
                nsecs: 500_000_000,
            }),
        );
        let y = result.unwrap().transform.translation.y;
        assert!(y - -0.9 < 0.001, "y: {y}");

        let result =
            tf_buffer.lookup_transform("base", "target", Some(Time { secs: 51, nsecs: 0 }));
        assert_eq!(
            result.unwrap().transform.translation,
            Vector3 {
                x: 1.0,
                y: 0.8,
                z: 0.0
            }
        );
    }

    #[test]
    fn test_changing_parents() {
        let mut tf_buffer = TfBuffer::new_with_duration(TimeDelta::new(200, 0).unwrap());

        let translation = Vector3 {
            x: 1.0,
            y: -0.7,
            z: 3.0,
        };

        let stamp = crate::tf_util::stamp_now();
        let tfs = TransformStamped {
            header: Header {
                frame_id: "base".to_string(),
                stamp: stamp.clone(),
                seq: 1,
            },
            child_frame_id: "leaf".to_string(),
            transform: Transform {
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                translation: translation.clone(),
            },
        };

        let is_static = true;
        let rv = tf_buffer.add_transform(&tfs, is_static);
        assert!(rv.is_ok());

        let tfs = TransformStamped {
            header: Header {
                frame_id: "other_base".to_string(),
                stamp: stamp.clone(),
                seq: 1,
            },
            child_frame_id: "leaf".to_string(),
            transform: Transform {
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                translation: translation.clone(),
            },
        };

        let rv = tf_buffer.add_transform(&tfs, is_static);
        assert!(rv.is_err());
    }

    #[test]
    fn test_long_buffer() {
        let mut tf_buffer = TfBuffer::new_with_duration(TimeDelta::new(200, 0).unwrap());

        let translation = Vector3 {
            x: 1.0,
            y: -0.7,
            z: 3.0,
        };

        let stamp = crate::tf_util::stamp_now();
        let tfs = TransformStamped {
            header: Header {
                frame_id: "base".to_string(),
                stamp: stamp.clone(),
                seq: 1,
            },
            child_frame_id: "leaf".to_string(),
            transform: Transform {
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                translation: translation.clone(),
            },
        };

        let is_static = true;
        assert!(tf_buffer.add_transform(&tfs, is_static).is_ok());

        let result = tf_buffer.lookup_transform("base", "leaf", Some(Time { secs: 51, nsecs: 0 }));
        assert_eq!(result.unwrap().transform.translation, translation);
        let result = tf_buffer.lookup_transform(
            "base",
            "leaf",
            Some(Time {
                secs: stamp.secs,
                nsecs: 0,
            }),
        );
        assert_eq!(result.unwrap().transform.translation, translation);
        let result = tf_buffer.lookup_transform(
            "base",
            "leaf",
            Some(Time {
                secs: stamp.secs + 1000,
                nsecs: 0,
            }),
        );
        assert_eq!(result.unwrap().transform.translation, translation);
    }

    #[test]
    fn test_long_dynamic_buffer() {
        let dt = 0.1;
        let num_secs = 200;
        let mut tf_buffer = TfBuffer::new_with_duration(TimeDelta::new(num_secs, 0).unwrap());

        let translation = Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };

        let stamp = Time {
            secs: 1_002_003_000,
            nsecs: 0,
        };
        let mut tfs = TransformStamped {
            header: Header {
                frame_id: "base".to_string(),
                stamp: stamp.clone(),
                seq: 1,
            },
            child_frame_id: "leaf".to_string(),
            transform: Transform {
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                translation: translation.clone(),
            },
        };

        use crate::tf_util;

        let num_steps = (num_secs as f64 / dt) as u32;
        for i in 0..num_steps {
            // set a time and position offset to the same value
            let offset = i as f64 * dt;
            tfs.header.stamp = tf_util::f64_to_stamp(tf_util::stamp_to_f64(&stamp) + offset);
            tfs.transform.translation.x = offset;
            let is_static = false;
            // println!("[{i}] {offset} {:?} {:?}", tf_util::stamp_to_f64(&stamp), tfs.header.stamp);
            let rv = tf_buffer.add_transform(&tfs, is_static);
            assert!(rv.is_ok(), "[i] {offset:.2} {rv:?}");
        }
        for i in 0..num_steps - 1 {
            let offset = i as f64 * dt + dt * 0.5;
            let stamp = tf_util::f64_to_stamp(tf_util::stamp_to_f64(&stamp) + offset);
            let rv = tf_buffer.lookup_transform("base", "leaf", Some(stamp.clone()));
            assert!(rv.is_ok(), "[{i}], offset: {offset}, error: {rv:?}");
            let tfs = rv.unwrap();
            let translation = tfs.transform.translation;
            let diff = translation.x - offset;
            assert!(diff.abs() < 0.001, "[{i}] ({diff:.1}).abs() !< 0.001, translation: {translation:?}, offset {offset:.1}, {stamp:?}");
        }
    }
}
