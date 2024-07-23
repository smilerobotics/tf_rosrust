use std::collections::{hash_map::Entry, BTreeSet, HashMap, HashSet};

use roslibrust_codegen::Time;
use chrono::TimeDelta;

use crate::{
    tf_error::TfError,
    tf_graph_node::TfGraphNode,
    tf_individual_transform_chain::{stamp_to_duration, TfIndividualTransformChain},
    transforms::{
        chain_transforms,
        geometry_msgs::{Transform, TransformStamped},
        get_inverse,
        std_msgs::Header,
        tf2_msgs::TFMessage,
        to_transform_stamped,
    },
};

#[derive(Clone, Debug)]
pub struct TfBuffer {
    parent_transform_index: HashMap<String, String>,
    child_transform_index: HashMap<String, HashSet<String>>,
    transform_data: HashMap<TfGraphNode, TfIndividualTransformChain>,
    cache_duration: TimeDelta,
}

const DEFAULT_CACHE_DURATION_SECONDS: u16 = 10;

impl TfBuffer {
    pub(crate) fn new() -> Self {
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

    pub(crate) fn handle_incoming_transforms(&mut self, transforms: TFMessage, static_tf: bool) {
        for transform in transforms.transforms {
            self.add_transform(&transform, static_tf);
            // self.add_transform(&get_inverse(&transform), static_tf);
        }
    }

    // TODO(lucasw) detect loops
    fn add_transform(&mut self, transform: &TransformStamped, static_tf: bool) {
        self.parent_transform_index
            .insert(transform.child_frame_id.clone(), transform.header.frame_id.clone());

        //TODO: Detect is new transform will create a loop
        self.child_transform_index
            .entry(transform.header.frame_id.clone())
            .or_default()
            .insert(transform.child_frame_id.clone());

        let key = TfGraphNode {
            child: transform.child_frame_id.clone(),
            parent: transform.header.frame_id.clone(),
        };

        match self.transform_data.entry(key) {
            Entry::Occupied(e) => e.into_mut(),
            Entry::Vacant(e) => e.insert(TfIndividualTransformChain::new(
                static_tf,
                self.cache_duration,
            )),
        }
        .add_to_buffer(transform.clone());
    }

    /// Retrieves the transform path
    fn retrieve_transform_path(
        &self,
        from: String,
        to: String,
        // stamp: Option<Time>,
    ) -> Result<Vec<String>, TfError> {
        let mut res = vec![];

        /*
        let duration = match stamp {
            Some(stamp) => Some(stamp_to_duration(stamp.clone())),
            None => None,
        };
        */

        // for (key, value) in &self.parent_transform_index {
        //     println!("{key}: {value}");
        // }
        // return Err(TfError::CouldNotFindTransform(from, to, self.child_transform_index.clone()));

        // Find the common parent of from and to, first get all the parents of from
        // all the way to the root, then start getting parents of to and terminate
        // when one is found that is in the from parents set

        // TODO(lucasw) use an IndexMap
        let mut from_lineage_visited = BTreeSet::<String>::new();
        let mut from_lineage = Vec::new();
        from_lineage_visited.insert(from.clone());
        from_lineage.push(from.clone());

        let mut cur_frame = from.clone();
        // println!("get 'from' frame lineage '{from}'");

        // TODO(lucasw) could this be done in one line?
        while self.parent_transform_index.contains_key(&cur_frame) {
            // println!("-- {cur_frame}");
            cur_frame = self.parent_transform_index.get(&cur_frame).unwrap().to_string();
            // println!("  |-> {cur_frame}");
            from_lineage_visited.insert(cur_frame.clone());
            from_lineage.push(cur_frame.clone());
        }
        println!("from lineage {from_lineage:?}");

        let mut to_lineage_visited = BTreeSet::<String>::new();
        let mut to_lineage = Vec::new();
        to_lineage.push(to.clone());

        // println!("get 'to' frame lineage '{to}'");
        let mut cur_frame = to.clone();
        while self.parent_transform_index.contains_key(&cur_frame) {
            // println!("-- {cur_frame}");
            cur_frame = self.parent_transform_index.get(&cur_frame).unwrap().to_string();
            // println!("  |-> {cur_frame}");
            to_lineage.push(cur_frame.clone());
        }
        println!("to lineage {to_lineage:?}");

        let mut common_parent = None;
        for frame in &to_lineage {
            if from_lineage_visited.contains(frame) {
                common_parent = Some(frame);
                println!("common parent found: {frame}");
                break;
            }
        }

        if common_parent.is_none() {
            println!("{from_lineage:?} - {to_lineage:?}");
            return Err(TfError::CouldNotFindTransform(
                from,
                to,
                self.child_transform_index.clone(),
            ))
        }
        let common_parent = common_parent.unwrap();

        for frame in &from_lineage {
            println!("frame in from lineage {frame}");
            res.push(frame.clone());
            if frame == common_parent {
                break;
            }
        }

        let mut found_common_parent = false;
        for frame in (&to_lineage).iter().rev() {
            println!("frame in to lineage {frame}");
            if found_common_parent {
                res.push(frame.clone());
            }
            if frame == common_parent {
                found_common_parent = true;
            }
        }
        println!("full path: {res:?}");

        Ok(res)
    }

    /// Looks up a transform within the tree at a given time.
    pub fn lookup_transform(
        &self,
        from: &str,
        to: &str,
        stamp0: Option<Time>,
    ) -> Result<TransformStamped, TfError> {
        let from = from.to_string();
        let to = to.to_string();

        let stamp;
        if stamp0.is_none() {
            println!("getting most recent transform");
            let path0 = self.retrieve_transform_path(from.clone(), to.clone());  // , stamp0.clone());
            match path0 {
                Err(x) => return Err(x),
                Ok(path0) => {
                    let mut first = from.clone();
                    let mut min_time = None;
                    for intermediate in path0.clone() {
                        let node = TfGraphNode {
                            child: intermediate.clone(),
                            parent: first.clone(),
                        };
                        first.clone_from(&intermediate);
                        // TODO(lucasw) need also get inverse node if this fails,
                        // and then invert the transform if the inverse is found
                        let time_cache = self.transform_data.get(&node).unwrap();
                        // TODO(lucasw) this doesn't get a coherent set of transforms when
                        // wanting the most recent- need to find the earliest and latest
                        // transform for each of these and get the overlap of all of them,
                        // then use the latest of those, then use that as the header stamp
                        // for the result as well (don't use time 0)
                        let transform = time_cache.get_closest_transform(stamp0.clone());
                        match transform {
                            Err(e) => return Err(e),
                            Ok(x) => {
                                let intermediate_time = stamp_to_duration(x.header.stamp.clone());
                                if intermediate_time.is_zero() {
                                    panic!("{x:?}");
                                }
                                // TODO(lucaw) use Some
                                if min_time.is_none() {
                                    // TODO(lucasw) if the intermediate transform is static
                                    // is this correct- does it return the asked for time
                                    // instead of whatever the original static frame was
                                    min_time = Some(intermediate_time);
                                } else {
                                    min_time = Some(std::cmp::min(intermediate_time, min_time.unwrap()));
                                }
                            }
                        }
                    }  // search for time that is before or equal to every transform in the chain
                    match min_time {
                        Some(min_time) => {
                            stamp = Time {
                                secs: min_time.num_seconds() as u32,
                                nsecs: min_time.subsec_nanos() as u32,
                            };
                            println!("most recent stamp {stamp:?} {min_time:?}");
                        },
                        // TODO(lucasw) is this possible given above checks?
                        None => { panic!("don't have valid stamp"); }
                    }
                }
            }
        } else {
            stamp = stamp0.unwrap();
        }

        println!("getting transform at stamp");
        let path = self.retrieve_transform_path(from.clone(), to.clone());  // , Some(stamp.clone()));

        match path {
            Ok(path) => {
                let mut tf_list: Vec<Transform> = Vec::new();
                let mut first = from.clone();
                for intermediate in path {
                    let node = TfGraphNode {
                        child: intermediate.clone(),
                        parent: first.clone(),
                    };
                    first.clone_from(&intermediate);
                    let time_cache = self.transform_data.get(&node).unwrap();
                    // TODO(lucasw) this doesn't get a coherent set of transforms when
                    // wanting the most recent- need to find the earliest and latest
                    // transform for each of these and get the overlap of all of them,
                    // then use the latest of those, then use that as the header stamp
                    // for the result as well (don't use time 0)
                    let transform = time_cache.get_closest_transform(Some(stamp.clone()));
                    match transform {
                        Err(e) => return Err(e),
                        Ok(x) => {
                            tf_list.push(x.transform);
                        }
                    }
                }
                let final_tf = chain_transforms(&tf_list);
                let msg = TransformStamped {
                    child_frame_id: to,
                    header: Header {
                        frame_id: from,
                        stamp: stamp,
                        seq: 1,
                    },
                    transform: final_tf,
                };
                Ok(msg)

            }
            Err(x) => Err(x)
        }
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
        let result = chain_transforms(&[tf2.transform, transforms.transform]);
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
    use crate::transforms::geometry_msgs::{Quaternion, Vector3};

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
                    x: 1f64,
                    y: 0f64,
                    z: 0f64,
                },
            },
        };
        buffer.add_transform(&world_to_item, true);
        buffer.add_transform(&get_inverse(&world_to_item), true);

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
        buffer.add_transform(&world_to_base_link, false);
        buffer.add_transform(&get_inverse(&world_to_base_link), false);

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
        buffer.add_transform(&base_link_to_camera, true);
        buffer.add_transform(&get_inverse(&base_link_to_camera), true);
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
                stamp: Time { secs: 0, nsecs: 0 },
                ..Default::default()
            },
            child_frame_id: CHILD0.to_string(),
            ..Default::default()
        };
        let transform01 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                stamp: Time { secs: 1, nsecs: 0 },
                ..Default::default()
            },
            child_frame_id: CHILD0.to_string(),
            ..Default::default()
        };
        let transform1 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                ..Default::default()
            },
            child_frame_id: CHILD1.to_string(),
            ..Default::default()
        };
        let transform0_key = TfGraphNode {
            child: CHILD0.to_owned(),
            parent: PARENT.to_owned(),
        };
        let transform1_key = TfGraphNode {
            child: CHILD1.to_owned(),
            parent: PARENT.to_owned(),
        };
        let static_tf = true;
        tf_buffer.add_transform(&transform00, static_tf);
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

        tf_buffer.add_transform(&transform01, static_tf);
        assert_eq!(tf_buffer.child_transform_index.len(), 1);
        assert!(tf_buffer.child_transform_index.contains_key(PARENT));
        let children = tf_buffer.child_transform_index.get(PARENT).unwrap();
        assert_eq!(children.len(), 1);
        assert!(children.contains(CHILD0));
        assert_eq!(tf_buffer.transform_data.len(), 1);
        assert!(tf_buffer.transform_data.contains_key(&transform0_key));
        let data = tf_buffer.transform_data.get(&transform0_key);
        assert!(data.is_some());
        assert_eq!(data.unwrap().transform_chain.len(), 2);

        tf_buffer.add_transform(&transform1, static_tf);
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
        let mut tf_buffer = TfBuffer::new_with_duration(TimeDelta::new(1, 0));
        let transform00 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                stamp: Time { secs: 0, nsecs: 0 },
                ..Default::default()
            },
            child_frame_id: CHILD0.to_string(),
            ..Default::default()
        };
        let transform01 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                stamp: Time { secs: 1, nsecs: 0 },
                ..Default::default()
            },
            child_frame_id: CHILD0.to_string(),
            ..Default::default()
        };
        let transform02 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                stamp: Time { secs: 2, nsecs: 0 },
                ..Default::default()
            },
            child_frame_id: CHILD0.to_string(),
            ..Default::default()
        };
        let transform0_key = TfGraphNode {
            child: CHILD0.to_owned(),
            parent: PARENT.to_owned(),
        };

        let static_tf = true;
        tf_buffer.add_transform(&transform00, static_tf);
        assert_eq!(tf_buffer.child_transform_index.len(), 1);
        assert_eq!(tf_buffer.transform_data.len(), 1);
        assert!(tf_buffer.transform_data.contains_key(&transform0_key));
        let data = tf_buffer.transform_data.get(&transform0_key);
        assert!(data.is_some());
        assert_eq!(data.unwrap().transform_chain.len(), 1);
        assert_eq!(
            data.unwrap().transform_chain[0].header.stamp,
            Time::from_nanos(0)
        );

        tf_buffer.add_transform(&transform01, static_tf);
        assert_eq!(tf_buffer.child_transform_index.len(), 1);
        assert_eq!(tf_buffer.transform_data.len(), 1);
        assert!(tf_buffer.transform_data.contains_key(&transform0_key));
        let data = tf_buffer.transform_data.get(&transform0_key);
        assert!(data.is_some());
        assert_eq!(data.unwrap().transform_chain.len(), 2);
        assert_eq!(
            data.unwrap().transform_chain[0].header.stamp,
            Time::from_nanos(0)
        );
        assert_eq!(
            data.unwrap().transform_chain[1].header.stamp,
            Time::from_nanos(1_000_000_000)
        );

        tf_buffer.add_transform(&transform02, static_tf);
        assert_eq!(tf_buffer.child_transform_index.len(), 1);
        assert_eq!(tf_buffer.transform_data.len(), 1);
        assert!(tf_buffer.transform_data.contains_key(&transform0_key));
        let data = tf_buffer.transform_data.get(&transform0_key);
        assert!(data.is_some());
        assert_eq!(data.unwrap().transform_chain.len(), 2);
        assert_eq!(
            data.unwrap().transform_chain[0].header.stamp,
            Time::from_nanos(1_000_000_000)
        );
        assert_eq!(
            data.unwrap().transform_chain[1].header.stamp,
            Time::from_nanos(2_000_000_000)
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

    /// Tests a case in which the tree structure changes dynamically
    /// time 1-2(sec): [base] -> [camera1] -> [marker] -> [target]
    /// time 3-4(sec): [base] -> [camera2] -> [marker] -> [target]
    /// time 5-6(sec): [base] -> [camera1] -> [marker] -> [target]
    #[test]
    fn test_dynamic_tree() {
        let mut tf_buffer = TfBuffer::new();

        let base_to_camera1 = TransformStamped {
            child_frame_id: "camera1".to_string(),
            header: Header {
                frame_id: "base".to_string(),
                stamp: Time { secs: 1, nsecs: 0 },
                seq: 1,
            },
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
        tf_buffer.add_transform(&base_to_camera1, true);
        tf_buffer.add_transform(&get_inverse(&base_to_camera1), true);

        let base_to_camera2 = TransformStamped {
            child_frame_id: "camera2".to_string(),
            header: Header {
                frame_id: "base".to_string(),
                stamp: Time { secs: 1, nsecs: 0 },
                seq: 1,
            },
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
        tf_buffer.add_transform(&base_to_camera2, true);
        tf_buffer.add_transform(&get_inverse(&base_to_camera2), true);

        let marker_to_target = TransformStamped {
            child_frame_id: "target".to_string(),
            header: Header {
                frame_id: "marker".to_string(),
                stamp: Time { secs: 1, nsecs: 0 },
                seq: 1,
            },
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
        tf_buffer.add_transform(&marker_to_target, true);
        tf_buffer.add_transform(&get_inverse(&marker_to_target), true);

        let mut camera1_to_marker = TransformStamped {
            child_frame_id: "marker".to_string(),
            header: Header {
                frame_id: "camera1".to_string(),
                stamp: Time { secs: 1, nsecs: 0 },
                seq: 1,
            },
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
        tf_buffer.add_transform(&camera1_to_marker, false);
        tf_buffer.add_transform(&get_inverse(&camera1_to_marker), false);

        camera1_to_marker.header.stamp.sec = 2;
        camera1_to_marker.header.seq += 1;
        camera1_to_marker.transform.translation.y = -1.0;
        tf_buffer.add_transform(&camera1_to_marker, false);
        tf_buffer.add_transform(&get_inverse(&camera1_to_marker), false);

        let mut camera2_to_marker = TransformStamped {
            child_frame_id: "marker".to_string(),
            header: Header {
                frame_id: "camera2".to_string(),
                stamp: Time { secs: 3, nsecs: 0 },
                seq: 1,
            },
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
        tf_buffer.add_transform(&camera2_to_marker, false);
        tf_buffer.add_transform(&get_inverse(&camera2_to_marker), false);

        camera2_to_marker.header.stamp.sec = 4;
        camera2_to_marker.header.seq += 1;
        camera2_to_marker.transform.translation.y = -1.0;
        tf_buffer.add_transform(&camera2_to_marker, false);
        tf_buffer.add_transform(&get_inverse(&camera2_to_marker), false);

        let result =
            tf_buffer.lookup_transform("base", "target", Some(Time { secs: 1, nsecs: 0 }));
        assert_eq!(
            result.unwrap().transform.translation,
            Vector3 {
                x: 1.5,
                y: 1.0,
                z: 0.0
            }
        );

        let result = tf_buffer.lookup_transform(
            "base",
            "target",
            Sopme(Time {
                secs: 1,
                nsecs: 500_000_000,
            }),
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

        let result = tf_buffer.lookup_transform(
            "base",
            "target",
            Some(Time {
                secs: 2,
                nsecs: 500_000_000,
            }),
        );
        assert!(result.is_err());

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

        let result = tf_buffer.lookup_transform(
            "base",
            "target",
            Some(Time {
                secs: 4,
                nsecs: 500_000_000,
            }),
        );
        assert!(result.is_err());

        camera1_to_marker.header.stamp.sec = 5;
        camera1_to_marker.header.seq += 1;
        camera1_to_marker.transform.translation.x = 0.5;
        camera1_to_marker.transform.translation.y = 1.0;
        tf_buffer.add_transform(&camera1_to_marker, false);
        tf_buffer.add_transform(&get_inverse(&camera1_to_marker), false);

        camera1_to_marker.header.stamp.sec = 6;
        camera1_to_marker.header.seq += 1;
        camera1_to_marker.transform.translation.y = -1.0;
        tf_buffer.add_transform(&camera1_to_marker, false);
        tf_buffer.add_transform(&get_inverse(&camera1_to_marker), false);

        let result =
            tf_buffer.lookup_transform("base", "target", Some(Time { secs: 5, nsecs: 0 }));
        assert_eq!(
            result.unwrap().transform.translation,
            Vector3 {
                x: 1.0,
                y: 1.0,
                z: 0.0
            }
        );

        let result = tf_buffer.lookup_transform(
            "base",
            "target",
            Some(Time {
                secs: 5,
                nsecs: 500_000_000,
            }),
        );
        assert_eq!(
            result.unwrap().transform.translation,
            Vector3 {
                x: 1.0,
                y: 0.0,
                z: 0.0
            }
        );

        let result =
            tf_buffer.lookup_transform("base", "target", Some(Time { secs: 6, nsecs: 0 }));
        assert_eq!(
            result.unwrap().transform.translation,
            Vector3 {
                x: 1.0,
                y: -1.0,
                z: 0.0
            }
        );
    }
}
