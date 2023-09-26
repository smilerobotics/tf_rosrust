use std::collections::{hash_map::Entry, HashMap, HashSet, VecDeque};

use r2r::{
    builtin_interfaces::msg::{Duration, Time},
    geometry_msgs::msg::{Transform, TransformStamped},
    std_msgs::msg::Header,
    tf2_msgs::msg::TFMessage,
};

use crate::{
    tf_error::TfError,
    tf_graph_node::TfGraphNode,
    tf_individual_transform_chain::TfIndividualTransformChain,
    transforms::{chain_transforms, get_inverse, to_transform_stamped},
};

#[derive(Clone, Debug)]
pub struct TfBuffer {
    child_transform_index: HashMap<String, HashSet<String>>,
    transform_data: HashMap<TfGraphNode, TfIndividualTransformChain>,
    cache_duration: Duration,
}

const DEFAULT_CACHE_DURATION_SECONDS: i32 = 10;

impl TfBuffer {
    pub(crate) fn new() -> Self {
        Self::new_with_duration(Duration {
            sec: DEFAULT_CACHE_DURATION_SECONDS,
            nanosec: 0,
        })
    }

    pub fn new_with_duration(cache_duration: Duration) -> Self {
        TfBuffer {
            child_transform_index: HashMap::new(),
            transform_data: HashMap::new(),
            cache_duration,
        }
    }

    pub(crate) fn handle_incoming_transforms(&mut self, transforms: TFMessage, static_tf: bool) {
        for transform in transforms.transforms {
            self.add_transform(&transform, static_tf);
            self.add_transform(&get_inverse(&transform), static_tf);
        }
    }

    fn add_transform(&mut self, transform: &TransformStamped, static_tf: bool) {
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
                self.cache_duration.clone(),
            )),
        }
        .add_to_buffer(transform.clone());
    }

    /// Retrieves the transform path
    fn retrieve_transform_path(
        &self,
        from: String,
        to: String,
        time: &Time,
    ) -> Result<Vec<String>, TfError> {
        let mut res = vec![];
        let mut frontier: VecDeque<String> = VecDeque::new();
        let mut visited: HashSet<String> = HashSet::new();
        let mut parents: HashMap<String, String> = HashMap::new();
        visited.insert(from.clone());
        frontier.push_front(from.clone());

        while !frontier.is_empty() {
            let current_node = frontier.pop_front().unwrap();
            if current_node == to {
                break;
            }
            if let Some(children) = self.child_transform_index.get(&current_node) {
                for v in children {
                    if visited.contains(v) {
                        continue;
                    }

                    if self
                        .transform_data
                        .get(&TfGraphNode {
                            child: v.clone(),
                            parent: current_node.clone(),
                        })
                        .map_or(false, |chain| chain.has_valid_transform(&time))
                    {
                        parents.insert(v.to_string(), current_node.clone());
                        frontier.push_front(v.to_string());
                        visited.insert(v.to_string());
                    }
                }
            }
        }
        let mut r = to.clone();
        while r != from {
            res.push(r.clone());
            let parent = parents.get(&r);

            match parent {
                Some(x) => r = x.to_string(),
                None => {
                    return Err(TfError::CouldNotFindTransform(
                        from,
                        to,
                        self.child_transform_index.clone(),
                    ))
                }
            }
        }
        res.reverse();
        Ok(res)
    }

    /// Looks up a transform within the tree at a given time.
    pub fn lookup_transform(
        &self,
        from: &str,
        to: &str,
        time: &Time,
    ) -> Result<TransformStamped, TfError> {
        let from = from.to_string();
        let to = to.to_string();
        let path = self.retrieve_transform_path(from.clone(), to.clone(), &time);

        match path {
            Ok(path) => {
                let mut tf_list: Vec<Transform> = Vec::new();
                let mut first = from.clone();
                for intermediate in path {
                    let node = TfGraphNode {
                        child: intermediate.clone(),
                        parent: first.clone(),
                    };
                    let time_cache = self.transform_data.get(&node).unwrap();
                    let transform = time_cache.get_closest_transform(&time);
                    match transform {
                        Err(e) => return Err(e),
                        Ok(x) => {
                            tf_list.push(x.transform);
                        }
                    }
                    first = intermediate.clone();
                }
                let final_tf = chain_transforms(&tf_list);
                let msg = TransformStamped {
                    child_frame_id: to,
                    header: Header {
                        frame_id: from,
                        stamp: time.clone(),
                    },
                    transform: final_tf,
                };
                Ok(msg)
            }
            Err(x) => Err(x),
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
        let tf1 = self.lookup_transform(from, fixed_frame, &time1)?;
        let tf2 = self.lookup_transform(to, fixed_frame, &time2)?;
        let transforms = get_inverse(&tf1);
        let result = chain_transforms(&[tf2.transform, transforms.transform]);
        Ok(to_transform_stamped(
            result,
            from.to_string(),
            to.to_string(),
            &time1,
        ))
    }
}

#[cfg(test)]
mod test {
    use crate::utils::time_from_nanosec;

    use super::*;

    use r2r::{
        builtin_interfaces::msg::Time,
        geometry_msgs::msg::{Quaternion, Vector3},
    };

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
                    sec: time.floor() as i32,
                    nanosec: nsecs,
                },
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
                    sec: time.floor() as i32,
                    nanosec: nsecs,
                },
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
                    sec: time.floor() as i32,
                    nanosec: nsecs,
                },
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
        let res = tf_buffer.lookup_transform("camera", "item", &time_from_nanosec(0));
        let expected = TransformStamped {
            child_frame_id: "item".to_string(),
            header: Header {
                frame_id: "camera".to_string(),
                stamp: time_from_nanosec(0),
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
        let res = tf_buffer.lookup_transform("camera", "item", &time_from_nanosec(700_000_000));
        let expected = TransformStamped {
            child_frame_id: "item".to_string(),
            header: Header {
                frame_id: "camera".to_string(),
                stamp: time_from_nanosec(700_000_000),
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
            time_from_nanosec(400_000_000),
            "camera",
            time_from_nanosec(700_000_000),
            "item",
        );
        let expected = TransformStamped {
            child_frame_id: "camera".to_string(),
            header: Header {
                frame_id: "camera".to_string(),
                stamp: time_from_nanosec(700_000_000),
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
                stamp: time_from_nanosec(0),
                ..Default::default()
            },
            child_frame_id: CHILD0.to_string(),
            ..Default::default()
        };
        let transform01 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                stamp: Time { sec: 1, nanosec: 0 },
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
        let mut tf_buffer = TfBuffer::new_with_duration(Duration { sec: 1, nanosec: 0 });
        let transform00 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                stamp: time_from_nanosec(0),
                ..Default::default()
            },
            child_frame_id: CHILD0.to_string(),
            ..Default::default()
        };
        let transform01 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                stamp: Time { sec: 1, nanosec: 0 },
                ..Default::default()
            },
            child_frame_id: CHILD0.to_string(),
            ..Default::default()
        };
        let transform02 = TransformStamped {
            header: Header {
                frame_id: PARENT.to_string(),
                stamp: Time { sec: 2, nanosec: 0 },
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
            data.unwrap().transform_chain.get(0).unwrap().header.stamp,
            time_from_nanosec(0)
        );

        tf_buffer.add_transform(&transform01, static_tf);
        assert_eq!(tf_buffer.child_transform_index.len(), 1);
        assert_eq!(tf_buffer.transform_data.len(), 1);
        assert!(tf_buffer.transform_data.contains_key(&transform0_key));
        let data = tf_buffer.transform_data.get(&transform0_key);
        assert!(data.is_some());
        assert_eq!(data.unwrap().transform_chain.len(), 2);
        assert_eq!(
            data.unwrap().transform_chain.get(0).unwrap().header.stamp,
            time_from_nanosec(0)
        );
        assert_eq!(
            data.unwrap().transform_chain.get(1).unwrap().header.stamp,
            time_from_nanosec(1_000_000_000)
        );

        tf_buffer.add_transform(&transform02, static_tf);
        assert_eq!(tf_buffer.child_transform_index.len(), 1);
        assert_eq!(tf_buffer.transform_data.len(), 1);
        assert!(tf_buffer.transform_data.contains_key(&transform0_key));
        let data = tf_buffer.transform_data.get(&transform0_key);
        assert!(data.is_some());
        assert_eq!(data.unwrap().transform_chain.len(), 2);
        assert_eq!(
            data.unwrap().transform_chain.get(0).unwrap().header.stamp,
            time_from_nanosec(1_000_000_000)
        );
        assert_eq!(
            data.unwrap().transform_chain.get(1).unwrap().header.stamp,
            time_from_nanosec(2_000_000_000)
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
                stamp: Time { sec: 1, nanosec: 0 },
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
                stamp: Time { sec: 1, nanosec: 0 },
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
                stamp: Time { sec: 1, nanosec: 0 },
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
                stamp: Time { sec: 1, nanosec: 0 },
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
        camera1_to_marker.transform.translation.y = -1.0;
        tf_buffer.add_transform(&camera1_to_marker, false);
        tf_buffer.add_transform(&get_inverse(&camera1_to_marker), false);

        let mut camera2_to_marker = TransformStamped {
            child_frame_id: "marker".to_string(),
            header: Header {
                frame_id: "camera2".to_string(),
                stamp: Time { sec: 3, nanosec: 0 },
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
        camera2_to_marker.transform.translation.y = -1.0;
        tf_buffer.add_transform(&camera2_to_marker, false);
        tf_buffer.add_transform(&get_inverse(&camera2_to_marker), false);

        let result = tf_buffer.lookup_transform("base", "target", &Time { sec: 1, nanosec: 0 });
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
            &Time {
                sec: 1,
                nanosec: 500_000_000,
            },
        );
        assert_eq!(
            result.unwrap().transform.translation,
            Vector3 {
                x: 1.5,
                y: 0.0,
                z: 0.0
            }
        );

        let result = tf_buffer.lookup_transform("base", "target", &Time { sec: 2, nanosec: 0 });
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
            &Time {
                sec: 2,
                nanosec: 500_000_000,
            },
        );
        assert!(result.is_err());

        let result = tf_buffer.lookup_transform("base", "target", &Time { sec: 3, nanosec: 0 });
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
            &Time {
                sec: 3,
                nanosec: 500_000_000,
            },
        );
        assert_eq!(
            result.unwrap().transform.translation,
            Vector3 {
                x: -1.5,
                y: -0.0,
                z: 0.0
            }
        );

        let result = tf_buffer.lookup_transform("base", "target", &Time { sec: 4, nanosec: 0 });
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
            &Time {
                sec: 4,
                nanosec: 500_000_000,
            },
        );
        assert!(result.is_err());

        camera1_to_marker.header.stamp.sec = 5;
        camera1_to_marker.transform.translation.x = 0.5;
        camera1_to_marker.transform.translation.y = 1.0;
        tf_buffer.add_transform(&camera1_to_marker, false);
        tf_buffer.add_transform(&get_inverse(&camera1_to_marker), false);

        camera1_to_marker.header.stamp.sec = 6;
        camera1_to_marker.transform.translation.y = -1.0;
        tf_buffer.add_transform(&camera1_to_marker, false);
        tf_buffer.add_transform(&get_inverse(&camera1_to_marker), false);

        let result = tf_buffer.lookup_transform("base", "target", &Time { sec: 5, nanosec: 0 });
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
            &Time {
                sec: 5,
                nanosec: 500_000_000,
            },
        );
        assert_eq!(
            result.unwrap().transform.translation,
            Vector3 {
                x: 1.0,
                y: 0.0,
                z: 0.0
            }
        );

        let result = tf_buffer.lookup_transform("base", "target", &Time { sec: 6, nanosec: 0 });
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
