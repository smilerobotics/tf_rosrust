use chrono::TimeDelta;
use roslibrust::codegen::integral_types::Time;
use roslibrust_util::{geometry_msgs, sensor_msgs, std_msgs, tf2_msgs};
use serde_derive::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::time::SystemTime;

use crate::transforms::isometry_to_transform;
use crate::TfBuffer;
use crate::{tf_error::TfError, LookupTransform};

pub fn to_stamp(secs: i32, nsecs: i32) -> Time {
    Time { secs, nsecs }
}

pub fn duration_now() -> TimeDelta {
    let elapsed = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap();
    // println!("{} {}", elapsed.as_secs(), elapsed.subsec_nanos());
    TimeDelta::new(elapsed.as_secs() as i64, elapsed.subsec_nanos()).unwrap()
}

pub fn duration_to_stamp(time: TimeDelta) -> Time {
    to_stamp(time.num_seconds() as i32, time.subsec_nanos())
}

pub fn f64_to_stamp(seconds: f64) -> Time {
    let secs = seconds as i32;
    let nsecs = ((seconds - secs as f64) * 1e9) as i32;
    to_stamp(secs, nsecs)
}

pub fn stamp_now() -> Time {
    duration_to_stamp(duration_now())
}

pub fn stamp_to_duration(stamp: &Time) -> TimeDelta {
    // TODO(lucasw) if a stamp is manually created it could have nsecs > 1e9
    let mut secs = stamp.secs;
    // if nsecs > 1e9 the timedelta will fail
    let mut nsecs = stamp.nsecs;
    let nsecs_per_sec = 1e9 as i32;
    secs += nsecs / nsecs_per_sec;
    nsecs %= nsecs_per_sec;
    TimeDelta::new(secs.into(), nsecs as u32)
        .unwrap_or_else(|| panic!("secs: {secs} nsecs: {nsecs}"))
}

pub fn duration_to_f64(time: TimeDelta) -> f64 {
    time.num_seconds() as f64 + (time.subsec_nanos() as f64 / 1e9)
}

pub fn stamp_to_f64(stamp: &Time) -> f64 {
    stamp.secs as f64 + (stamp.nsecs as f64) / 1e9
}

/// use for loading from a toml
#[derive(Deserialize, Serialize, Debug)]
struct TransformRaw {
    frame: String,
    child_frame: String,
    x: Option<f64>,
    y: Option<f64>,
    z: Option<f64>,
    roll: Option<f64>,
    pitch: Option<f64>,
    yaw: Option<f64>,
}

pub fn quat_msg_to_rpy(quat_msg: geometry_msgs::Quaternion) -> (f64, f64, f64) {
    let quat = nalgebra::UnitQuaternion::new_normalize(nalgebra::geometry::Quaternion::new(
        quat_msg.w, quat_msg.x, quat_msg.y, quat_msg.z,
    ));
    let (roll, pitch, yaw) = quat.euler_angles();
    (roll, pitch, yaw)
}

pub fn rpy_to_quat_msg(roll: f64, pitch: f64, yaw: f64) -> geometry_msgs::Quaternion {
    let unit_quat = nalgebra::UnitQuaternion::from_euler_angles(roll, pitch, yaw);
    let quat = unit_quat.quaternion();
    geometry_msgs::Quaternion {
        x: quat.coords[0],
        y: quat.coords[1],
        z: quat.coords[2],
        w: quat.coords[3],
    }
}

impl TransformRaw {
    fn from_transform_stamped(tfs: geometry_msgs::TransformStamped) -> Self {
        let (roll, pitch, yaw) = quat_msg_to_rpy(tfs.transform.rotation);

        let tr = tfs.transform.translation;
        let (x, y, z) = (tr.x, tr.y, tr.z);

        Self {
            frame: tfs.header.frame_id,
            child_frame: tfs.child_frame_id,
            x: Some(x),
            y: Some(y),
            z: Some(z),
            roll: Some(roll),
            pitch: Some(pitch),
            yaw: Some(yaw),
        }
    }
}

pub fn transforms_to_toml(tfm: tf2_msgs::TFMessage) -> Result<String, anyhow::Error> {
    let mut tf_vec = Vec::new();
    for tfs in tfm.transforms {
        tf_vec.push(TransformRaw::from_transform_stamped(tfs));
    }

    let mut tf_data = HashMap::new();
    tf_data.insert("tf", tf_vec);

    Ok(toml::to_string(&tf_data).unwrap())
}

pub fn get_transforms_from_toml(filename: &str) -> Result<tf2_msgs::TFMessage, anyhow::Error> {
    let tf_data = {
        let contents = match std::fs::read_to_string(filename) {
            Ok(contents) => contents,
            // Handle the `error` case.
            Err(err) => {
                panic!("Could not read file '{filename}', {err}");
            }
        };
        // println!("{contents}");
        let tf_data: HashMap<String, Vec<TransformRaw>> = toml::from_str(&contents)?;
        // println!("{tf_data:?}");

        tf_data
    };

    let mut tfm = tf2_msgs::TFMessage::default();
    for tfr in tf_data.get("tf").ok_or(anyhow::anyhow!("no tfs"))? {
        let mut transform = geometry_msgs::TransformStamped::default();
        // transform.header.stamp = stamp;
        transform.header.frame_id = tfr.frame.clone();
        transform.child_frame_id = tfr.child_frame.clone();
        transform.transform.rotation.w = 1.0;

        // TODO(lucasw) try out a macro
        if let Some(x) = tfr.x {
            transform.transform.translation.x = x;
        }
        if let Some(y) = tfr.y {
            transform.transform.translation.y = y;
        }
        if let Some(z) = tfr.z {
            transform.transform.translation.z = z;
        }

        let roll = tfr.roll.unwrap_or(0.0);
        let pitch = tfr.pitch.unwrap_or(0.0);
        let yaw = tfr.yaw.unwrap_or(0.0);

        transform.transform.rotation = rpy_to_quat_msg(roll, pitch, yaw);

        tfm.transforms.push(transform);
    }

    Ok(tfm)
}

// adapted from 'old_tf_to_new_tf'
pub fn tf2tf(
    tf_listener: &crate::tf_listener::TfListener,
    lookup_parent: &str,
    lookup_child: &str,
    broadcast_parent: &str,
    broadcast_child: &str,
    zero_xyz: (bool, bool, bool),
    zero_rotation: bool,
) -> Result<geometry_msgs::TransformStamped, anyhow::Error> {
    // get the most recent parent child transform, zero out x,y,z and/or rotation
    // TODO(lucasw) does tf listener handle looping time?  No it doesn't
    let mut tfs = tf_listener.lookup_transform(lookup_parent, lookup_child, None)?;

    tfs.header.frame_id = broadcast_parent.to_string();
    tfs.child_frame_id = broadcast_child.to_string();
    let (zero_x, zero_y, zero_z) = zero_xyz;
    if zero_x {
        tfs.transform.translation.x = 0.0;
    }
    if zero_y {
        tfs.transform.translation.y = 0.0;
    }
    if zero_z {
        tfs.transform.translation.z = 0.0;
    }
    if zero_rotation {
        tfs.transform.rotation.x = 0.0;
        tfs.transform.rotation.y = 0.0;
        tfs.transform.rotation.z = 0.0;
        tfs.transform.rotation.w = 1.0;
    }
    Ok(tfs)
}

/// use for loading from a toml
#[derive(Deserialize, Serialize, Debug)]
pub struct Tf2TfConfig {
    pub lookup_parent: String,
    pub lookup_child: String,
    pub broadcast_parent: String,
    pub broadcast_child: String,
    // set these to 0.0 to accomplish the same thing as zero_x/y/z/etc. above
    fixed_x: Option<f64>,
    fixed_y: Option<f64>,
    fixed_z: Option<f64>,
    fixed_roll: Option<f64>,
    fixed_pitch: Option<f64>,
    fixed_yaw: Option<f64>,
}

pub fn get_tf2tf_from_toml(filename: &str) -> Result<Vec<Tf2TfConfig>, anyhow::Error> {
    let contents = match std::fs::read_to_string(filename) {
        Ok(contents) => contents,
        // Handle the `error` case.
        Err(err) => {
            panic!("Could not read file '{filename}', {err}");
        }
    };
    // println!("{contents}");
    let mut tf2tf_data: HashMap<String, Vec<Tf2TfConfig>> = toml::from_str(&contents)?;
    // println!("{tf2tf_data:?}");
    tf2tf_data
        .remove("tf2tf")
        .ok_or(anyhow::anyhow!("no tf2tfs"))
}

pub fn tf2tf_to_tfm(
    tf_lookup: &impl LookupTransform,
    tf2tf_config: &Vec<Tf2TfConfig>,
) -> (tf2_msgs::TFMessage, Vec<TfError>) {
    let mut tfm = tf2_msgs::TFMessage::default();
    let mut tf_errors = Vec::new();
    for tf2tf in tf2tf_config {
        // get the most recent parent child transform, zero out x,y,z and/or rotation
        match tf_lookup.lookup_transform(&tf2tf.lookup_parent, &tf2tf.lookup_child, None) {
            Err(error) => {
                // if rv is error, continue on and get as many transforms as possible...
                tf_errors.push(error);
                continue;
            }
            Ok(mut tfs) => {
                tfs.header.frame_id = tf2tf.broadcast_parent.clone();
                tfs.child_frame_id = tf2tf.broadcast_child.clone();
                if let Some(x) = tf2tf.fixed_x {
                    tfs.transform.translation.x = x;
                }
                if let Some(y) = tf2tf.fixed_y {
                    tfs.transform.translation.y = y;
                }
                if let Some(z) = tf2tf.fixed_z {
                    tfs.transform.translation.z = z;
                }

                let (mut roll, mut pitch, mut yaw) = quat_msg_to_rpy(tfs.transform.rotation);
                if let Some(fixed_roll) = tf2tf.fixed_roll {
                    roll = fixed_roll;
                }
                if let Some(fixed_pitch) = tf2tf.fixed_pitch {
                    pitch = fixed_pitch;
                }
                if let Some(fixed_yaw) = tf2tf.fixed_yaw {
                    yaw = fixed_yaw;
                }
                tfs.transform.rotation = rpy_to_quat_msg(roll, pitch, yaw);

                tfm.transforms.push(tfs);
            }
        }
    }
    (tfm, tf_errors)
}

/// use for loading joints from a toml
#[derive(Deserialize, Serialize, Debug)]
pub struct JointConfig {
    name: String,
    parent: String,
    child: String,
    // offset from parent before applying rotation, default all 0.0
    translation_x: Option<f64>,
    translation_y: Option<f64>,
    translation_z: Option<f64>,
    // axis to rotate around, will be normalized to length 1.0
    axis_x: f64,
    axis_y: f64,
    axis_z: f64,
}

#[derive(Debug)]
pub struct Joint {
    pub name: String,
    pub parent: String,
    pub child: String,
    // offset from parent before applying rotation, default all 0.0
    pub translation: nalgebra::base::Vector3<f64>,
    // axis to rotate around, will be normalized to length 1.0
    // TODO(lucasw) if it wasn't normalized it would be a way to apply a scale factor to incoming
    // joint state values
    pub axis: nalgebra::base::UnitVector3<f64>,
}

pub fn get_joints_from_toml(filename: &str) -> Result<HashMap<String, Joint>, anyhow::Error> {
    let contents = match std::fs::read_to_string(filename) {
        Ok(contents) => contents,
        Err(err) => {
            panic!("Could not read joint file: '{filename}', {err}");
        }
    };
    let mut joint_data: HashMap<String, Vec<JointConfig>> = toml::from_str(&contents)?;
    let joint_data = joint_data
        .remove("joint")
        .ok_or(anyhow::anyhow!("no joints"))?;

    let mut joints = HashMap::new();
    for joint_config in joint_data {
        let x = joint_config.translation_x.unwrap_or(0.0);
        let y = joint_config.translation_y.unwrap_or(0.0);
        let z = joint_config.translation_z.unwrap_or(0.0);
        let axis_x = joint_config.axis_x;
        let axis_y = joint_config.axis_y;
        let axis_z = joint_config.axis_z;
        let joint = Joint {
            name: joint_config.name,
            parent: joint_config.parent,
            child: joint_config.child,
            translation: nalgebra::Vector3::new(x, y, z),
            axis: nalgebra::Unit::new_normalize(nalgebra::Vector3::new(axis_x, axis_y, axis_z)),
        };
        // return an error if multiple entries with the same joint name
        // let _ = joints.try_insert(joint.name, joint)?;
        let _ = joints.insert(joint.name.clone(), joint);
    }
    Ok(joints)
}

/// convert a JointState message to a hashmap with joint name keys
pub fn joint_state_map(js: &sensor_msgs::JointState) -> HashMap<String, (f64, f64, f64)> {
    let mut joints = HashMap::new();
    for (ind, joint_name) in js.name.iter().enumerate() {
        let position = {
            if ind < js.position.len() {
                js.position[ind]
            } else {
                0.0
            }
        };

        let velocity = {
            if ind < js.velocity.len() {
                js.velocity[ind]
            } else {
                0.0
            }
        };

        let effort = {
            if ind < js.effort.len() {
                js.effort[ind]
            } else {
                0.0
            }
        };

        // TODO(lucasw) return error/s if duplicate names
        joints.insert(joint_name.clone(), (position, velocity, effort));
    }
    joints
}

pub fn joint_states_to_tfm(
    js: &sensor_msgs::JointState,
    joints_config: &HashMap<String, Joint>,
) -> Result<tf2_msgs::TFMessage, anyhow::Error> {
    let joint_states = joint_state_map(js);
    let mut tfm = tf2_msgs::TFMessage::default();
    for (joint_name, (position, _velocity, _effort)) in joint_states {
        let rv = joints_config.get(&joint_name);
        match rv {
            Some(joint) => {
                let axis_angle = joint.axis.into_inner() * position;
                let iso = nalgebra::Isometry3::new(
                    nalgebra::base::Vector3::new(
                        joint.translation.x,
                        joint.translation.y,
                        joint.translation.z,
                    ),
                    axis_angle,
                );
                let transform = isometry_to_transform(iso);
                let tfs = geometry_msgs::TransformStamped {
                    header: std_msgs::Header {
                        frame_id: joint.parent.clone(),
                        stamp: js.header.stamp.clone(),
                        seq: js.header.seq,
                    },
                    child_frame_id: joint.child.clone(),
                    transform,
                };
                tfm.transforms.push(tfs);
            }
            None => {
                println!("no {joint_name} in joint config");
            }
        }
    }
    Ok(tfm)
}

/// print the current tf tree
/// adapted from tf_demo tf_tree.py
fn print_tree_recursive(
    tf_buffer: &TfBuffer,
    parent_to_children: &HashMap<String, HashSet<String>>,
    parent: &str,
    level: u8,
) -> Result<(), anyhow::Error> {
    if level > 1 {
        for _ in 0..level {
            print!("   ");
        }
    }
    if level > 0 {
        print!("-- ");
    }

    print!("{parent}");
    let rate = tf_buffer.get_rate(parent);
    if let Some(rate) = rate {
        if rate > 0.0 {
            print!("  {rate:.2}Hz");
        }
    }
    println!();

    match parent_to_children.get(parent) {
        Some(children) => {
            let mut children = children.iter().collect::<Vec<_>>();
            children.sort();
            for child in children {
                let _ = print_tree_recursive(tf_buffer, parent_to_children, child, level + 1);
            }
        }
        None => {
            // no children
            return Ok(());
        }
    }

    Ok(())
}

pub fn print_tree(tf_buffer: &TfBuffer) -> Result<(), anyhow::Error> {
    let parent_to_children = tf_buffer.get_parent_to_children();
    if false {
        let mut keys_sorted = parent_to_children.keys().collect::<Vec<_>>();
        keys_sorted.sort();
        for parent in keys_sorted {
            println!("{parent}: {:?}", parent_to_children.get(parent).unwrap());
        }
    }

    let roots = tf_buffer.get_roots()?;
    for root in roots {
        println!("[");
        let _ = print_tree_recursive(tf_buffer, &parent_to_children, &root, 0);
        println!("]");
    }
    Ok(())
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_time_conversion() {
        let base_stamp = Time {
            secs: 1_002_003_000,
            nsecs: 0,
        };

        for i in 0..1000 {
            let offset = i as f64 * 0.1;
            // see if there's a panic within this
            let _stamp = f64_to_stamp(stamp_to_f64(&base_stamp) + offset);
        }
    }

    #[test]
    fn test_joints() {
        let config_file = format!("{}/examples/joints.toml", env!("CARGO_MANIFEST_DIR"));
        let joints_config = get_joints_from_toml(&config_file).unwrap();

        let header = std_msgs::Header::default();
        let joint_state = sensor_msgs::JointState {
            header,
            name: vec![
                "back_wheel_joint".into(),
                "front_steer_joint".into(),
                "front_wheel_joint".into(),
            ],
            position: vec![1.570792, 0.31459, 0.5236],
            velocity: vec![],
            effort: vec![],
        };
        let tfm = joint_states_to_tfm(&joint_state, &joints_config).unwrap();
        assert_eq!(tfm.transforms.len(), 3);
        // TODO(lucasw) could assert translations and rotations are as expected
        for tfs in &tfm.transforms {
            println!("{tfs:?}");
        }
    }
}
