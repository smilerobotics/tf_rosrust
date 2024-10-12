use chrono::TimeDelta;
use roslibrust_codegen::Time;
use serde_derive::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::SystemTime;

use crate::transforms::{geometry_msgs, tf2_msgs};
use crate::{tf_error::TfError, LookupTransform};

pub fn to_stamp(secs: u32, nsecs: u32) -> Time {
    roslibrust_codegen::Time { secs, nsecs }
}

pub fn duration_now() -> TimeDelta {
    let elapsed = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap();
    // println!("{} {}", elapsed.as_secs(), elapsed.subsec_nanos());
    TimeDelta::new(elapsed.as_secs() as i64, elapsed.subsec_nanos()).unwrap()
}

pub fn duration_to_stamp(time: TimeDelta) -> Time {
    to_stamp(time.num_seconds() as u32, time.subsec_nanos() as u32)
}

pub fn f64_to_stamp(seconds: f64) -> roslibrust_codegen::Time {
    let secs = seconds as u32;
    let nsecs = ((seconds - secs as f64) * 1e9) as u32;
    to_stamp(secs, nsecs)
}

pub fn stamp_now() -> roslibrust_codegen::Time {
    duration_to_stamp(duration_now())
}

pub fn stamp_to_duration(stamp: &Time) -> TimeDelta {
    // TODO(lucasw) if a stamp is manually created it could have nsecs > 1e9
    let mut secs = stamp.secs;
    // if nsecs > 1e9 the timedelta will fail
    let mut nsecs = stamp.nsecs;
    let nsecs_per_sec = 1e9 as u32;
    secs += nsecs / nsecs_per_sec;
    nsecs %= nsecs_per_sec;
    TimeDelta::new(secs.into(), nsecs).unwrap_or_else(|| panic!("secs: {secs} nsecs: {nsecs}"))
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
}
