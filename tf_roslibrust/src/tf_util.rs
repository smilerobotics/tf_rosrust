use chrono::TimeDelta;
use roslibrust_codegen::Time;
use serde_derive::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::SystemTime;

use crate::transforms::{geometry_msgs, tf2_msgs};

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

pub fn stamp_now() -> roslibrust_codegen::Time {
    duration_to_stamp(duration_now())
}

pub fn stamp_to_duration(stamp: &Time) -> TimeDelta {
    TimeDelta::new(stamp.secs.into(), stamp.nsecs).unwrap()
}

pub fn duration_to_f64(time: TimeDelta) -> f64 {
    time.num_seconds() as f64 + (time.subsec_nanos() as f64 / 1e9)
}

pub fn stamp_to_f64(stamp: Time) -> f64 {
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

impl TransformRaw {
    fn from_transform_stamped(tfs: geometry_msgs::TransformStamped) -> Self {
        let rot = tfs.transform.rotation;
        let quat = nalgebra::UnitQuaternion::new_normalize(nalgebra::geometry::Quaternion::new(
            rot.w, rot.x, rot.y, rot.z,
        ));
        let (roll, pitch, yaw) = quat.euler_angles();

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

        let unit_quat = nalgebra::UnitQuaternion::from_euler_angles(roll, pitch, yaw);
        let quat = unit_quat.quaternion();
        transform.transform.rotation.x = quat.coords[0];
        transform.transform.rotation.y = quat.coords[1];
        transform.transform.rotation.z = quat.coords[2];
        transform.transform.rotation.w = quat.coords[3];

        tfm.transforms.push(transform);
    }

    Ok(tfm)
}
