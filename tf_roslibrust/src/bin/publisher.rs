use serde_derive::Deserialize;
use std::collections::HashMap;
use tf_roslibrust::tf_util;

roslibrust_codegen_macro::find_and_generate_ros_messages!();

/// Load a toml file of a list of trnasforms and publish them
/// tf_publisher examples/transforms.toml

#[derive(Deserialize, Debug)]
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

fn get_transforms(filename: &str) -> Result<tf2_msgs::TFMessage, anyhow::Error> {
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

        let roll = {
            if let Some(roll) = tfr.roll {
                roll
            } else {
                0.0
            }
        };
        let pitch = {
            if let Some(pitch) = tfr.pitch {
                pitch
            } else {
                0.0
            }
        };
        let yaw = {
            if let Some(yaw) = tfr.yaw {
                yaw
            } else {
                0.0
            }
        };

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

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use roslibrust::ros1::NodeHandle;

    // need to have leading slash on node name and topic to function properly
    // so figure out namespace then prefix it to name and topics
    let mut ns = String::from("");
    let args = std::env::args();
    let mut args2 = Vec::new();
    {
        // get namespace
        for arg in args {
            if arg.starts_with("__ns:=") {
                ns = arg.replace("__ns:=", "");
            } else {
                args2.push(arg);
            }
        }
    }

    let full_node_name = &format!("/{ns}/tf_publisher").replace("//", "/");
    // println!("{}", format!("full ns and node name: {full_node_name}"));

    let config_file = &args2[1];
    let mut tfm = get_transforms(config_file)?;

    let nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
        .await
        .unwrap();

    // TODO(lucasw) optionally tf_static, and set to latching
    let latching = false;
    let tf_publisher = nh
        .advertise::<tf2_msgs::TFMessage>("/tf", 10, latching)
        .await
        .unwrap();

    let mut update_interval = tokio::time::interval(tokio::time::Duration::from_millis(1000));

    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        // TODO(lucasw) give the msg receiver thread a chance to cleanly finish the mcap
        println!("ctrl-c, exiting");
        std::process::exit(0);
    });

    // TODO(lucasw) optional time limit
    loop {
        update_interval.tick().await;
        let stamp = tf_util::stamp_now();

        for tf in &mut tfm.transforms {
            tf.header.stamp = stamp.clone();
        }

        tf_publisher.publish(&tfm).await?;
    }

    // Ok(())
}
