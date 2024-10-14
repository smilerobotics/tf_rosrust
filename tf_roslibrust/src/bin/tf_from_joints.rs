use std::collections::HashMap;
use tf_roslibrust::tf_util;
use tf_roslibrust::transforms::isometry_to_transform;
use tf_roslibrust::transforms::{geometry_msgs, sensor_msgs, std_msgs, tf2_msgs};

/// Load a toml file of a list of joint names and their properties,
/// subscribe to JointState topic and then publish the transforms
/// tf_from_joints examples/joints.toml

fn joint_state_map(js: &sensor_msgs::JointState) -> HashMap<String, (f64, f64, f64)> {
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

    let full_node_name = &format!("/{ns}/tf_from_joints").replace("//", "/");
    // println!("{}", format!("full ns and node name: {full_node_name}"));

    let config_file = &args2[1];
    let joints = tf_util::get_joints_from_toml(config_file)?;

    let nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
        .await
        .unwrap();

    // TODO(lucasw) allow remapping
    let js_topic = format!("/{ns}/joint_states").replace("//", "/");
    let mut js_subscriber = nh
        .subscribe::<sensor_msgs::JointState>(&js_topic, 50)
        .await?;

    // TODO(lucasw) optionally tf_static, and set to latching
    let latching = false;
    let tf_publisher = nh
        .advertise::<tf2_msgs::TFMessage>("/tf", 10, latching)
        .await
        .unwrap();

    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        // TODO(lucasw) give the msg receiver thread a chance to cleanly finish the mcap
        println!("ctrl-c, exiting");
        std::process::exit(0);
    });

    // TODO(lucasw) optional time limit
    loop {
        let rv = js_subscriber.next().await;
        if let Some(Ok(js)) = rv {
            let joint_states = joint_state_map(&js);
            let mut tfm = tf2_msgs::TFMessage::default();
            for (joint_name, (position, _velocity, _effort)) in joint_states {
                let rv = joints.get(&joint_name);
                match rv {
                    Some(joint) => {
                        // TODO(lucasw) try joint.translation here
                        let axis_angle = joint.axis.into_inner() * position;
                        let iso = nalgebra::Isometry3::new(
                            nalgebra::base::Vector3::default(),
                            axis_angle,
                        );
                        let mut transform = isometry_to_transform(iso);
                        transform.translation.x = joint.translation.x;
                        transform.translation.y = joint.translation.y;
                        transform.translation.z = joint.translation.z;
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

            tf_publisher.publish(&tfm).await?;
        }
    }

    // Ok(())
}
