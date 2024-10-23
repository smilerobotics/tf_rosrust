use roslibrust::ros1::NodeHandle;
use tf_roslibrust::tf_util;
use tf_roslibrust::transforms::{sensor_msgs, tf2_msgs};

/// Load a toml file of a list of joint names and their properties,
/// subscribe to JointState topic and then publish the transforms
/// tf_from_joints examples/joints.toml

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Info)
        .init()
        .unwrap();

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
    // log::info!("{}", format!("full ns and node name: {full_node_name}"));

    let config_file = &args2[1];
    let joints_config = tf_util::get_joints_from_toml(config_file)?;
    for joint in &joints_config {
        log::info!("{joint:?}");
    }

    {
        let nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
            .await
            .unwrap();

        // TODO(lucasw) allow remapping
        let js_topic = format!("/{ns}/joint_states").replace("//", "/");
        let mut js_subscriber = nh
            .subscribe::<sensor_msgs::JointState>(&js_topic, 200)
            .await?;

        // TODO(lucasw) optionally tf_static, and set to latching
        let latching = false;
        let tf_publisher = nh
            .advertise::<tf2_msgs::TFMessage>("/tf", 20, latching)
            .await
            .unwrap();

        {
            let mut nh = nh.clone();
            tokio::spawn(async move {
                tokio::signal::ctrl_c().await.unwrap();
                log::info!("ctrl-c, exiting");
                // this is redundant with the unregister that happens when the nh clone goes out of
                // scope, but keep it here for now
                nh.unregister_all_subscribers().await;
            });
        }

        // TODO(lucasw) optional time limit
        loop {
            let rv = js_subscriber.next().await;
            match rv {
                Some(Ok(js)) => {
                    let tfm = tf_util::joint_states_to_tfm(&js, &joints_config);
                    if let Ok(tfm) = tfm {
                        tf_publisher.publish(&tfm).await?;
                    }
                }
                Some(Err(err)) => {
                    log::error!("{err}");
                }
                None => {
                    break;
                }
            }
        }
    }
    // wait for publisher tasks to finish
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

    Ok(())
}
