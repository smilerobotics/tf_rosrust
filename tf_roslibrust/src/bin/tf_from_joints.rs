use clap::{arg, command};
use roslibrust::ros1::NodeHandle;
use roslibrust_util::get_params_remaps;
use roslibrust_util::{sensor_msgs, tf2_msgs};
use std::collections::HashMap;
use tf_roslibrust::tf_util;

/// Load a toml file of a list of joint names and their properties,
/// subscribe to JointState topic and then publish the transforms
/// tf_from_joints examples/joints.toml

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Info)
        .init()
        .unwrap();

    let (full_node_name, unused_args, remaps) = {
        let mut params = HashMap::<String, String>::new();
        params.insert("_name".to_string(), "tf_fron_joints".to_string());
        let mut remaps = HashMap::<String, String>::new();
        remaps.insert("joint_states".into(), "joint_states".into());
        remaps.insert("tf".into(), "/tf".into());
        let (_ns, full_node_name, unused_args) = get_params_remaps(&mut params, &mut remaps);
        (full_node_name, unused_args, remaps)
    };

    let master_uri =
        std::env::var("ROS_MASTER_URI").unwrap_or("http://localhost:11311".to_string());

    let matches = command!()
        .arg(
            arg!(
                -i --input <INPUT> "input toml file with transforms to look up and republish with optional modifications"
            )
            .required(true),
        )
        .get_matches_from(unused_args);
    let config_file = matches.get_one::<String>("input").unwrap();
    println!("# loading {config_file}");

    let joints_config = tf_util::get_joints_from_toml(config_file)?;
    for joint in &joints_config {
        log::info!("{joint:?}");
    }

    {
        let nh = NodeHandle::new(&master_uri, &full_node_name).await.unwrap();

        // TODO(lucasw) allow remapping
        let js_topic = remaps.get("joint_states").unwrap();
        let mut js_subscriber = nh
            .subscribe::<sensor_msgs::JointState>(js_topic, 200)
            .await?;

        let tf_topic = remaps.get("tf").unwrap();
        // TODO(lucasw) optionally tf_static, and set to latching
        let latching = false;
        let tf_publisher = nh
            .advertise::<tf2_msgs::TFMessage>(tf_topic, 20, latching)
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
                        tf_publisher.publish(&tfm)?;
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
    // TODO(lucasw) the publisher isn't reliably getting unregistered
    // wait for publisher tasks to finish
    tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;

    Ok(())
}
