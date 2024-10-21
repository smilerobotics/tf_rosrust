use roslibrust::ros1::NodeHandle;
use tf_roslibrust::tf_util;
use tf_roslibrust::transforms::tf2_msgs;

/// Load a toml file of a list of transforms and publish them
/// tf_publisher examples/transforms.toml

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Info)
        // .without_timestamps() // required for running wsl2
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

    let full_node_name = &format!("/{ns}/tf_publisher").replace("//", "/");
    // log::info!("{}", format!("full ns and node name: {full_node_name}"));

    let config_file = &args2[1];
    let mut tfm = tf_util::get_transforms_from_toml(config_file)?;

    {
        let mut nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
            .await
            .unwrap();

        // TODO(lucasw) optionally tf_static, and set to latching
        let latching = false;
        let tf_publisher = nh
            .advertise::<tf2_msgs::TFMessage>("/tf", 10, latching)
            .await
            .unwrap();

        let mut update_interval = tokio::time::interval(tokio::time::Duration::from_millis(1000));

        // TODO(lucasw) optional time limit
        loop {
            tokio::select! {
                _ = tokio::signal::ctrl_c() => {
                    log::info!("ctrl-c, exiting");
                    break;
                }
                _ = update_interval.tick() => {
                    let stamp = tf_util::stamp_now();

                    for tf in &mut tfm.transforms {
                        tf.header.stamp = stamp.clone();
                    }

                    let rv = tf_publisher.publish(&tfm).await;
                    if rv.is_err() {
                        log::error!("{rv:?}");
                        // TODO(lucasw) also want to return not-ok
                        break;
                    }
                }
            }
        }
        // nh.inner.unregister_publisher("/tf").await;
        nh.unregister_all_subscribers().await;
    }
    // wait for publisher tasks to finish
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

    Ok(())
}
