use tf_roslibrust::tf_util;
use tf_roslibrust::tf_util::tf2_msgs;

/// Load a toml file of a list of transforms and publish them
/// tf_publisher examples/transforms.toml

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
    let mut tfm = tf_util::get_transforms_from_toml(config_file)?;

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
