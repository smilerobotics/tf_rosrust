use tf_roslibrust::tf_util;

roslibrust_codegen_macro::find_and_generate_ros_messages!();

/// Load a toml file of a list of trnasforms and publish them

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
        // TODO(lucasw) if there is a rosnode kill the listener will stop receiving messages
        // but this loop keeps going
        update_interval.tick().await;
        let stamp = tf_util::stamp_now();

        let tfm = {
            let mut transform = geometry_msgs::TransformStamped::default();
            transform.header.stamp = stamp;
            transform.header.frame_id = "odom".to_string();
            transform.child_frame_id = "test".to_string();
            transform.transform.translation.z = 1.0;
            transform.transform.rotation.w = 1.0;

            let mut tfm = tf2_msgs::TFMessage::default();
            tfm.transforms.push(transform);
            tfm
        };
        // println!("{tfm:?}");
        tf_publisher.publish(&tfm).await?;
    }

    // Ok(())
}
