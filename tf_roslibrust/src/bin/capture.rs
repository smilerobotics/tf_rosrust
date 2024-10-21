use roslibrust::ros1::NodeHandle;
use tf_roslibrust::tf_util;
use tf_roslibrust::transforms::tf2_msgs;
use tf_roslibrust::LookupTransform;

/// Load a toml file of a list of parent/child frames and then find all those transforms in
/// a live system and output a new toml listing of their current values
/// tf_capture examples/transforms.toml > current_transforms.toml

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

    let full_node_name = &format!("/{ns}/tf_capture").replace("//", "/");
    // log::info!("{}", format!("full ns and node name: {full_node_name}"));

    let config_file = &args2[1];
    let old_tfm = tf_util::get_transforms_from_toml(config_file)?;

    let mut nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
        .await
        .unwrap();

    println!(
        "# {:.3} getting transforms...",
        tf_util::stamp_to_f64(&tf_util::stamp_now())
    );
    let listener = tf_roslibrust::TfListener::new(&nh).await;

    // let some transforms arrive
    // TODO(lucasw) make this a param or regular cli arg
    let gather_tf_seconds = 3;
    println!("# waiting {gather_tf_seconds}s for transforms");
    tokio::time::sleep(tokio::time::Duration::from_secs(gather_tf_seconds)).await;

    let mut new_tfm = tf2_msgs::TFMessage::default();
    for old_tfs in &old_tfm.transforms {
        // TODO(lucasw) make a lookup_most_recent_transform and then don't need the None
        let res =
            listener.lookup_transform(&old_tfs.header.frame_id, &old_tfs.child_frame_id, None);
        match res {
            Ok(new_tfs) => {
                // log::info!("{new_tfs:?}");
                new_tfm.transforms.push(new_tfs);
                // TODO(lucasw) compare old values to current values, warn if large
            }
            Err(err) => {
                eprintln!("# {err:?} -> going to use old version for this transform");
                new_tfm.transforms.push(old_tfs.clone());
            }
        }
    }

    let toml = tf_util::transforms_to_toml(new_tfm)?;
    print!("\n\n{toml}");

    // manual cleanup
    nh.unregister_all_subscribers().await;

    Ok(())
}
