use clap::{arg, command};
use roslibrust::ros1::NodeHandle;
use tf_roslibrust::tf_util;
use tf_roslibrust::transforms::tf2_msgs;
use tf_roslibrust::LookupTransform;

/// Load a toml file of a list of parent/child frames and then find all those transforms in
/// a live system and output a new toml listing of their current values
/// tf_capture -i examples/transforms.toml > current_transforms.toml

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
    let mut unused_args = Vec::new();
    {
        // get namespace
        for arg in args {
            if arg.starts_with("__ns:=") {
                ns = arg.replace("__ns:=", "");
            } else {
                unused_args.push(arg);
            }
        }
    }

    let matches = command!()
        .arg(
            arg!(
                -i --input <INPUT> "input toml file with transforms to look for live"
            )
            .required(true),
        )
        .arg(
            arg!(
                -w --wait <WAIT> "number of seconds to collect tf data"
            )
            .default_value("3")
            .value_parser(clap::value_parser!(u64))
            .required(false),
        )
        .get_matches_from(unused_args);
    let config_file = matches.get_one::<String>("input").unwrap();
    println!("# loading {config_file}");
    let wait_seconds = *matches.get_one::<u64>("wait").unwrap();

    let full_node_name = &format!("/{ns}/tf_capture").replace("//", "/");
    // log::info!("{}", format!("full ns and node name: {full_node_name}"));

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
    println!("# waiting {wait_seconds}s for transforms");
    tokio::time::sleep(tokio::time::Duration::from_secs(wait_seconds)).await;

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
