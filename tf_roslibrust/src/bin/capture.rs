use clap::{arg, command};
use roslibrust::ros1::NodeHandle;
use roslibrust_util::get_params_remaps;
use roslibrust_util::tf2_msgs;
use std::collections::HashMap;
use tf_roslibrust::tf_util;
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

    let (full_node_name, unused_args, _remaps) = {
        let mut params = HashMap::<String, String>::new();
        params.insert("_name".to_string(), "tf_capture".to_string());
        let mut remaps = HashMap::<String, String>::new();
        let (_ns, full_node_name, unused_args) = get_params_remaps(&mut params, &mut remaps);
        (full_node_name, unused_args, remaps)
    };

    let master_uri =
        std::env::var("ROS_MASTER_URI").unwrap_or("http://localhost:11311".to_string());

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

    let old_tfm = tf_util::get_transforms_from_toml(config_file)?;

    let mut nh = NodeHandle::new(&master_uri, &full_node_name).await?;

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
