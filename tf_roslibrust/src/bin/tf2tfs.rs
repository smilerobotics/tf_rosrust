/// load toml file of tf2tf parent-child lookups and broadcast pairs, then periodically
/// look each up and broadcast them (if they have different timestamps than the last lookup)
use clap::{arg, command};
use roslibrust::ros1::NodeHandle;
use std::collections::HashMap;
use tf_roslibrust::{
    tf_util::{get_tf2tf_from_toml, tf2tf_to_tfm, to_stamp},
    transforms::tf2_msgs,
    TfListener,
};

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
                -i --input <INPUT> "input toml file with transforms to look up and republish with optional modifications"
            )
            .required(true),
        )
        .get_matches_from(unused_args);
    let config_file = matches.get_one::<String>("input").unwrap();
    println!("# loading {config_file}");

    let full_node_name = &format!("/{ns}/tf2tfs").replace("//", "/");
    // log::info!("{}", format!("full ns and node name: {full_node_name}"));

    let tf2tf_config = get_tf2tf_from_toml(config_file)?;
    // assume the broadcast_child frame is unique (otherwise there'd be multiple parents
    // TODO(lucasw) get_tf2tf_from_toml() could error on duplicate broadcast_child frames
    let mut last_timestamps = HashMap::new();
    for tf2tf in &tf2tf_config {
        log::info!("{tf2tf:?}");
        // TODO(lucasw) if tf_static is supported last won't want to use time 0 here
        last_timestamps.insert(tf2tf.broadcast_child.clone(), to_stamp(0, 0));
    }

    {
        let nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
            .await
            .unwrap();

        let tf_listener = TfListener::new(&nh).await;

        // TODO(lucasw) optionally tf_static, and set to latching
        let latching = false;
        let tf_publisher = nh
            .advertise::<tf2_msgs::TFMessage>("/tf", 10, latching)
            .await
            .unwrap();

        // TODO(lucsaw) make update rate configurable per tf2tf entry, but only down to 50Hz and
        // round to nearest 10ms?
        let mut update_interval = tokio::time::interval(tokio::time::Duration::from_millis(20));

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

        let mut error_count = 0;
        let mut no_new_count = 0;
        loop {
            update_interval.tick().await;

            if tf_listener.is_finished() {
                log::warn!("listener finished");
                break;
            }

            let (tfm, tf_errors) = tf2tf_to_tfm(&tf_listener, &tf2tf_config);
            // TODO(lucasw) look at the tf_errors, occasionally log some
            if !tf_errors.is_empty() {
                if error_count == 0 {
                    log::warn!("{tf_errors:?}");
                }
                error_count += 1;
            } else {
                if error_count > 0 {
                    log::warn!("no tf errors now after {error_count}");
                }
                error_count = 0;
            }

            // see if any of the transforms have the same timestamps as previous published ones
            // and remove them
            let mut new_tfm = tf2_msgs::TFMessage::default();
            for tfs in tfm.transforms {
                if tfs.header.stamp != *last_timestamps.get(&tfs.child_frame_id).unwrap() {
                    last_timestamps.insert(tfs.child_frame_id.clone(), tfs.header.stamp.clone());
                    new_tfm.transforms.push(tfs);
                }
            }

            if !new_tfm.transforms.is_empty() {
                tf_publisher.publish(&new_tfm).await?;
                if no_new_count > 400 {
                    log::warn!(
                        "new tfs {} after {no_new_count} updates",
                        new_tfm.transforms.len()
                    );
                }
                no_new_count = 0;
            } else {
                // expect most
                if no_new_count % 400 == 0 && no_new_count > 400 {
                    log::warn!("no new transforms to publish after {no_new_count} updates");
                }
                no_new_count += 1;
            }
        }
    }
    // wait for publisher tasks to finish
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;

    Ok(())
}
