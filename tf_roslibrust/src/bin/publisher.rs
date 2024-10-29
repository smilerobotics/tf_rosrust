use clap::{arg, command};
use roslibrust::ros1::NodeHandle;
use tf_roslibrust::tf_util;
use tf_roslibrust::transforms::tf2_msgs;

/// Load a toml file of a list of transforms and publish them
/// tf_publisher -i examples/transforms.toml

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
                -i --input <INPUT> "input toml file with transforms to publish"
            )
            .required(true),
        )
        .arg(
            arg!(
                -w --wait <WAIT> "number of milliseconds between publishing tf data"
            )
            .default_value("1000")
            .value_parser(clap::value_parser!(u64))
            .required(false),
        )
        .get_matches_from(unused_args);
    let config_file = matches.get_one::<String>("input").unwrap();
    let wait_millis = *matches.get_one::<u64>("wait").unwrap();
    println!("# loading {config_file}, waiting {wait_millis}ms between publishes");

    let full_node_name = &format!("/{ns}/tf_publisher").replace("//", "/");
    // log::info!("{}", format!("full ns and node name: {full_node_name}"));

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

        let mut update_interval =
            tokio::time::interval(tokio::time::Duration::from_millis(wait_millis));

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
