/// Copyright 2024 Lucas WAlter
/// BSD 3-Clause
///
/// show a text version of the ros tf tree as received on /tf and /tf_static
use clap::{arg, command};
use roslibrust::ros1::NodeHandle;
use tf_roslibrust::tf_util;
use tf_roslibrust::TfListener;

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Info)
        .init()
        .unwrap();

    // TODO(lucasw) ought to have mcap_tools misc::get_params_remaps()
    // but want to avoid that dependency
    // instead need to move non-mcap utilities into another crate (could be in same repo)
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

    // don't have any args, but want --version to work
    let matches = command!()
        .arg(
            arg!(
                -w --wait <WAIT> "number of seconds to collect tf data"
            )
            .default_value("3")
            .value_parser(clap::value_parser!(u64))
            .required(false),
        )
        .get_matches_from(unused_args);

    let wait_seconds = *matches.get_one::<u64>("wait").unwrap();

    let full_node_name = &format!("/{ns}/tree").replace("//", "/");
    // println!("{}", format!("full ns and node name: {full_node_name}"));

    {
        let nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
            .await
            .unwrap();

        let listener = TfListener::new(&nh).await;

        println!("collecting tf data for {wait_seconds} seconds...");
        tokio::time::sleep(tokio::time::Duration::from_secs(wait_seconds)).await;
        // now done collecting
        listener.force_finish();

        let buffer = listener.buffer.read().unwrap();
        let _ = tf_util::print_tree(&buffer);
    }

    // TODO(lucasw) wait for async unregistering
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    Ok(())
}
