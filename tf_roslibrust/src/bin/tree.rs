/// Copyright 2024 Lucas WAlter
/// BSD 3-Clause
///
/// show a text version of the ros tf tree as received on /tf and /tf_static
use clap::{arg, command};
use roslibrust::ros1::NodeHandle;
use std::collections::{HashMap, HashSet};
use tf_roslibrust::{TfBuffer, TfListener};

/// print the current tf tree
/// adapted from tf_demo tf_tree.py

fn print_tree(
    tf_buffer: &TfBuffer,
    parent_to_children: &HashMap<String, HashSet<String>>,
    parent: &str,
    level: u8,
) -> Result<(), anyhow::Error> {
    if level > 1 {
        for _ in 0..level {
            print!("   ");
        }
    }
    if level > 0 {
        print!("-- ");
    }

    print!("{parent}");
    let rate = tf_buffer.get_rate(parent);
    if let Some(rate) = rate {
        if rate > 0.0 {
            print!("  {rate:.2}Hz");
        }
    }
    println!();

    match parent_to_children.get(parent) {
        Some(children) => {
            let mut children = children.iter().collect::<Vec<_>>();
            children.sort();
            for child in children {
                let _ = print_tree(tf_buffer, parent_to_children, child, level + 1);
            }
        }
        None => {
            // no children
            return Ok(());
        }
    }

    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Info)
        .init()
        .unwrap();

    // TODO(lucasw) ought to have mcap_tools mpsisc::get_params_remaps()
    // but mcap_tools depends on tf_roslibrust, so can't be circular-
    // instead need to move non-mcap utilities into another crate
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
        let parent_to_children = buffer.get_parent_to_children();
        if false {
            let mut keys_sorted = parent_to_children.keys().collect::<Vec<_>>();
            keys_sorted.sort();
            for parent in keys_sorted {
                println!("{parent}: {:?}", parent_to_children.get(parent).unwrap());
            }
        }

        let roots = buffer.get_roots()?;
        for root in roots {
            println!("[");
            let _ = print_tree(&buffer, &parent_to_children, &root, 0);
            println!("]");
        }
    }

    // TODO(lucasw) wait for async unregistering
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    Ok(())
}
