use std::collections::{HashMap, HashSet};
use tf_roslibrust::TfListener;

roslibrust_codegen_macro::find_and_generate_ros_messages!();

/// print the current tf tree
/// adapted from tf_demo tf_tree.py

fn print_tree(
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
    println!("{parent}");

    match parent_to_children.get(parent) {
        Some(children) => {
            let mut children = children.iter().collect::<Vec<_>>();
            children.sort();
            for child in children {
                let _ = print_tree(parent_to_children, child, level + 1);
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

    let full_node_name = &format!("/{ns}/tree").replace("//", "/");
    // println!("{}", format!("full ns and node name: {full_node_name}"));

    let nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
        .await
        .unwrap();

    let listener = TfListener::new(&nh).await;

    // let some transforms arrive
    tokio::time::sleep(tokio::time::Duration::from_secs(3)).await;

    {
        // TODO(lucasw) make tf listener stop listening
        let buffer = listener.buffer.read().unwrap();
        let root = buffer.get_root()?;
        println!("\n");
        let parent_to_children = buffer.get_parent_to_children();
        {
            let mut keys_sorted = parent_to_children.keys().collect::<Vec<_>>();
            keys_sorted.sort();
            for parent in keys_sorted {
                println!("{parent}: {:?}", parent_to_children.get(parent).unwrap());
            }
        }
        let _ = print_tree(&parent_to_children, &root, 0);
    }

    Ok(())
}
