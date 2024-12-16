use anyhow::{Context, Result};
use roslibrust::ros1;
use roslibrust_util::nav_msgs;
use std::collections::HashMap;

#[tokio::main]
async fn main() -> Result<()> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Info)
        .init()
        .unwrap();

    let mut params = HashMap::<String, String>::new();
    params.insert("_name".to_string(), "odom_latest".to_string());
    let mut remaps = HashMap::<String, String>::new();
    remaps.insert("odom".into(), "odom".into());
    let (_ns, full_node_name, _remaining_args) =
        roslibrust_util::get_params_remaps(&mut params, &mut remaps);

    let master_uri =
        std::env::var("ROS_MASTER_URI").unwrap_or("http://localhost:11311".to_string());

    let odom_topic = remaps.get("odom").context("no odom remap")?;

    let nh = ros1::NodeHandle::new(&master_uri, &full_node_name).await?;

    {
        let odom_sub = nh.subscribe::<nav_msgs::Odometry>(odom_topic, 20).await?;

        let latest_odom = roslibrust_util::LatestFromSubscriber::new("odom", odom_sub);

        let mut update_interval = tokio::time::interval(tokio::time::Duration::from_millis(1000));

        loop {
            tokio::select! {
                _ = tokio::signal::ctrl_c() => {
                    println!("ctrl-c, exiting");
                    break;
                }
                _ = update_interval.tick() => {
                    let odom = latest_odom.latest.lock().unwrap().clone();
                    match odom {
                        Some(odom) => {
                            log::info!("{odom:?}");
                        }
                        None => {
                            log::info!("no odom yet");
                        }
                    }
                }
            }
        }
    }

    Ok(())
}
