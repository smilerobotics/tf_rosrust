/// load toml file of tf2tf parent-child lookups and broadcast pairs, then periodically
/// look each up and broadcast them (if they have different timestamps than the last lookup)
use roslibrust::ros1::NodeHandle;
use tf_roslibrust::{
    tf_util::{get_tf2tf_from_toml, tf2tf_to_tfm},
    transforms::tf2_msgs,
    TfListener,
};

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
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

    let full_node_name = &format!("/{ns}/tf2tfs").replace("//", "/");
    // println!("{}", format!("full ns and node name: {full_node_name}"));

    let config_file = &args2[1];
    let tf2tf_config = get_tf2tf_from_toml(config_file)?;

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

    let mut update_interval = tokio::time::interval(tokio::time::Duration::from_millis(100));

    // TODO(lucasw) maybe can do something with cancel tokens, and have the recv in the tf listener
    // get cancelled when the ctrl-c arrives?
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        // TODO(lucasw) give the msg receiver thread a chance to cleanly finish the mcap
        println!("ctrl-c, exiting");
        std::process::exit(0);
    });

    loop {
        update_interval.tick().await;

        let (tfm, _tf_errors) = tf2tf_to_tfm(&tf_listener, &tf2tf_config);
        // TODO(lucasw) look at the tf_errors, occasionally log some

        if !tfm.transforms.is_empty() {
            tf_publisher.publish(&tfm).await?;
        }
    }

    // Ok(())
}
