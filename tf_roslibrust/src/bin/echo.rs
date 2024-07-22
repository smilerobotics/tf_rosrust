use tf_roslibrust::TfListener;
use roslibrust_codegen::Time;

/// Take in a source and destination frame argument
/// and repeatedly print the transform between them if any

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use roslibrust::ros1::NodeHandle;

    // need to have leading slash on node name and topic to function properly
    // so figure out namespace then prefix it to name and topics
    let mut ns = String::from("");
    let args = std::env::args();
    {
        // get namespace
        for arg in args {
            if arg.starts_with("__ns:=") {
               ns = arg.replace("__ns:=", "");
            }
        }
    }

    let full_node_name = &format!("/{ns}/echo").replace("//", "/");
    println!("{}", format!("full ns and node name: {full_node_name}"));

    let nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
        .await.unwrap();

    let frame1 = "map";
    let frame2 = "base_link";

    let listener = TfListener::new(&nh);

    tokio::select! {
        _ = async { loop {
            listener.await.update_tf();
        }} => {},
        _ = async { loop {
            listener.await.update_tf_static();
        }} => {},
        _ = async { loop {
            let tf = listener.await.lookup_transform(frame1, frame2, Time {secs: 0, nsecs: 0}).unwrap();
            println!("{tf:?}");

            tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
        }} => {},
    }

    Ok(())
}
