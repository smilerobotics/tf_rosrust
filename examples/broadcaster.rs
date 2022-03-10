use tf_rosrust::{TfBroadcaster, TransformStamped};

fn main() {
    rosrust::init("broadcaster");
    let broadcaster = TfBroadcaster::new();

    let rate = rosrust::rate(100.0);
    let mut tf = TransformStamped::default();
    tf.header.frame_id = "base_link".to_string();
    tf.child_frame_id = "camera".to_string();
    tf.transform.rotation.w = 1.0;
    let mut theta = 0.01_f64;
    while rosrust::is_ok() {
        theta += 0.01;
        tf.header.stamp = rosrust::now();
        tf.transform.translation.x = theta.sin();
        tf.transform.translation.y = theta.cos();
        broadcaster.send_transform(tf.clone()).unwrap();
        println!("{tf:?}");
        rate.sleep();
    }
}
