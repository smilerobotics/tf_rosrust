use tf_roslibrust::LookupTransform;
use tf_roslibrust::{tf_util, TfListener};

roslibrust_codegen_macro::find_and_generate_ros_messages!();

/// Take in a source and destination frame argument
/// and repeatedly print the transform between them if any

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
    println!("{args2:?}");
    let frame1 = &args2[1];
    let frame2 = &args2[2];
    println!("lookup up '{frame1}' to '{frame2}'");

    let full_node_name = &format!("/{ns}/echo").replace("//", "/");
    // println!("{}", format!("full ns and node name: {full_node_name}"));

    let nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
        .await
        .unwrap();

    let listener = TfListener::new(&nh).await;

    let mut update_interval = tokio::time::interval(tokio::time::Duration::from_millis(1000));

    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        // TODO(lucasw) give the msg receiver thread a chance to cleanly finish the mcap
        println!("ctrl-c, exiting");
        tokio::time::sleep(tokio::time::Duration::from_millis(1000)).await;
        std::process::exit(0);
    });

    // println!("tf loop");
    loop {
        // TODO(lucasw) if there is a rosnode kill the listener will stop receiving messages
        // but this loop keeps going
        update_interval.tick().await;

        // TODO(lucasw) maybe a lookup could return a special error to signal the listener is done
        if listener.is_finished() {
            break;
        }

        // println!("update {remaining:?}");
        // let lookup_stamp = tf_util::stamp_now();
        // TODO(lucasw) maybe just have a lookup_transform_recent function
        // TODO(lucasw) swapping position of frame 1 2 to match tf2_tools echo.py
        // let t0 = tf_util::duration_now();
        let res = listener.lookup_transform(frame1, frame2, None);
        let t1 = tf_util::duration_now();
        match res {
            Ok(tf) => {
                // let tdiff = tf_util::duration_to_f64(t1 - t0);
                // println!("lookup time {:?}s", t1 - t0);
                let t1 = tf_util::duration_to_f64(t1);
                let lookup_time = tf_util::stamp_to_f64(&tf.header.stamp);
                println!(
                    "\nAt time {lookup_time:.3}, (current time {t1:.3}, {:.3}s old)",
                    t1 - lookup_time
                );
                println!("frame {} -> {}", tf.header.frame_id, tf.child_frame_id);
                let xyz = tf.transform.translation;
                println!("- Translation: [{:.3} {:.3} {:.3}]", xyz.x, xyz.y, xyz.z);
                let quat = tf.transform.rotation;
                println!(
                    "- Rotation: in Quaternion [{:.6} {:.6} {:.6} {:.6}]",
                    quat.x, quat.y, quat.z, quat.w
                );

                // nalgebra documentation ought to say what components are in what place in this
                // constructor
                let unit_quat = nalgebra::UnitQuaternion::from_quaternion(
                    nalgebra::Quaternion::new(quat.w, quat.x, quat.y, quat.z),
                );
                let (roll, pitch, yaw) = unit_quat.euler_angles();
                println!("            in RPY (radian) [{roll:.6} {pitch:.6} {yaw:.6}]",);
            }
            Err(err) => {
                println!("{t1:?} {err:?}");
            }
        }
        // TODO(lucasw) publishing a dynamic transform followed by a static (for
        // the same parent and child frames) results in CouldNotFindTransform error
        // but doing the reverse works, but then turning off the dynamic results in
        // the lookup still working- the parent-child relationship gets latched in
        // as static or dynamic and later update don't change it.
        // Compare to old tf_echo and tf2_tools echo.py
        // println!("{stamp_now:?} {lookup_stamp:?} {tf:?}");
    }

    Ok(())
}
