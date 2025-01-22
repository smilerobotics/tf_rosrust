//importing in execute! macro
#[macro_use]
extern crate crossterm;

use anyhow::Context;
use clap::command;
use crossterm::cursor;
use crossterm::event::{read, Event, KeyCode, KeyEvent, KeyModifiers};
use crossterm::style::Print;
use crossterm::terminal::{disable_raw_mode, Clear, ClearType};
use roslibrust::ros1::NodeHandle;
use roslibrust_util::sensor_msgs;
use std::collections::HashMap;
use std::io::stdout;

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    // don't want params and remaps to be mutable after init is done
    let (nh, _full_node_name, _params, remaps) = {
        let mut params = HashMap::<String, String>::new();
        params.insert("_name".to_string(), "key_to_joy".to_string());
        let mut remaps = HashMap::<String, String>::new();
        remaps.insert("joy".to_string(), "joy".to_string());

        let (_ns, full_node_name, remaining_args) =
            roslibrust_util::get_params_remaps(&mut params, &mut remaps);

        // using clap only for version reporting currently
        let _matches = command!().get_matches_from(remaining_args);

        let ros_master_uri =
            std::env::var("ROS_MASTER_URI").unwrap_or("http://localhost:11311".to_string());
        let nh = NodeHandle::new(&ros_master_uri, &full_node_name).await?;
        // log::info!("connected to roscore at {ros_master_uri}");

        (nh, full_node_name, params, remaps)
    };

    let joy_topic = remaps.get("joy").context("no joy topic found")?;
    let joy_pub = nh
        .advertise::<sensor_msgs::Joy>(joy_topic, 8, false)
        .await?;

    let mut stdout = stdout();
    // going into raw mode
    // clearing the screen, going to top left corner and printing welcoming message
    execute!(stdout, Clear(ClearType::All), cursor::MoveTo(0, 0), Print(r#"ctrl + c to exit, ctrl + h to print "Hello world", alt + t to print "crossterm is cool""#))
            .unwrap();

    /*
    x-box
    0 is left-right on the left stick
    1 is up-down on the left stick
    2 the left trigger
    5 is the right trigger
    */
    let mut joy = sensor_msgs::Joy {
        axes: vec![0.0; 5],
        ..Default::default()
    };

    // key detection
    loop {
        // going to top left corner
        execute!(stdout, cursor::MoveTo(0, 0)).unwrap();

        match read().unwrap() {
            Event::Key(KeyEvent {
                code: KeyCode::Char('h'),
                modifiers: KeyModifiers::CONTROL,
                //clearing the screen and printing our message
                ..
            }) => execute!(stdout, Clear(ClearType::All), Print("Hello world!")).unwrap(),
            Event::Key(KeyEvent {
                code: KeyCode::Char('a'),
                ..
            }) => {
                joy.axes[0] -= 0.1;
                joy.axes[0] = joy.axes[0].clamp(-1.0, 1.0);
                execute!(
                    stdout,
                    Clear(ClearType::All),
                    Print(format!("left {}", joy.axes[0]))
                )
                .unwrap();
            }
            Event::Key(KeyEvent {
                code: KeyCode::Char('d'),
                ..
            }) => {
                joy.axes[0] += 0.1;
                joy.axes[0] = joy.axes[0].clamp(-1.0, 1.0);
                execute!(
                    stdout,
                    Clear(ClearType::All),
                    Print(format!("right {}", joy.axes[0]))
                )
                .unwrap();
            }
            Event::Key(KeyEvent {
                code: KeyCode::Char('c'),
                modifiers: KeyModifiers::CONTROL,
                ..
            }) => break,
            _ => (),
        }

        joy_pub.publish(&joy).await?;
    }

    // disabling raw mode
    disable_raw_mode().unwrap();

    Ok(())
}
