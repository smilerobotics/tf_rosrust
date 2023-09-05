# tf_rosrust

[![Build Status](https://img.shields.io/github/actions/workflow/status/smilerobotics/tf_rosrust/ci.yml?branch=main&logo=github)](https://github.com/smilerobotics/tf_rosrust/actions) [![crates.io](https://img.shields.io/crates/v/tf_rosrust.svg?logo=rust)](https://crates.io/crates/tf_rosrust) [![docs](https://docs.rs/tf_rosrust/badge.svg)](https://docs.rs/tf_rosrust)

This project is forked from [arjo129 rustros_tf](https://github.com/arjo129/rustros_tf),[MaxiMaerz rustros_tf](https://github.com/MaxiMaerz/rustros_tf).

- changes from original code
    - fix transformations by MaxiMaerz
    - contains ros msg files to run without ros installation
    - fix dynamic tf handling

This is a rust port of the [ROS tf library](http://wiki.ros.org/tf). It is intended for being used in robots to help keep track of multiple coordinate frames and is part of a larger suite of rust libraries that provide support for various robotics related functionality.


## Features
So far the only the following have been implemented:
* `TfListener` with `lookup_transform` and time traversal.
* `TfBroadcaster` to publish `/tf`

I am still working on the following:
* More efficient cache data structure.
* Weed out `unwrap()`s

## Supported platforms
Currently only Ubuntu 18.04 running ROS Melodic on x86_64 is tested. It should work on any linux based system with a proper ROS installation.

## Getting Started
Install [ROS](http://wiki.ros.org/melodic/Installation) first. On ubuntu, this can be done like so:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop
```

After installing ROS, you may simply add this crate as a dependency to your cargo project:

```toml
[dependencies]
tf_rosrust = "0.0.5"
```

## Third party software

This product includes copies and modifications of software developed by third parties:

- [`ros_msgs`](ros_msgs) includes copies and modifications of msg packages by [ros](https://github.com/ros) and [tf2_msgs](https://github.com/ros/geometry2), licensed under the 2-Clause BSD License.

See the license files included in these directories for more details.

# Example usage
The following example shows a simple lookup.

```rust
use tf_rosrust::TfListener;

fn main() {
    rosrust::init("listener");
    let listener = TfListener::new();

    let rate = rosrust::rate(1.0);
    while rosrust::is_ok() {
        let tf = listener.lookup_transform("camera", "base_link", rosrust::Time::new());
        println!("{tf:?}");
        rate.sleep();
    }
}
```
