#![allow(unreachable_pub, missing_docs, non_snake_case)]

use ros2_client::builtin_interfaces;

pub trait MessageType: Sized {
    fn message_type_name() -> ros2_client::MessageTypeName;
}
macro_rules! message_type {
    ($($package_name:ident / $type_name:ident),* $(,)?) => {$(
        impl crate::msg::MessageType for crate::msg::$package_name::$type_name {
            fn message_type_name() -> ros2_client::MessageTypeName {
                ros2_client::MessageTypeName::new(stringify!($package_name), stringify!($type_name))
            }
        }
    )*};
}
message_type!(geometry_msgs / TransformStamped, tf2_msgs / TFMessage,);

/// [std_msgs](https://github.com/ros2/common_interfaces/tree/HEAD/std_msgs)
pub mod std_msgs {
    use serde::{Deserialize, Serialize};

    use crate::msg::*;

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Header {
        pub stamp: builtin_interfaces::Time,
        pub frame_id: String,
    }
    impl Default for Header {
        fn default() -> Self {
            Self {
                stamp: builtin_interfaces::Time::ZERO,
                frame_id: Default::default(),
            }
        }
    }
}

/// [geometry_msgs](https://github.com/ros2/common_interfaces/tree/HEAD/geometry_msgs)
pub mod geometry_msgs {
    use serde::{Deserialize, Serialize};

    use crate::msg::*;

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Pose {
        pub position: Point,
        pub orientation: Quaternion,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Point {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Quaternion {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Transform {
        pub translation: geometry_msgs::Vector3,
        pub rotation: geometry_msgs::Quaternion,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct TransformStamped {
        pub header: std_msgs::Header,
        pub child_frame_id: String,
        pub transform: Transform,
    }
}

/// [tf2_msgs](https://github.com/ros2/geometry2/tree/HEAD/tf2_msgs)
pub mod tf2_msgs {
    use serde::{Deserialize, Serialize};

    use crate::msg::*;

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct TFMessage {
        pub transforms: Vec<geometry_msgs::TransformStamped>,
    }
}
