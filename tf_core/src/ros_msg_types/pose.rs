use nalgebra as na;

use crate::Point;

#[cfg(feature = "ros")]
ros_nalgebra::rosmsg_include!(geometry_msgs / Pose);

#[derive(Debug, Clone)]
pub struct Pose {
    pub position: Point,
    pub orientation: na::Quaternion<f64>,
}

#[cfg(feature = "ros")]
impl From<Pose> for geometry_msgs::Pose {
    fn from(value: Pose) -> Self {
        Self {
            position: value.position.into(),
            orientation: value.orientation.into(),
        }
    }
}

#[cfg(feature = "ros")]
impl From<geometry_msgs::Pose> for Pose {
    fn from(value: geometry_msgs::Pose) -> Self {
        Self {
            position: value.position.into(),
            orientation: value.orientation.into(),
        }
    }
}

#[cfg(feature = "ros2")]
impl From<Pose> for r2r::geometry_msgs::msg::Pose {
    fn from(value: Pose) -> Self {
        let orientation = r2r::geometry_msgs::msg::Quaternion {
            x: value.orientation.i,
            y: value.orientation.j,
            z: value.orientation.k,
            w: value.orientation.w,
        };
        Self {
            position: value.position.into(),
            orientation,
        }
    }
}

#[cfg(feature = "ros2")]
impl From<r2r::geometry_msgs::msg::Pose> for Pose {
    fn from(value: r2r::geometry_msgs::msg::Pose) -> Self {
        let orientation = na::Quaternion::new(
            value.orientation.w,
            value.orientation.x,
            value.orientation.y,
            value.orientation.z,
        );
        Self {
            position: value.position.into(),
            orientation,
        }
    }
}
