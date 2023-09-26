#[cfg(feature = "ros")]
ros_nalgebra::rosmsg_include!(geometry_msgs / Point);

#[derive(Debug, Clone, Copy)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[cfg(feature = "ros")]
impl From<Point> for geometry_msgs::Point {
    fn from(value: Point) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

#[cfg(feature = "ros")]
impl From<geometry_msgs::Point> for Point {
    fn from(value: geometry_msgs::Point) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

#[cfg(feature = "ros2")]
impl From<Point> for r2r::geometry_msgs::msg::Point {
    fn from(value: Point) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

#[cfg(feature = "ros2")]
impl From<r2r::geometry_msgs::msg::Point> for Point {
    fn from(value: r2r::geometry_msgs::msg::Point) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}
