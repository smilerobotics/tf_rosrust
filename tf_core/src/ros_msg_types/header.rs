use crate::Time;

#[cfg(feature = "ros")]
ros_nalgebra::rosmsg_include!(std_msgs / Header,);

#[derive(Debug, Clone)]
pub struct Header {
    pub seq: u32,
    pub stamp: Time,
    pub frame_id: String,
}

#[cfg(feature = "ros")]
impl From<Header> for std_msgs::Header {
    fn from(value: Header) -> Self {
        Self {
            seq: value.seq,
            stamp: value.stamp.into(),
            frame_id: value.frame_id,
        }
    }
}

#[cfg(feature = "ros")]
impl From<std_msgs::Header> for Header {
    fn from(value: std_msgs::Header) -> Self {
        Self {
            seq: value.seq,
            stamp: value.stamp.into(),
            frame_id: value.frame_id,
        }
    }
}

#[cfg(feature = "ros2")]
impl From<Header> for r2r::std_msgs::msg::Header {
    fn from(value: Header) -> Self {
        Self {
            stamp: value.stamp.into(),
            frame_id: value.frame_id,
        }
    }
}

#[cfg(feature = "ros2")]
impl From<r2r::std_msgs::msg::Header> for Header {
    fn from(value: r2r::std_msgs::msg::Header) -> Self {
        Self {
            seq: 0,
            stamp: value.stamp.into(),
            frame_id: value.frame_id,
        }
    }
}
