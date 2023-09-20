use nalgebra as na;
#[cfg(feature = "ros")]
ros_nalgebra::rosmsg_include!(
    geometry_msgs / Transform,
    geometry_msgs / Pose,
    geometry_msgs / Vector3,
    geometry_msgs / Quaternion,
    geometry_msgs / TransformStamped,
    std_msgs / Header,
    tf2_msgs / TFMessage
);

#[derive(Debug, Clone, Copy)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

#[cfg(feature = "ros")]
impl From<Time> for rosrust::Time {
    fn from(value: Time) -> Self {
        Self {
            sec: value.sec as u32,
            nsec: value.nanosec,
        }
    }
}

#[cfg(feature = "ros")]
impl From<rosrust::Time> for Time {
    fn from(value: rosrust::Time) -> Self {
        Self {
            sec: value.sec as i32,
            nanosec: value.nsec,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Duration {
    pub sec: i32,
    pub nanosec: u32,
}

#[cfg(feature = "ros")]
impl From<Duration> for rosrust::Duration {
    fn from(value: Duration) -> Self {
        Self {
            sec: value.sec,
            nsec: value.nanosec as i32,
        }
    }
}

#[cfg(feature = "ros")]
impl From<rosrust::Duration> for Duration {
    fn from(value: rosrust::Duration) -> Self {
        Self {
            sec: value.sec,
            nanosec: value.nsec as u32,
        }
    }
}

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

#[derive(Debug, Clone)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: String,
    pub transform: na::Isometry3<f64>,
}

#[cfg(feature = "ros")]
impl From<TransformStamped> for geometry_msgs::TransformStamped {
    fn from(value: TransformStamped) -> Self {
        Self {
            header: value.header.into(),
            child_frame_id: value.child_frame_id,
            transform: value.transform.into(),
        }
    }
}

#[cfg(feature = "ros")]
impl From<geometry_msgs::TransformStamped> for TransformStamped {
    fn from(value: geometry_msgs::TransformStamped) -> Self {
        let translation = value.transform.translation.into();
        let u_q = na::UnitQuaternion::from_quaternion(na::Quaternion::new(
            value.transform.rotation.w,
            value.transform.rotation.x,
            value.transform.rotation.y,
            value.transform.rotation.z,
        ));
        let angles = u_q.euler_angles();
        let rotation = na::Vector3::new(angles.0, angles.1, angles.2);
        Self {
            header: value.header.into(),
            child_frame_id: value.child_frame_id,
            transform: na::Isometry3::new(translation, rotation),
        }
    }
}

#[derive(Debug, Clone)]
pub struct TFMessage {
    pub transforms: Vec<TransformStamped>,
}

#[cfg(feature = "ros")]
impl From<TFMessage> for tf2_msgs::TFMessage {
    fn from(value: TFMessage) -> Self {
        let mut tf_message_inner = vec![];
        for t in value.transforms {
            tf_message_inner.push(t.into());
        }
        tf2_msgs::TFMessage {
            transforms: tf_message_inner,
        }
    }
}

#[cfg(feature = "ros")]
impl From<tf2_msgs::TFMessage> for TFMessage {
    fn from(value: tf2_msgs::TFMessage) -> Self {
        let mut tf_message_inner = vec![];
        for t in value.transforms {
            tf_message_inner.push(t.into());
        }
        Self {
            transforms: tf_message_inner,
        }
    }
}
