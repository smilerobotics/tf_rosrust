use nalgebra as na;

use crate::Header;

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

#[cfg(feature = "ros2")]
impl From<TransformStamped> for r2r::geometry_msgs::msg::TransformStamped {
    fn from(value: TransformStamped) -> Self {
        let transform = r2r::geometry_msgs::msg::Transform {
            translation: r2r::geometry_msgs::msg::Vector3 {
                x: value.transform.translation.x,
                y: value.transform.translation.y,
                z: value.transform.translation.z,
            },
            rotation: r2r::geometry_msgs::msg::Quaternion {
                x: value.transform.rotation.i,
                y: value.transform.rotation.j,
                z: value.transform.rotation.k,
                w: value.transform.rotation.w,
            },
        };
        Self {
            header: value.header.into(),
            child_frame_id: value.child_frame_id,
            transform,
        }
    }
}

#[cfg(feature = "ros2")]
impl From<r2r::geometry_msgs::msg::TransformStamped> for TransformStamped {
    fn from(value: r2r::geometry_msgs::msg::TransformStamped) -> Self {
        let tran = value.transform.translation;
        let rot = value.transform.rotation;
        Self {
            header: value.header.into(),
            child_frame_id: value.child_frame_id,
            transform: na::Isometry3::new(
                na::Vector3::new(tran.x, tran.y, tran.z),
                na::Vector3::new(rot.x, rot.y, rot.z),
            ),
        }
    }
}
