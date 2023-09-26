use nalgebra as na;

use crate::geometry_msgs;

#[derive(Debug, Clone, PartialEq)]
pub struct Transform {
    pub translation: na::Vector3<f64>,
    pub rotation: na::Quaternion<f64>,
}

#[cfg(feature = "ros")]
impl From<Transform> for geometry_msgs::Transform {
    fn from(value: Transform) -> Self {
        Self {
            translation: value.translation.into(),
            rotation: value.rotation.into(),
        }
    }
}

#[cfg(feature = "ros")]
impl From<geometry_msgs::Transform> for Transform {
    fn from(value: geometry_msgs::Transform) -> Self {
        Self {
            translation: value.translation.into(),
            rotation: value.rotation.into(),
        }
    }
}
