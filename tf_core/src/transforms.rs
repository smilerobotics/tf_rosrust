use na::geometry::{Isometry3, Translation3, UnitQuaternion};
use nalgebra as na;

use crate::{Header, Pose, Transform, TransformStamped};

pub fn isometry_from_pose(pose: impl Into<Pose>) -> Isometry3<f64> {
    let pose: Pose = pose.into();
    let trans = Translation3::new(pose.position.x, pose.position.y, pose.position.z);
    let rot = UnitQuaternion::new_normalize(nalgebra::geometry::Quaternion::new(
        pose.orientation.w,
        pose.orientation.i,
        pose.orientation.j,
        pose.orientation.k,
    ));

    Isometry3::from_parts(trans, rot)
}

pub fn isometry_from_transform(tf: impl Into<Transform>) -> Isometry3<f64> {
    let tf: Transform = tf.into();
    let trans = Translation3::new(tf.translation.x, tf.translation.y, tf.translation.z);
    let rot = UnitQuaternion::new_normalize(nalgebra::geometry::Quaternion::new(
        tf.rotation.w,
        tf.rotation.i,
        tf.rotation.j,
        tf.rotation.k,
    ));

    Isometry3::from_parts(trans, rot)
}

pub fn isometry_to_transform(iso: Isometry3<f64>) -> Transform {
    Transform {
        translation: na::Vector3::new(iso.translation.x, iso.translation.y, iso.translation.z),
        rotation: na::Quaternion::new(
            iso.rotation.i,
            iso.rotation.j,
            iso.rotation.k,
            iso.rotation.w,
        ),
    }
}

pub fn get_inverse(trans: &TransformStamped) -> TransformStamped {
    TransformStamped {
        header: Header {
            seq: 1u32,
            stamp: trans.header.stamp,
            frame_id: trans.child_frame_id.clone(),
        },
        child_frame_id: trans.header.frame_id.clone(),
        transform: isometry_to_transform(
            isometry_from_transform(&trans.transform).clone().inverse(),
        ),
    }
}

///Chain multiple transforms together. Takes in a vector of transforms. The vector should be in order of desired transformations
pub fn chain_transforms(transforms: &[Transform]) -> Transform {
    let mut final_transform = Isometry3::identity();
    for t in transforms {
        let tf = isometry_from_transform(t);
        final_transform *= tf;
    }
    isometry_to_transform(final_transform)
}

pub fn interpolate(t1: Transform, t2: Transform, weight: f64) -> Transform {
    let r1 = nalgebra::geometry::Quaternion::new(
        t1.rotation.w,
        t1.rotation.i,
        t1.rotation.j,
        t1.rotation.k,
    );
    let r2 = nalgebra::geometry::Quaternion::new(
        t2.rotation.w,
        t2.rotation.i,
        t2.rotation.j,
        t2.rotation.k,
    );
    let r1 = UnitQuaternion::from_quaternion(r1);
    let r2 = UnitQuaternion::from_quaternion(r2);
    let res = r1.try_slerp(&r2, weight, 1e-9);
    match res {
        Some(qt) => Transform {
            translation: na::Vector3::new(
                t1.translation.x * weight + t2.translation.x * (1.0 - weight),
                t1.translation.y * weight + t2.translation.y * (1.0 - weight),
                t1.translation.z * weight + t2.translation.z * (1.0 - weight),
            ),
            rotation: na::Quaternion::new(qt.coords[0], qt.coords[1], qt.coords[2], qt.coords[3]),
        },
        None => {
            if weight > 0.5 {
                Transform {
                    translation: na::Vector3::new(
                        t1.translation.x * weight + t2.translation.x * (1.0 - weight),
                        t1.translation.y * weight + t2.translation.y * (1.0 - weight),
                        t1.translation.z * weight + t2.translation.z * (1.0 - weight),
                    ),
                    rotation: t1.rotation,
                }
            } else {
                Transform {
                    translation: na::Vector3::new(
                        t1.translation.x * weight + t2.translation.x * (1.0 - weight),
                        t1.translation.y * weight + t2.translation.y * (1.0 - weight),
                        t1.translation.z * weight + t2.translation.z * (1.0 - weight),
                    ),
                    rotation: t2.rotation,
                }
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_basic_translation_chaining() {
        let tf1 = Transform {
            translation: na::Vector3::new(1f64, 1f64, 0f64),
            rotation: na::Quaternion::new(0f64, 0f64, 0f64, 1f64),
        };
        let expected_tf = Transform {
            translation: na::Vector3::new(2f64, 2f64, 0f64),
            rotation: na::Quaternion::new(0f64, 0f64, 0f64, 1f64),
        };
        let transform_chain = vec![tf1.clone(), tf1];
        let res = chain_transforms(&transform_chain);
        assert_eq!(res, expected_tf);
    }

    #[test]
    fn test_basic_interpolation() {
        let tf1 = Transform {
            translation: na::Vector3::new(1f64, 1f64, 0f64),
            rotation: na::Quaternion::new(0f64, 0f64, 0f64, 1f64),
        };
        let tf2 = Transform {
            translation: na::Vector3::new(2f64, 2f64, 0f64),
            rotation: na::Quaternion::new(0f64, 0f64, 0f64, 1f64),
        };
        let expected = Transform {
            translation: na::Vector3::new(1.5f64, 1.5f64, 0f64),
            rotation: na::Quaternion::new(0f64, 0f64, 0f64, 1f64),
        };
        assert_eq!(interpolate(tf1, tf2, 0.5), expected);
    }
}

pub(crate) fn to_transform_stamped(
    tf: Transform,
    from: std::string::String,
    to: std::string::String,
    time: rosrust::Time,
) -> TransformStamped {
    TransformStamped {
        header: Header {
            frame_id: from,
            stamp: time.into(),
            seq: 1u32,
        },
        child_frame_id: to,
        transform: tf,
    }
}
