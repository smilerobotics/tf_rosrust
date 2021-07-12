use std::cmp::Ordering;

use crate::transforms::geometry_msgs::TransformStamped;

#[derive(Clone, Debug)]
pub(crate) struct OrderedTF {
    pub(crate) tf: TransformStamped,
}

impl PartialEq for OrderedTF {
    fn eq(&self, other: &Self) -> bool {
        self.tf.header.stamp == other.tf.header.stamp
    }
}

impl Eq for OrderedTF {}

impl Ord for OrderedTF {
    fn cmp(&self, other: &Self) -> Ordering {
        self.tf.header.stamp.cmp(&other.tf.header.stamp)
    }
}

impl PartialOrd for OrderedTF {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.tf.header.stamp.cmp(&other.tf.header.stamp))
    }
}
