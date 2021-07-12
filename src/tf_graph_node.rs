#[derive(Clone, Debug, Hash)]
pub(crate) struct TfGraphNode {
    pub(crate) child: String,
    pub(crate) parent: String,
}

impl PartialEq for TfGraphNode {
    fn eq(&self, other: &Self) -> bool {
        self.child == other.child && self.parent == other.parent
    }
}

impl Eq for TfGraphNode {}
