#[derive(Clone, Debug, Hash, Eq, PartialEq)]
pub(crate) struct TfGraphNode {
    pub(crate) child: String,
    pub(crate) parent: String,
}

#[cfg(test)]
mod test {
    use std::collections::HashMap;

    use crate::tf_graph_node::TfGraphNode;

    #[test]
    fn test() {
        let mut hash_map = HashMap::new();
        hash_map.insert(
            TfGraphNode {
                child: "child0".to_owned(),
                parent: "parent".to_owned(),
            },
            0,
        );
        assert_eq!(hash_map.len(), 1);
        hash_map.insert(
            TfGraphNode {
                child: "child0".to_owned(),
                parent: "parent".to_owned(),
            },
            1,
        );
        assert_eq!(hash_map.len(), 1);

        let mut hash_map = HashMap::new();
        hash_map.insert(
            TfGraphNode {
                child: "child0".to_owned(),
                parent: "parent0".to_owned(),
            },
            0,
        );
        assert_eq!(hash_map.len(), 1);
        hash_map.insert(
            TfGraphNode {
                child: "parent0".to_owned(),
                parent: "child0".to_owned(),
            },
            0,
        );
        assert_eq!(hash_map.len(), 2);
        hash_map.insert(
            TfGraphNode {
                child: "child0".to_owned(),
                parent: "parent1".to_owned(),
            },
            0,
        );
        assert_eq!(hash_map.len(), 3);
    }
}
