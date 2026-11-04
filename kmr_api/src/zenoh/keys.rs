//! The gateway's key expressions (contracts/zenoh.md § Keys). No per-group key (FR-048).

/// Where each tick's sample is published: `kmr/<instance>/history`.
pub fn history(instance: &str) -> String {
    format!("kmr/{instance}/history")
}

/// The queryable serving the gateway's ring: `kmr/<instance>/history/ring`.
pub fn ring(instance: &str) -> String {
    format!("kmr/{instance}/history/ring")
}

/// Where the remote source `identity` publishes its Desired: `kmr/<instance>/desired/<identity>`.
// DECISION: one key per source under `desired/`, so the transport ACL can bind each
// certificate to its own key and the gateway reads the sender from the key.
pub fn desired(instance: &str, identity: &str) -> String {
    format!("kmr/{instance}/desired/{identity}")
}

/// Every source's Desired key: what the gateway subscribes to.
pub(crate) fn desired_any(instance: &str) -> String {
    format!("kmr/{instance}/desired/*")
}

/// The sender named by a Desired key of `instance`.
pub(crate) fn sender<'k>(key: &'k str, instance: &str) -> Option<&'k str> {
    key.strip_prefix("kmr/")?
        .strip_prefix(instance)?
        .strip_prefix("/desired/")
        .filter(|identity| is_chunk(identity))
}

/// Whether `name` is one key chunk: not empty, no `/`, no wildcard or reserved character.
pub(crate) fn is_chunk(name: &str) -> bool {
    !name.is_empty() && !name.contains(['/', '*', '$', '#', '?', ' '])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_sender_is_the_last_chunk_of_the_instances_desired_key() {
        assert_eq!(sender(&desired("arm-1", "ctl-2"), "arm-1"), Some("ctl-2"));
        assert_eq!(sender("kmr/arm-2/desired/ctl-2", "arm-1"), None);
        assert_eq!(sender("kmr/arm-1/desired/a/b", "arm-1"), None);
        assert_eq!(sender("kmr/arm-1/history", "arm-1"), None);
    }

    #[test]
    fn a_chunk_has_no_separator_nor_wildcard() {
        assert!(is_chunk("arm-1"));
        assert!(!is_chunk("") && !is_chunk("a/b") && !is_chunk("*") && !is_chunk("a$b"));
    }
}
