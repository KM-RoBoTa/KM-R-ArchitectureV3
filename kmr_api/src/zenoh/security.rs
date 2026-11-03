//! Who may talk to the gateway (SRS FR-13.2, NFR-12): the identity's transport (mutual TLS
//! from files, or loopback), the transport ACL that denies every key by default, and the
//! Desired rule the gateway thread applies again after the transport authenticated the peer.

use std::fmt::Write as _;

use super::{GatewayError, keys};

/// How the gateway authenticates and is authenticated. Holds paths only: the certificate and
/// the key stay files on the machine, never in source or in the descriptor (FR-060).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Identity {
    /// Plain TCP on the loopback interface only: every endpoint must be `tcp/127.0.0.1:..`,
    /// `tcp/[::1]:..` or `tcp/localhost:..`. No certificate: any process of this machine is
    /// trusted to name itself in its Desired key; no transport ACL.
    // DECISION: a loopback identity for one-machine setups and tests; it refuses any other
    // endpoint, so it never opens the robot to the network.
    Loopback,
    /// Mutual TLS: this machine's certificate and key, and the CA that signs every peer's.
    /// Every endpoint must be `tls/..`. A peer's identity is its certificate's common name.
    Files {
        /// This machine's certificate (PEM).
        cert: &'static str,
        /// This machine's private key (PEM).
        key: &'static str,
        /// The CA certificate every peer's certificate must chain to (PEM).
        ca: &'static str,
    },
}

impl Identity {
    /// The loopback identity: plain TCP, loopback endpoints only.
    pub const fn loopback() -> Self {
        Self::Loopback
    }

    /// Mutual TLS from the files at `cert`, `key` and `ca`; their content is read by the
    /// transport when the run starts.
    pub const fn files(cert: &'static str, key: &'static str, ca: &'static str) -> Self {
        Self::Files { cert, key, ca }
    }
}

/// Everything the transport needs besides the endpoints: settings as `(key, JSON5 value)`.
pub(crate) fn transport(
    identity: &Identity,
    instance: &str,
    read_history: &[&str],
    sources: &[&str],
    endpoints: &[&String],
) -> Result<Vec<(&'static str, String)>, GatewayError> {
    match identity {
        Identity::Loopback => {
            if let Some(endpoint) = endpoints.iter().find(|e| !is_loopback(e)) {
                return Err(GatewayError::NotLoopback {
                    endpoint: (*endpoint).clone(),
                });
            }
            Ok(Vec::new())
        }
        Identity::Files { cert, key, ca } => {
            if let Some(endpoint) = endpoints.iter().find(|e| !e.starts_with("tls/")) {
                return Err(GatewayError::NotTls {
                    endpoint: (*endpoint).clone(),
                });
            }
            for path in [cert, key, ca] {
                if path.contains(['"', '\\']) {
                    return Err(GatewayError::Config(format!("path `{path}`")));
                }
            }
            Ok(vec![
                ("transport/link/tls/root_ca_certificate", quoted(ca)),
                ("transport/link/tls/listen_certificate", quoted(cert)),
                ("transport/link/tls/listen_private_key", quoted(key)),
                ("transport/link/tls/connect_certificate", quoted(cert)),
                ("transport/link/tls/connect_private_key", quoted(key)),
                ("transport/link/tls/enable_mtls", "true".into()),
                ("access_control", acl(instance, read_history, sources)?),
            ])
        }
    }
}

/// The transport ACL: deny by default; each `read_history` identity may subscribe to and
/// query the History keys; each identity the gateway knows may publish on its own Desired key
/// only, where the gateway's rule counts the ones that are not sources.
// DECISION: an identity that only reads History may still reach its own Desired key, so its
// Desired is dropped and counted by the gateway (SRS FR-13.2) instead of vanishing in the
// transport; an identity the gateway does not know is denied by the transport alone.
pub(crate) fn acl(
    instance: &str,
    read_history: &[&str],
    sources: &[&str],
) -> Result<String, GatewayError> {
    let mut known: Vec<&str> = read_history.to_vec();
    known.extend(sources.iter().filter(|s| !read_history.contains(s)));
    if let Some(bad) = known.iter().find(|name| !keys::is_chunk(name)) {
        return Err(GatewayError::Config(format!("identity `{bad}`")));
    }
    let history = format!(
        "[{}, {}]",
        quoted(&keys::history(instance)),
        quoted(&keys::ring(instance))
    );
    let mut rules = String::new();
    let mut subjects = String::new();
    let mut policies = String::new();
    // The two directions of a History read: their declarations in, our data out.
    let _ = write!(
        rules,
        r#"{{"id":"history-in","messages":["declare_subscriber","query"],"flows":["ingress"],"permission":"allow","key_exprs":{history}}},"#
    );
    let _ = write!(
        rules,
        r#"{{"id":"history-out","messages":["put","reply","declare_queryable"],"flows":["egress"],"permission":"allow","key_exprs":{history}}},"#
    );
    for (i, name) in known.iter().enumerate() {
        let desired = quoted(&keys::desired(instance, name));
        let _ = write!(
            rules,
            r#"{{"id":"desired-{i}","messages":["put"],"flows":["ingress"],"permission":"allow","key_exprs":[{desired}]}},{{"id":"desired-decl-{i}","messages":["declare_subscriber"],"flows":["egress"],"permission":"allow","key_exprs":[{desired},{any}]}},"#,
            any = quoted(&keys::desired_any(instance)),
        );
        let _ = write!(
            subjects,
            r#"{{"id":"s{i}","cert_common_names":[{}]}},"#,
            quoted(name)
        );
        let reads = if read_history.contains(name) {
            r#""history-in","history-out","#
        } else {
            ""
        };
        let _ = write!(
            policies,
            r#"{{"rules":[{reads}"desired-{i}","desired-decl-{i}"],"subjects":["s{i}"]}},"#
        );
    }
    Ok(format!(
        r#"{{"enabled":true,"default_permission":"deny","rules":[{rules}],"subjects":[{subjects}],"policies":[{policies}]}}"#
    ))
}

/// `text` as a JSON string; it holds no quote nor backslash.
fn quoted(text: &str) -> String {
    format!("\"{text}\"")
}

/// Whether `endpoint` is plain TCP on the loopback interface.
fn is_loopback(endpoint: &str) -> bool {
    ["tcp/127.0.0.1:", "tcp/[::1]:", "tcp/localhost:"]
        .iter()
        .any(|prefix| endpoint.starts_with(prefix))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::zenoh::{Gateway, GatewayConfig};

    /// A mutual-TLS gateway of `arm-1` read by `viz-1`, listening on TLS.
    fn tls_gateway() -> Gateway {
        Gateway::new(GatewayConfig {
            instance: "arm-1",
            identity: Identity::files(
                "/etc/kmr/arm-1.crt",
                "/etc/kmr/arm-1.key",
                "/etc/kmr/ca.crt",
            ),
            read_history: &["viz-1"],
        })
        .listen("tls/0.0.0.0:7447")
    }

    #[test]
    fn mutual_tls_with_a_deny_by_default_acl_is_a_valid_session_config() {
        let config = tls_gateway()
            .session_config(&["ctl-2"])
            .expect("a valid config");
        let get = |key: &str| config.get_json(key).expect(key);
        assert_eq!(get("access_control/enabled"), "true");
        assert_eq!(get("access_control/default_permission"), "\"deny\"");
        assert_eq!(get("transport/link/tls/enable_mtls"), "true");
        assert_eq!(
            get("transport/link/tls/listen_private_key"),
            "\"/etc/kmr/arm-1.key\"",
            "a path, never the key itself"
        );
        let acl = get("access_control");
        assert!(acl.contains("kmr/arm-1/desired/ctl-2") && acl.contains("kmr/arm-1/desired/viz-1"));
        assert!(acl.contains("\"viz-1\"") && acl.contains("\"ctl-2\""));
    }

    #[test]
    fn only_history_readers_get_the_history_rules() {
        let acl = acl("arm-1", &["viz-1"], &["ctl-2"]).expect("an acl");
        let policy = |subject: &str| {
            let end = acl
                .find(&format!("\"subjects\":[\"{subject}\"]"))
                .expect(subject);
            let start = acl[..end].rfind("{\"rules\"").expect("its policy");
            acl[start..end].to_owned()
        };
        assert!(policy("s0").contains("history-in"), "viz-1 reads History");
        assert!(!policy("s1").contains("history-in"), "ctl-2 does not");
    }

    #[test]
    fn a_certificate_identity_takes_tls_endpoints_only() {
        let gateway = tls_gateway().connect("tcp/10.0.0.2:7447");
        assert!(matches!(
            gateway.session_config(&[]),
            Err(GatewayError::NotTls { endpoint }) if endpoint == "tcp/10.0.0.2:7447"
        ));
    }

    #[test]
    fn a_loopback_identity_takes_loopback_endpoints_only() {
        let gateway = Gateway::new(GatewayConfig {
            instance: "arm-1",
            identity: Identity::loopback(),
            read_history: &[],
        });
        assert!(
            gateway
                .clone()
                .listen("tcp/127.0.0.1:7447")
                .session_config(&[])
                .is_ok()
        );
        assert!(matches!(
            gateway.listen("tcp/0.0.0.0:7447").session_config(&[]),
            Err(GatewayError::NotLoopback { .. })
        ));
    }

    #[test]
    fn an_identity_that_is_not_one_key_chunk_is_refused() {
        assert!(matches!(
            acl("arm-1", &["a/b"], &[]),
            Err(GatewayError::Config(_))
        ));
    }
}
