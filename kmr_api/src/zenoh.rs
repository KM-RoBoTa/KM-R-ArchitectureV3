//! The Zenoh gateway plugin (feature `zenoh`; SRS FR-7.1, section 7 Zenoh): the robot's
//! History out and remote Desired in, over two [`Latest`](crate::Latest) slots, so the tick
//! never waits on the network.

mod gateway;
pub mod keys;
mod msg;

use std::sync::Arc;

use ::zenoh::Wait;
use kmr_core::hal::TickFlags;
use kmr_core::state::{Q, Qd, Tau};
use kmr_core::{RobotState, Stages};

use crate::plugin::{Plugin, Plugins, seal::Plug};
use crate::slot::{Latest, LatestReader, LatestWriter};
use crate::{Faults, Time};

pub use gateway::Counters;
pub use msg::{DesiredMsg, SampleMsg, decode_ring, desired_len, sample_len};

use gateway::{Entities, Inbox, Running, bump};

/// How the gateway reaches its peers.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Identity {
    /// Plain TCP on the loopback interface only: every endpoint must be `tcp/127.0.0.1:..`,
    /// `tcp/[::1]:..` or `tcp/localhost:..`. No certificate: any process of this machine is
    /// trusted to name itself in its Desired key.
    // DECISION: a loopback identity for one-machine setups and tests; it refuses any other
    // endpoint, so it never opens the robot to the network.
    Loopback,
}

impl Identity {
    /// The loopback identity: plain TCP, loopback endpoints only.
    pub const fn loopback() -> Self {
        Self::Loopback
    }
}

/// The gateway's settings (contracts/zenoh.md § Builder).
#[derive(Clone, Debug)]
pub struct GatewayConfig {
    /// The robot's name in every key: `kmr/<instance>/..`. One key chunk.
    pub instance: &'static str,
    /// How the gateway authenticates and is authenticated.
    pub identity: Identity,
    /// Identities that may read the History keys.
    pub read_history: &'static [&'static str],
}

/// The Zenoh gateway: plug it in with [`Robot::add_plugins`](crate::Robot::add_plugins). Its
/// session and thread start with `run()` and stop when the run ends. Without an endpoint it
/// listens on nothing and connects to nothing.
#[derive(Clone, Debug)]
pub struct Gateway {
    config: GatewayConfig,
    listen: Vec<String>,
    connect: Vec<String>,
    counters: Counters,
}

impl Gateway {
    /// A gateway with `config`, no endpoint yet.
    pub fn new(config: GatewayConfig) -> Self {
        Self {
            config,
            listen: Vec::new(),
            connect: Vec::new(),
            counters: Counters::default(),
        }
    }

    /// Listens on `endpoint`, e.g. `tcp/127.0.0.1:7447`.
    pub fn listen(mut self, endpoint: impl Into<String>) -> Self {
        self.listen.push(endpoint.into());
        self
    }

    /// Connects to the peer at `endpoint`.
    pub fn connect(mut self, endpoint: impl Into<String>) -> Self {
        self.connect.push(endpoint.into());
        self
    }

    /// The gateway's counters, readable from any thread while and after the robot runs.
    pub fn counters(&self) -> Counters {
        self.counters.clone()
    }

    /// The instance, checked to be one key chunk.
    fn instance(&self) -> Result<&'static str, GatewayError> {
        let instance = self.config.instance;
        if keys::is_chunk(instance) {
            Ok(instance)
        } else {
            Err(GatewayError::Name { name: instance })
        }
    }

    /// The session's configuration: peer mode, no multicast scouting, the endpoints.
    fn session_config(&self) -> Result<::zenoh::Config, GatewayError> {
        let mut config = ::zenoh::Config::default();
        let mut set = |key: &str, value: String| {
            config
                .insert_json5(key, &value)
                .map_err(|e| GatewayError::Config(format!("{key}: {e}")))
        };
        set("mode", "\"peer\"".into())?;
        set("scouting/multicast/enabled", "false".into())?;
        set("listen/endpoints", json_list(&self.listen)?)?;
        set("connect/endpoints", json_list(&self.connect)?)?;
        match self.config.identity {
            Identity::Loopback => {
                if let Some(endpoint) = self
                    .listen
                    .iter()
                    .chain(&self.connect)
                    .find(|e| !is_loopback(e))
                {
                    return Err(GatewayError::NotLoopback {
                        endpoint: endpoint.clone(),
                    });
                }
            }
        }
        Ok(config)
    }

    /// Opens the session, declares the keys, spawns the gateway thread; returns it with the
    /// tick's side.
    pub(crate) fn start<const N: usize>(&self) -> Result<(Running, TickSide<N>), GatewayError> {
        let instance = self.instance()?;
        let session = ::zenoh::open(self.session_config()?)
            .wait()
            .map_err(GatewayError::session)?;
        let entities = Entities::declare(session, instance)?;

        let shared = &self.counters.0;
        let (desired_in, desired_out) =
            Latest::counting(Arc::clone(&shared.superseded_desired)).split();
        let (history_in, history_out) =
            Latest::counting(Arc::clone(&shared.missed_samples)).split();
        let inbox = AnySender {
            instance,
            counters: self.counters.clone(),
        };
        let running = gateway::spawn(
            entities,
            inbox,
            desired_in,
            history_out,
            self.counters.clone(),
        )?;
        Ok((running, TickSide::new(desired_out, history_in)))
    }
}

impl Plug for Gateway {
    fn plug(self, plugins: &mut Plugins) {
        plugins.gateways.push(self);
    }
}

impl Plugin for Gateway {}

/// The one gateway of `gateways`, if any.
pub(crate) fn one(gateways: Vec<Gateway>) -> Result<Option<Gateway>, GatewayError> {
    let mut gateways = gateways.into_iter();
    let first = gateways.next();
    match gateways.next() {
        Some(_) => Err(GatewayError::TwoGateways),
        None => Ok(first),
    }
}

/// Why the gateway could not start.
#[derive(Debug, thiserror::Error)]
pub enum GatewayError {
    /// The instance or an identity is not one key chunk.
    #[error("`{name}` is not a valid name: use one key chunk, no `/`, `*`, `$`, `#`, `?` or space")]
    Name { name: &'static str },
    /// A loopback gateway was given an endpoint off the loopback interface.
    #[error(
        "endpoint `{endpoint}` is not on the loopback interface; a loopback gateway only takes `tcp/127.0.0.1:..`, `tcp/[::1]:..` or `tcp/localhost:..`"
    )]
    NotLoopback { endpoint: String },
    /// Two gateways were plugged in.
    #[error("two gateways are plugged in; plug in one")]
    TwoGateways,
    /// The session configuration was refused.
    #[error("gateway configuration: {0}")]
    Config(String),
    /// The session did not open, or a key could not be declared.
    #[error("gateway session: {0}")]
    Session(String),
}

impl GatewayError {
    /// A Zenoh error, as text: Zenoh's error type is not `Sync`.
    fn session(error: impl core::fmt::Display) -> Self {
        Self::Session(error.to_string())
    }
}

/// `endpoints` as a JSON list.
fn json_list(endpoints: &[String]) -> Result<String, GatewayError> {
    if let Some(bad) = endpoints.iter().find(|e| e.contains(['"', '\\'])) {
        return Err(GatewayError::Config(format!("endpoint `{bad}`")));
    }
    let quoted: Vec<String> = endpoints.iter().map(|e| format!("\"{e}\"")).collect();
    Ok(format!("[{}]", quoted.join(",")))
}

/// Whether `endpoint` is plain TCP on the loopback interface.
fn is_loopback(endpoint: &str) -> bool {
    ["tcp/127.0.0.1:", "tcp/[::1]:", "tcp/localhost:"]
        .iter()
        .any(|prefix| endpoint.starts_with(prefix))
}

/// Hands every Desired of the instance that decodes to the tick.
struct AnySender {
    instance: &'static str,
    counters: Counters,
}

impl<const N: usize> Inbox<N> for AnySender {
    fn receive(&mut self, key: &str, payload: &[u8], desired: &mut LatestWriter<DesiredMsg<N>>) {
        if keys::sender(key, self.instance).is_none() {
            return;
        }
        match DesiredMsg::decode(payload) {
            Some(msg) => {
                desired.put(msg);
                bump(&self.counters.0.accepted);
            }
            None => bump(&self.counters.0.malformed),
        }
    }
}

/// The tick's side of the gateway: the core's stage points (SRS FR-7.1). Fixed-size copies
/// only: no heap, lock, wait, I/O or crypto.
pub(crate) struct TickSide<const N: usize> {
    desired: LatestReader<DesiredMsg<N>>,
    history: LatestWriter<SampleMsg<N>>,
    /// The index of the running tick, taken at `inbound`.
    tick: u64,
}

impl<const N: usize> TickSide<N> {
    /// The tick's halves of the two slots.
    pub(crate) fn new(
        desired: LatestReader<DesiredMsg<N>>,
        history: LatestWriter<SampleMsg<N>>,
    ) -> Self {
        Self {
            desired,
            history,
            tick: 0,
        }
    }
}

impl<const N: usize> Stages<N> for TickSide<N> {
    fn inbound(&mut self, state: &mut RobotState<N>, time: &kmr_core::Time) {
        self.tick = Time::new(time).tick();
        if let Some(msg) = self.desired.take() {
            apply(&msg, state);
        }
    }

    fn outbound(&mut self, state: &RobotState<N>, _time: &kmr_core::Time) {
        self.history.put(sample(state, self.tick));
    }
}

/// Writes the fields of `msg` whose bit is set, through the state's setters.
// DECISION: a non-finite value is not written: the remote side cannot hand NaN to the limit
// stage.
fn apply<const N: usize>(msg: &DesiredMsg<N>, state: &mut RobotState<N>) {
    let [q, qd, tau] = msg.written;
    for i in 0..N.min(64) {
        let bit = 1_u64 << i;
        if q & bit != 0
            && let Some(&v) = msg.q.get(i).filter(|v| v.is_finite())
        {
            let _ = state.set_at(Q(v), i);
        }
        if qd & bit != 0
            && let Some(&v) = msg.qd.get(i).filter(|v| v.is_finite())
        {
            let _ = state.set_at(Qd(v), i);
        }
        if tau & bit != 0
            && let Some(&v) = msg.tau.get(i).filter(|v| v.is_finite())
        {
            let _ = state.set_at(Tau(v), i);
        }
    }
}

/// Depth 0 of `state` as the message of tick `tick`.
fn sample<const N: usize>(state: &RobotState<N>, tick: u64) -> SampleMsg<N> {
    let faults = state.faults(0).copied().unwrap_or_default();
    let flags = state.tick_flags(0).unwrap_or(TickFlags::NONE);
    SampleMsg {
        tick,
        age: state.sample_age(),
        q: values(state.current::<Q>(), |v| v.0),
        qd: values(state.current::<Qd>(), |v| v.0),
        tau: values(state.current::<Tau>(), |v| v.0),
        faults: Faults::new(faults, flags),
    }
}

/// The raw values of `field`, NaN without a sample.
fn values<T: Copy, const N: usize>(field: Option<&[T; N]>, raw: fn(T) -> f32) -> [f32; N] {
    field.map_or([f32::NAN; N], |values| values.map(raw))
}

#[cfg(test)]
mod tests;
