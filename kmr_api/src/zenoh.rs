//! The Zenoh gateway plugin (feature `zenoh`; SRS FR-7.1, section 7 Zenoh): the robot's
//! History out and remote Desired in, over two [`Latest`](crate::Latest) slots, so the tick
//! never waits on the network.

mod gateway;
pub mod keys;
mod msg;
mod security;
mod watchdog;

use std::sync::Arc;

use ::zenoh::Wait;
use kmr_core::hal::{EXTERNAL_FLAGS, ExternalFlags, TickFlags};
use kmr_core::state::{Q, Qd, Tau};
use kmr_core::{RobotState, Stages};

use crate::plugin::{Plugin, Plugins, seal::Plug};
use crate::slot::{Latest, LatestReader, LatestWriter};
use crate::{Faults, Time};

pub use gateway::Counters;
pub use msg::{DesiredMsg, SampleMsg, decode_ring, desired_len, sample_len};
pub use security::Identity;
pub use watchdog::{Allowed, Remote};

use gateway::{Entities, Running};
use security::Rule;
use watchdog::Watchdog;

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
    /// Where the unauthorized and remote-timeout flags are raised: the loop's.
    flags: &'static ExternalFlags,
}

impl Gateway {
    /// A gateway with `config`, no endpoint yet.
    pub fn new(config: GatewayConfig) -> Self {
        Self {
            config,
            listen: Vec::new(),
            connect: Vec::new(),
            counters: Counters::default(),
            flags: &EXTERNAL_FLAGS,
        }
    }

    /// Raises the gateway's flags in `flags` instead of the loop's: for scripted runs.
    #[cfg(test)]
    pub(crate) fn flags(mut self, flags: &'static ExternalFlags) -> Self {
        self.flags = flags;
        self
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

    /// The session's configuration: peer mode, no multicast scouting, the endpoints, the
    /// identity's transport and ACL with `sources` as the remote sources.
    pub(crate) fn session_config(&self, sources: &[&str]) -> Result<::zenoh::Config, GatewayError> {
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
        let endpoints: Vec<&String> = self.listen.iter().chain(&self.connect).collect();
        let instance = self.instance()?;
        for (key, value) in security::transport(
            &self.config.identity,
            instance,
            self.config.read_history,
            sources,
            &endpoints,
        )? {
            set(key, value)?;
        }
        Ok(config)
    }

    /// Opens the session, declares the keys, spawns the gateway thread with `sources` as the
    /// remote sources; returns it with the tick's side.
    pub(crate) fn start<const N: usize>(
        &self,
        sources: &[Remote],
    ) -> Result<(Running, TickSide<N>), GatewayError> {
        let instance = self.instance()?;
        watchdog::checked(sources)?;
        let names: Vec<&str> = sources.iter().map(Remote::identity).collect();
        let session = ::zenoh::open(self.session_config(&names)?)
            .wait()
            .map_err(GatewayError::session)?;
        let entities = Entities::declare(session, instance)?;

        let shared = &self.counters.0;
        let (desired_in, desired_out) =
            Latest::counting(Arc::clone(&shared.superseded_desired)).split();
        let (history_in, history_out) =
            Latest::counting(Arc::clone(&shared.missed_samples)).split();
        let (watchdog, heard) = Watchdog::new(sources, self.flags);
        let rule = Rule::new(instance, sources, heard, self.counters.clone(), self.flags);
        let running = gateway::spawn(
            entities,
            rule,
            desired_in,
            history_out,
            self.counters.clone(),
        )?;
        Ok((running, TickSide::new(desired_out, history_in, watchdog)))
    }
}

/// The gateway a builder plugged in, with the remote sources it registered.
pub(crate) struct Plugged {
    gateway: Gateway,
    sources: Vec<Remote>,
}

impl Plugged {
    /// The one gateway of `gateways` with `sources`; `None` without gateway or source.
    pub(crate) fn new(
        gateways: Vec<Gateway>,
        sources: Vec<Remote>,
    ) -> Result<Option<Self>, GatewayError> {
        let mut gateways = gateways.into_iter();
        let first = gateways.next();
        if gateways.next().is_some() {
            return Err(GatewayError::TwoGateways);
        }
        match (first, sources.first()) {
            (Some(gateway), _) => Ok(Some(Self { gateway, sources })),
            (None, Some(source)) => Err(GatewayError::NoGateway {
                identity: source.identity,
            }),
            (None, None) => Ok(None),
        }
    }

    /// [`Gateway::start`] with the registered sources.
    pub(crate) fn start<const N: usize>(&self) -> Result<(Running, TickSide<N>), GatewayError> {
        self.gateway.start(&self.sources)
    }
}

impl Plug for Gateway {
    fn plug(self, plugins: &mut Plugins) {
        plugins.gateways.push(self);
    }
}

impl Plugin for Gateway {}

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
    /// A gateway with a certificate identity was given an endpoint other than `tls/..`.
    #[error(
        "endpoint `{endpoint}` is not TLS; a gateway with `Identity::files` only takes `tls/..` endpoints"
    )]
    NotTls { endpoint: String },
    /// Two gateways were plugged in.
    #[error("two gateways are plugged in; plug in one")]
    TwoGateways,
    /// A remote source was added without a gateway to receive it.
    #[error(
        "remote source `{identity}` is added, but no gateway is plugged in; add `.add_plugins(Gateway::new(..))`"
    )]
    NoGateway { identity: &'static str },
    /// One identity was added as a remote source twice.
    #[error("remote source `{identity}` is added twice; add each source once")]
    TwoSources { identity: &'static str },
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

/// The tick's side of the gateway: the core's stage points (SRS FR-7.1). Fixed-size copies
/// only: no heap, lock, wait, I/O or crypto.
pub(crate) struct TickSide<const N: usize> {
    desired: LatestReader<DesiredMsg<N>>,
    history: LatestWriter<SampleMsg<N>>,
    watchdog: Watchdog,
    /// The index of the running tick, taken at `inbound`.
    tick: u64,
}

impl<const N: usize> TickSide<N> {
    /// The tick's halves of the two slots.
    pub(crate) fn new(
        desired: LatestReader<DesiredMsg<N>>,
        history: LatestWriter<SampleMsg<N>>,
        watchdog: Watchdog,
    ) -> Self {
        Self {
            desired,
            history,
            watchdog,
            tick: 0,
        }
    }
}

impl<const N: usize> Stages<N> for TickSide<N> {
    fn inbound(&mut self, state: &mut RobotState<N>, time: &kmr_core::Time) {
        let time = Time::new(time);
        self.tick = time.tick();
        self.watchdog.tick(time.dt());
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
