use std::error::Error;
use std::time::Duration;

use kmr_adapters::dynamixel::TableError;
use kmr_core::error::{ConfigError, RunError, StateError as CoreStateError};

/// Public error returned by [`State`](crate::State) accessors.
///
/// This is kmr_api's OWN error type, not a re-export of `kmr_core`'s
/// `StateError`: the value types (`Q`, `Qd`, `Tau`) are shared with the core,
/// but the error vocabulary users match on stays a deliberate api choice that
/// the engine can refine without breaking them. `From<CoreStateError>` maps
/// the core cause across the boundary.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum StateError {
    /// An actuator index exceeded the number of recorded entries or actuators.
    #[error("index {index} is out of range: only {len} entries are available")]
    OutOfRange { index: usize, len: usize },

    /// A requested history depth exceeded the retained history.
    #[error("history depth {provided_depth} is out of range: maximum depth is {max_depth}")]
    HistoryValueOutOfRange {
        provided_depth: usize,
        max_depth: usize,
    },

    /// The requested value has not been recorded yet.
    #[error("value requested is unavailable")]
    CriticalValueMissing,

    /// The actuator had not been read yet `depth` ticks back, or never was (a run without
    /// hardware): there is no value to give, not even a zero.
    #[error("no sample {depth} ticks back: the actuator had not been read yet")]
    NoSample { depth: usize },
}

/// Payload-free cause of a [`PrivateStateError`].
///
/// The rich error carries the offending numbers (index, len, depth…) and
/// renders them through `Display`. When you only need to branch on *what*
/// went wrong — not the specifics — match on this instead: unit variants, no
/// `{ .. }` binding. Get it via [`PrivateStateError::kind`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StateErrorKind {
    /// An actuator index exceeded the number of recorded entries or actuators.
    OutOfRange,
    /// A requested history depth exceeded the retained history.
    HistoryValueOutOfRange,
    /// The requested value has not been recorded yet.
    CriticalValueMissing,
    /// The actuator had not been read yet at that depth.
    NoSample,
}

impl StateError {
    /// The payload-free cause — match this without `{ .. }`. The specific
    /// numbers stay in `self` (its `Display`).
    pub fn kind(&self) -> StateErrorKind {
        match self {
            Self::OutOfRange { .. } => StateErrorKind::OutOfRange,
            Self::HistoryValueOutOfRange { .. } => StateErrorKind::HistoryValueOutOfRange,
            Self::CriticalValueMissing => StateErrorKind::CriticalValueMissing,
            Self::NoSample { .. } => StateErrorKind::NoSample,
        }
    }
}

impl From<CoreStateError> for StateError {
    fn from(e: CoreStateError) -> Self {
        match e {
            CoreStateError::OutOfRange { index, len } => Self::OutOfRange { index, len },
            CoreStateError::HistoryValueOutOfRange {
                provided_depth,
                max_depth,
            } => Self::HistoryValueOutOfRange {
                provided_depth,
                max_depth,
            },
            CoreStateError::CriticalValueMissing => Self::CriticalValueMissing,
            CoreStateError::NoSample { depth } => Self::NoSample { depth },
        }
    }
}

/// Error returned by [`Robot::run`](crate::Robot::run): what went wrong and how to fix it.
///
/// `Debug` is derived in debug builds, so `{e:?}` shows the variant and its fields, the
/// adapter's typed error included; release builds print the `Display` text instead, so
/// `fn main() -> Result<(), UserError>` shows one readable line and no internal cause
/// (SRS NFR-2).
#[cfg_attr(debug_assertions, derive(Debug))]
#[derive(thiserror::Error)]
pub enum UserError {
    /// Two bus plugins, or two different bus handles, name the same port (SRS FR-6.4: a
    /// registration error).
    #[error(
        "port `{port}` is declared as two buses; declare each port once: one `.add_plugins(..)`, or one `Bus::..build()` handle shared by its actuators"
    )]
    DuplicateBus { port: &'static str },

    /// A bus plugin names a bus no member of the robot's hardware has (SRS FR-6.4).
    #[error(
        "the bus plugin of `{port}` names bus `{label}`, which no driver of this robot has; check `.bus(..)` against the hardware table"
    )]
    UnknownBus {
        label: &'static str,
        port: &'static str,
    },

    /// Two brands of the robot's hardware label a bus alike: a plugin could not tell them
    /// apart (SRS FR-6.4).
    #[error(
        "bus label `{label}` names a bus of two drivers; give each bus its own label in the hardware table"
    )]
    SharedBusLabel { label: &'static str },

    /// A bus plugin names no bus, and the robot's hardware has several (SRS FR-6.4).
    #[error(
        "the bus plugin of `{port}` names no bus, and this robot has several; name one with `.bus(\"label\")`"
    )]
    LabelRequired { port: &'static str },

    /// Hardware is declared, but the adapter it needs is not compiled in.
    #[error(
        "hardware is declared, but kmr_api was built without its `{feature}` feature; enable `kmr_api/{feature}`, or remove the bus plugins and actuators to simulate"
    )]
    FeatureOff { feature: &'static str },

    /// Bus plugins are declared, but `robot!` names no hardware table for them.
    #[error(
        "bus plugins are declared, but this robot has no hardware table; hand each actuator its bus with `.add_actuator(..on(bus))` instead of `.add_plugins(..)`, or add `hardware = ..` to `robot!`, or remove the plugins to simulate"
    )]
    NoHardwareDeclared,

    /// The actuators declared with `add_actuator` are not as many as the model's.
    #[error(
        "{declared} actuators are declared with `.add_actuator(..)`, but this robot's model has {expected}; declare one per non-fixed joint, in the model's joint order"
    )]
    ActuatorCount { declared: usize, expected: usize },

    /// Actuators are declared on a robot whose hardware `robot!` already names: a preset, or
    /// `hardware = ..`. `preset` is that `robot!` argument.
    #[error(
        "`robot!({preset})` brings its own hardware table, so `.add_actuator(..)` is refused; bind its buses with `.add_plugins(..)`, or name the robot by `robot!(model = ..)` alone to declare its actuators"
    )]
    PresetHardware { preset: &'static str },

    /// A bus handle carries no baud rate: without a hardware table, nothing else gives one.
    #[error("bus `{port}` has no baud rate; give it `.baud(..)` before `.build()`")]
    BusWithoutBaud { port: &'static str },

    /// The declared actuators break a rule of the hardware table; actuator numbers are engine
    /// indices, in `add_actuator` order.
    #[error("the declared actuators are refused: {source}")]
    HardwareRejected {
        #[source]
        source: Box<TableError>,
    },

    /// The hardware failed to bind or to come up. `source` is the hardware error: the claims
    /// that do not cover the actuators, or the failed member, named by its position and type;
    /// that member's own error, typed, is the next [`Error::source`], for `downcast_ref`. The
    /// message names each member by its type's last path segment, `member 0 (Dynamixel)`;
    /// `source` keeps the full type name.
    #[error("hardware bring-up failed: {}", bring_up_message(&**.source))]
    BringUp {
        #[source]
        source: Box<dyn Error + Send + Sync>,
    },

    /// A payload is mounted on a link the model does not have (SRS FR-8.1, FR-6.4).
    #[error(
        "a payload is mounted on link `{link}`, which the model does not have; name a link of the robot's descriptor"
    )]
    UnknownLink { link: &'static str },

    /// The tick period is zero or outside [10 µs, 1 s] (SRS FR-3.1).
    #[error("dt of {dt:?} is out of range; give `.set_dt(..)` a value from 10 µs to 1 s")]
    DtOutOfRange { dt: Duration },

    /// An actuator has a limit from the robot's model and no policy for its breach (SRS
    /// FR-4.5, FR-6.4).
    #[error(
        "actuator {actuator} has a joint limit and no limit policy; set one with `.set_limit_policy_all(Policy::Clamp)` (or `set_limit_policy`, `set_limit_policy_group`)"
    )]
    NoPolicy { actuator: usize },

    /// A builder setting names an actuator the robot does not have (SRS FR-6.4).
    #[error("actuator {index} does not exist: this robot has {n} actuators, 0 to {n} - 1")]
    NoActuator { index: usize, n: usize },

    /// An envelope bound is not finite, or its lower bound is above its upper one (SRS
    /// FR-13.5).
    #[error(
        "the envelope of actuator {actuator} is invalid; give each bound finite values with lower <= upper"
    )]
    InvalidEnvelope { actuator: usize },

    /// The Zenoh gateway could not start: its settings, or its session (SRS FR-7.1).
    #[cfg(feature = "zenoh")]
    #[error("Zenoh gateway: {source}")]
    Gateway {
        #[source]
        source: crate::zenoh::GatewayError,
    },
}

#[cfg(feature = "zenoh")]
impl From<crate::zenoh::GatewayError> for UserError {
    fn from(source: crate::zenoh::GatewayError) -> Self {
        Self::Gateway { source }
    }
}

/// Prints the [`Display`](core::fmt::Display) text, so `fn main() -> Result<(), UserError>`
/// shows a readable line.
#[cfg(not(debug_assertions))]
impl core::fmt::Debug for UserError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Display::fmt(self, f)
    }
}

impl UserError {
    /// The bring-up error of an adapter failing with `source`, kept typed.
    pub(crate) fn bring_up(source: impl Error + Send + Sync + 'static) -> Self {
        Self::BringUp {
            source: Box::new(source),
        }
    }
}

/// Last path segment of a type name, without its generics or a reference:
/// `&mut kmr_adapters::dynamixel::Dynamixel<kmr_adapters::dynamixel::DxlRsV2, 1>` is
/// `Dynamixel`.
pub(crate) fn short_name(name: &str) -> &str {
    let path = name.split_once('<').map_or(name, |(path, _)| path);
    let last = path.rsplit_once("::").map_or(path, |(_, last)| last);
    let last = last.rsplit_once(' ').map_or(last, |(_, last)| last);
    last.trim_start_matches('&')
}

/// The hardware error's text, each `member N (type)` the engine wrote before the member's own
/// error shortened by [`short_name`]; the member's own text is left as it is.
fn bring_up_message(source: &(dyn Error + 'static)) -> String {
    let text = source.to_string();
    let own = source.source().map(ToString::to_string);
    let engine = match &own {
        Some(own) => text.strip_suffix(own.as_str()).unwrap_or(&text),
        None => &text,
    };
    let mut message = shorten_members(engine);
    message.push_str(&text[engine.len()..]);
    message
}

/// `text` with the type of each `member N (type)` shortened by [`short_name`].
fn shorten_members(text: &str) -> String {
    const MEMBER: &str = "member ";
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    while let Some(at) = rest.find(MEMBER) {
        let (before, after) = rest.split_at(at + MEMBER.len());
        out.push_str(before);
        rest = after;
        let position = after.trim_start_matches(|c: char| c.is_ascii_digit());
        let Some(name) = position
            .strip_prefix(" (")
            .filter(|_| position.len() < after.len())
        else {
            continue;
        };
        let Some(end) = closing(name) else {
            continue;
        };
        out.push_str(&after[..after.len() - position.len()]);
        out.push_str(" (");
        out.push_str(short_name(&name[..end]));
        rest = &name[end..];
    }
    out.push_str(rest);
    out
}

/// Byte offset of the `)` that closes a `(` opened just before `text`.
fn closing(text: &str) -> Option<usize> {
    let mut depth = 0_usize;
    for (at, c) in text.char_indices() {
        match c {
            '(' => depth += 1,
            ')' if depth == 0 => return Some(at),
            ')' => depth -= 1,
            _ => {}
        }
    }
    None
}

impl From<ConfigError> for UserError {
    fn from(e: ConfigError) -> Self {
        match e {
            ConfigError::DtOutOfRange { dt } => Self::DtOutOfRange { dt },
            ConfigError::NoPolicy { actuator } => Self::NoPolicy { actuator },
        }
    }
}

impl<E: Error + Send + Sync + 'static> From<RunError<E>> for UserError {
    fn from(e: RunError<E>) -> Self {
        match e {
            RunError::Config(config) => config.into(),
            RunError::Hal(hardware) => Self::bring_up(hardware),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;
    use kmr_core::hal::{Coverage, HardwareError, Member, MemberError};

    /// Stands for an adapter error.
    #[derive(Debug, thiserror::Error)]
    #[error("port FT8ISZ6H not found under /dev/serial/by-id")]
    struct NotFound;

    fn every_variant() -> [UserError; 13] {
        [
            UserError::DuplicateBus { port: "FT8ISZ6H" },
            UserError::UnknownBus {
                label: "elbow",
                port: "FT8ISZ6H",
            },
            UserError::LabelRequired { port: "FT8ISZ6H" },
            UserError::SharedBusLabel { label: "bus0" },
            UserError::FeatureOff {
                feature: "dynamixel",
            },
            UserError::NoHardwareDeclared,
            UserError::bring_up(NotFound),
            UserError::DtOutOfRange {
                dt: Duration::from_micros(5),
            },
            UserError::UnknownLink { link: "nope" },
            UserError::NoPolicy { actuator: 2 },
            UserError::ActuatorCount {
                declared: 9,
                expected: 10,
            },
            UserError::PresetHardware { preset: "XmBench" },
            UserError::BusWithoutBaud { port: "AL02L1KL" },
        ]
    }

    #[cfg(not(debug_assertions))]
    #[test]
    fn release_debug_prints_the_display_text() {
        for e in every_variant() {
            assert_eq!(format!("{e:?}"), e.to_string());
        }
    }

    #[cfg(debug_assertions)]
    #[test]
    fn debug_shows_the_variant_and_its_fields() {
        let [bus, _, _, _, _, _, bring_up, ..] = every_variant();
        assert_eq!(format!("{bus:?}"), r#"DuplicateBus { port: "FT8ISZ6H" }"#);
        assert_eq!(format!("{bring_up:?}"), "BringUp { source: NotFound }");
    }

    #[test]
    fn a_bring_up_error_keeps_the_adapter_error() {
        let e = UserError::bring_up(NotFound);
        let source = e.source().and_then(|s| s.downcast_ref::<NotFound>());
        assert!(source.is_some());
        assert_eq!(
            e.to_string(),
            "hardware bring-up failed: port FT8ISZ6H not found under /dev/serial/by-id"
        );
    }

    /// An adapter error whose own text looks like the engine's.
    #[derive(Debug, PartialEq, thiserror::Error)]
    #[error("member 0 (bus::Echo<u8>) answered")]
    struct Echo;

    /// The hardware error of member 0, `name`, failing with `error`.
    fn member<E: Error + 'static>(
        name: &'static str,
        error: E,
    ) -> HardwareError<MemberError<E, Infallible>> {
        HardwareError::Member(MemberError::Head { name, error })
    }

    #[test]
    fn a_short_name_is_the_last_path_segment_without_generics() {
        let names = [
            (
                "kmr_adapters::dynamixel::Dynamixel<kmr_adapters::dynamixel::DxlRsV2, 1>",
                "Dynamixel",
            ),
            ("a::B<c::D<e::F<g::H, 2>>, [i::J; 3], (k::L, m::N)>", "B"),
            ("&mut drivers::Fake<drivers::Leg>", "Fake"),
            ("&mut Fake<Leg>", "Fake"),
            ("&kmr_api::Fake", "Fake"),
            ("run::recording::Recorder", "Recorder"),
            ("Recorder", "Recorder"),
        ];
        for (name, short) in names {
            assert_eq!(short_name(name), short, "{name}");
        }
    }

    #[test]
    fn a_member_error_names_the_member_short_and_keeps_the_chain() {
        let full = "&mut kmr_adapters::dynamixel::Dynamixel<kmr_adapters::dynamixel::DxlRsV2, 1>";
        let e = UserError::bring_up(member(full, NotFound));

        assert_eq!(
            e.to_string(),
            "hardware bring-up failed: member 0 (Dynamixel): port FT8ISZ6H not found under \
             /dev/serial/by-id"
        );
        let hardware = e
            .source()
            .and_then(|s| s.downcast_ref::<HardwareError<MemberError<NotFound, Infallible>>>());
        assert!(hardware.is_some_and(|h| h.to_string().contains(full)));
        let own = e.source().and_then(Error::source);
        assert!(own.and_then(|s| s.downcast_ref::<NotFound>()).is_some());
    }

    #[test]
    fn the_members_own_text_is_left_as_it_is() {
        let e = UserError::bring_up(member("a::b::Fake<a::Arm>", Echo));

        assert_eq!(
            e.to_string(),
            "hardware bring-up failed: member 0 (Fake): member 0 (bus::Echo<u8>) answered"
        );
    }

    #[test]
    fn a_coverage_error_names_both_members_short() {
        let overlap = Coverage::Overlap {
            actuator: 0,
            first: Member {
                position: 0,
                name: "drivers::Fake<drivers::Arm>",
            },
            second: Member {
                position: 1,
                name: "&mut drivers::Fake<(drivers::Leg, u8)>",
            },
        };
        let e = UserError::bring_up(HardwareError::<Infallible>::Coverage(overlap));

        assert_eq!(
            e.to_string(),
            "hardware bring-up failed: actuator 0 is owned by member 0 (Fake) and member 1 (Fake)"
        );
    }

    #[test]
    fn every_message_is_one_line() {
        for e in every_variant() {
            assert!(!e.to_string().contains('\n'), "{e}");
        }
    }

    #[test]
    fn messages_name_the_fix() {
        let [
            _,
            unknown,
            unlabelled,
            shared,
            feature,
            hardware,
            bring_up,
            dt,
            link,
            policy,
            count,
            preset,
            baud,
        ] = every_variant();
        assert!(policy.to_string().contains("actuator 2"));
        assert!(
            policy
                .to_string()
                .contains("`.set_limit_policy_all(Policy::Clamp)`")
        );
        assert!(link.to_string().contains("`nope`"));
        assert!(shared.to_string().contains("its own label"));
        assert!(unknown.to_string().contains("`.bus(..)`"));
        assert!(unlabelled.to_string().contains(r#"`.bus("label")`"#));
        assert!(feature.to_string().contains("`kmr_api/dynamixel`"));
        assert!(hardware.to_string().contains("`hardware = ..`"));
        assert!(hardware.to_string().contains("`.add_actuator(..on(bus))`"));
        assert!(count.to_string().starts_with("9 actuators"));
        assert!(count.to_string().contains("has 10"));
        assert!(preset.to_string().contains("`robot!(XmBench)`"));
        assert!(baud.to_string().contains("`.baud(..)`"));
        assert!(bring_up.to_string().contains("FT8ISZ6H"));
        assert!(dt.to_string().contains("5µs"));
        assert!(dt.to_string().contains("10 µs to 1 s"));
    }
}
