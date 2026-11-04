//! Bus plugins as the adapters take them: which port, at which baud, for
//! which bus of the hardware table (FR-029, FR-030 (SRS FR-6.3, FR-6.4)).
//!
//! `kmr_api::Bus` builds a [`Binding`]. An adapter matches it to a table bus
//! by label before any I/O, and resolves its port at Init.

use core::fmt;

/// Where a bus plugin finds its serial port.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Port {
    /// A `/dev/serial/by-id` entry holding this serial. Preferred.
    ById(&'static str),
    /// This device path, as given. Less reliable across reboots.
    ByDev(&'static str),
}

/// Names the port in errors.
impl fmt::Display for Port {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ById(serial) => write!(f, "serial {serial}"),
            Self::ByDev(path) => write!(f, "path {path}"),
        }
    }
}

/// One bus plugin.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Binding {
    /// Where the port is.
    pub port: Port,
    /// Line rate in bit/s replacing the table's for this bus.
    pub baud: Option<u32>,
    /// Label of the table bus it binds; optional on a one-bus table.
    pub bus: Option<&'static str>,
}

/// Why the bus plugins do not bind the table's buses one to one. Each
/// adapter turns it into its own error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Unmatched {
    /// A plugin names a label no table bus has.
    UnknownBus {
        /// Label the plugin names.
        label: &'static str,
        /// Port of the plugin.
        port: Port,
    },
    /// A table bus no plugin binds.
    UnboundBus {
        /// Bus label.
        bus: &'static str,
    },
    /// A plugin left over once every bus is bound.
    ExtraPlugin {
        /// Port of the plugin.
        port: Port,
    },
    /// A plugin without a label, on a table of several buses.
    LabelRequired {
        /// Port of the plugin.
        port: Port,
    },
    /// Two plugins bind one bus.
    BusBoundTwice {
        /// Bus label.
        bus: &'static str,
        /// Port of the second plugin.
        port: Port,
    },
}

/// The binding of each bus of `labels`, matched by label. A one-bus table
/// also takes a plugin without a label. No I/O.
///
/// The only code of D6 (assumed, pending human: option B); option A or C
/// replaces this function only.
pub(crate) fn match_plugins<const NB: usize>(
    labels: [&'static str; NB],
    plugins: &[Binding],
) -> Result<[Binding; NB], Unmatched> {
    let mut bound: [Option<Binding>; NB] = [None; NB];
    for plugin in plugins {
        let port = plugin.port;
        let bus = match plugin.bus {
            Some(label) => labels
                .iter()
                .position(|&known| known == label)
                .ok_or(Unmatched::UnknownBus { label, port })?,
            None if NB == 1 => 0,
            None if NB == 0 => return Err(Unmatched::ExtraPlugin { port }),
            None => return Err(Unmatched::LabelRequired { port }),
        };
        let (Some(slot), Some(&label)) = (bound.get_mut(bus), labels.get(bus)) else {
            continue;
        };
        match (slot.is_some(), plugin.bus) {
            (false, _) => *slot = Some(*plugin),
            (true, None) => return Err(Unmatched::ExtraPlugin { port }),
            (true, Some(_)) => return Err(Unmatched::BusBoundTwice { bus: label, port }),
        }
    }
    let unbound = bound
        .iter()
        .zip(labels)
        .find(|(binding, _)| binding.is_none());
    if let Some((_, bus)) = unbound {
        return Err(Unmatched::UnboundBus { bus });
    }
    // Never taken: every bus was found bound above.
    Ok(bound.map(|binding| {
        binding.unwrap_or(Binding {
            port: Port::ByDev(""),
            baud: None,
            bus: None,
        })
    }))
}
