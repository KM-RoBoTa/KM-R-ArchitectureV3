//! Buses: where this machine's actuators are plugged in (SRS FR-6.3).

use kmr_adapters::binding::{Binding, Port};

/// One serial bus, bound to this machine's port.
///
/// On a preset robot, the hardware table gives brand, protocol and baud, a call here overrides
/// the baud, and [`Robot::add_plugins`](crate::Robot::add_plugins) registers it. On a
/// `robot!(model = ..)` robot, [`build`](Self::build) it and hand the handle to the actuators on
/// it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bus {
    pub(crate) binding: Binding,
}

impl Bus {
    /// The USB adapter with this serial id, found under `/dev/serial/by-id`.
    /// Preferred: it survives reboots and replugging.
    pub fn by_id(serial: &'static str) -> Self {
        Self::at(Port::ById(serial))
    }

    /// The device at this path, e.g. `/dev/ttyACM0`. Less reliable than
    /// [`by_id`](Self::by_id): `/dev/tty*` numbers change across reboots.
    pub fn by_dev(path: &'static str) -> Self {
        Self::at(Port::ByDev(path))
    }

    /// Overrides the hardware table's baud rate for this bus.
    pub fn baud(mut self, baud: u32) -> Self {
        self.binding.baud = Some(baud);
        self
    }

    /// Names the bus of the hardware table this plugin binds. Optional on a one-bus robot.
    pub fn bus(mut self, label: &'static str) -> Self {
        self.binding.bus = Some(label);
        self
    }

    /// The handle of this bus, for the actuators on it: `Rx28::ids(1..=10).on(bus)`.
    pub fn build(self) -> BusHandle {
        BusHandle {
            binding: self.binding,
        }
    }

    /// A plugin on `port`, with the table's baud and no label.
    fn at(port: Port) -> Self {
        Self {
            binding: Binding {
                port,
                baud: None,
                bus: None,
            },
        }
    }

    /// The serial id or the path, as written.
    pub(crate) fn port(&self) -> &'static str {
        match self.binding.port {
            Port::ById(s) | Port::ByDev(s) => s,
        }
    }
}

/// A built [`Bus`], handed to each actuator on it. Copy it freely: the handles of one bus are
/// one bus.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BusHandle {
    pub(crate) binding: Binding,
}

impl BusHandle {
    /// The serial id or the path, as written.
    pub(crate) fn port(&self) -> &'static str {
        Bus {
            binding: self.binding,
        }
        .port()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn baud_changes_the_baud_only() {
        let plain = Bus::by_id("FT8ISZ6H");
        let fast = Bus::by_id("FT8ISZ6H").baud(3_000_000);
        assert_eq!(fast.binding.baud, Some(3_000_000));
        assert_eq!(
            Bus {
                binding: Binding {
                    baud: None,
                    ..fast.binding
                }
            },
            plain
        );
    }

    #[test]
    fn bus_sets_the_label_only() {
        let labelled = Bus::by_dev("/dev/ttyACM0").bus("arm");
        assert_eq!(labelled.binding.bus, Some("arm"));
        assert_eq!(labelled.binding.port, Port::ByDev("/dev/ttyACM0"));
        assert_eq!(labelled.binding.baud, None);
    }
}
