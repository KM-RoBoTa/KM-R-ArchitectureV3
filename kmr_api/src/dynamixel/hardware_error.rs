//! [`HardwareError`]: the raw byte of `hardware_error_at`, decoded for a Dynamixel.

use core::fmt;

use kmr_adapters::dynamixel::{Model, Protocol};

/// A Dynamixel's hardware error, decoded from the raw byte of
/// [`State::hardware_error_at`](crate::State::hardware_error_at).
///
/// The adapter stores the same layout on both protocols, the X series' Hardware Error Status:
/// bit 0 input voltage, bit 2 overheating, bit 3 motor encoder, bit 4 electrical shock, bit 5
/// overload. On Protocol 2 it is that register. On Protocol 1 it is the status packet's error
/// byte cut down to its hardware faults, which sit at the same positions: input voltage,
/// overheating and overload. Angle limit, range, checksum and instruction errors fail the read
/// instead and never reach the byte. Any other bit is unknown: kept in [`raw`](Self::raw),
/// shown by `Display`, named by no query.
///
/// ```
/// use kmr_api::dynamixel::Ax12a;
///
/// let error = Ax12a::hardware_error(0b0010_0001);
/// assert!(error.input_voltage() && error.overload());
/// assert!(!error.overheating());
/// assert_eq!(error.to_string(), "input voltage, overload");
/// assert!(Ax12a::hardware_error(0).is_empty());
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HardwareError {
    raw: u8,
    protocol: Protocol,
}

const INPUT_VOLTAGE: u8 = 1 << 0;
const OVERHEATING: u8 = 1 << 2;
const MOTOR_ENCODER: u8 = 1 << 3;
const ELECTRICAL_SHOCK: u8 = 1 << 4;
const OVERLOAD: u8 = 1 << 5;

/// Each named bit with its words, lowest bit first.
const NAMES: [(u8, &str); 5] = [
    (INPUT_VOLTAGE, "input voltage"),
    (OVERHEATING, "overheating"),
    (MOTOR_ENCODER, "motor encoder"),
    (ELECTRICAL_SHOCK, "electrical shock"),
    (OVERLOAD, "overload"),
];

// DECISION: the bit table is written here, not decoded through dxl_rs_v2's HardwareErrors:
// kmr_api does not depend on the SDK, and adding it, even as a re-export through kmr_adapters,
// would tie the public surface to the SDK's type and its "input voltage error" wording.
impl HardwareError {
    /// The byte `raw`, read from an actuator of `model`. Most code reaches it through the model
    /// type instead, e.g. `Ax12a::hardware_error(raw)`.
    pub fn new(model: Model, raw: u8) -> Self {
        Self {
            raw,
            protocol: model.protocol(),
        }
    }

    /// The bits this protocol's byte can name.
    fn named(self) -> u8 {
        match self.protocol {
            Protocol::One => INPUT_VOLTAGE | OVERHEATING | OVERLOAD,
            Protocol::Two => {
                INPUT_VOLTAGE | OVERHEATING | MOTOR_ENCODER | ELECTRICAL_SHOCK | OVERLOAD
            }
        }
    }

    /// Whether named bit `bit` is set.
    fn has(self, bit: u8) -> bool {
        self.raw & self.named() & bit != 0
    }

    /// The byte as stored, unknown bits included.
    pub fn raw(self) -> u8 {
        self.raw
    }

    /// Input voltage outside the actuator's voltage limits.
    pub fn input_voltage(self) -> bool {
        self.has(INPUT_VOLTAGE)
    }

    /// Internal temperature above the actuator's temperature limit.
    pub fn overheating(self) -> bool {
        self.has(OVERHEATING)
    }

    /// Load above the maximum torque.
    pub fn overload(self) -> bool {
        self.has(OVERLOAD)
    }

    /// Motor encoder malfunction. Protocol 2 only: always `false` on Protocol 1.
    pub fn motor_encoder(self) -> bool {
        self.has(MOTOR_ENCODER)
    }

    /// Electrical shock, or not enough power for the motor. Protocol 2 only: always `false` on
    /// Protocol 1.
    pub fn electrical_shock(self) -> bool {
        self.has(ELECTRICAL_SHOCK)
    }

    /// No bit set, unknown bits included.
    // DECISION: an unknown bit counts, so `is_empty` and `Display`'s "none" agree and a bit no
    // query names is never taken for a healthy actuator.
    pub fn is_empty(self) -> bool {
        self.raw == 0
    }

    /// Some bit set: `!is_empty()`.
    pub fn any(self) -> bool {
        !self.is_empty()
    }
}

/// The set conditions in words, lowest bit first, e.g. "input voltage, overload"; then any
/// unknown bits, e.g. "unknown bits 0x80"; "none" when no bit is set.
impl fmt::Display for HardwareError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_empty() {
            return f.write_str("none");
        }
        let mut sep = "";
        for (bit, name) in NAMES {
            if self.has(bit) {
                write!(f, "{sep}{name}")?;
                sep = ", ";
            }
        }
        let unknown = self.raw & !self.named();
        if unknown != 0 {
            write!(f, "{sep}unknown bits {unknown:#04x}")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamixel::{Ax12a, Mx64, Mx64V2, Rx28, Xm430W210, Xw430T200, Xw540T140, Xw540T260};

    /// Every query, in bit order: input voltage, overheating, motor encoder, electrical shock,
    /// overload.
    fn flags(error: HardwareError) -> [bool; 5] {
        [
            error.input_voltage(),
            error.overheating(),
            error.motor_encoder(),
            error.electrical_shock(),
            error.overload(),
        ]
    }

    const P1: Model = Model::Ax12a;
    const P2: Model = Model::Xm430W210;

    #[test]
    fn protocol_1_names_input_voltage_overheating_and_overload() {
        let at = |bit: u8| flags(HardwareError::new(P1, 1 << bit));
        assert_eq!(at(0), [true, false, false, false, false]);
        assert_eq!(at(2), [false, true, false, false, false]);
        assert_eq!(at(5), [false, false, false, false, true]);
    }

    #[test]
    fn protocol_1_names_no_other_bit() {
        for bit in [1, 3, 4, 6, 7] {
            let error = HardwareError::new(P1, 1 << bit);
            assert_eq!(flags(error), [false; 5], "bit {bit}");
            assert_eq!(error.raw(), 1 << bit);
            assert!(error.any());
        }
    }

    #[test]
    fn protocol_2_names_its_five_bits() {
        let at = |bit: u8| flags(HardwareError::new(P2, 1 << bit));
        assert_eq!(at(0), [true, false, false, false, false]);
        assert_eq!(at(2), [false, true, false, false, false]);
        assert_eq!(at(3), [false, false, true, false, false]);
        assert_eq!(at(4), [false, false, false, true, false]);
        assert_eq!(at(5), [false, false, false, false, true]);
    }

    #[test]
    fn protocol_2_names_no_unused_bit() {
        for bit in [1, 6, 7] {
            let error = HardwareError::new(P2, 1 << bit);
            assert_eq!(flags(error), [false; 5], "bit {bit}");
            assert!(error.any());
        }
    }

    #[test]
    fn zero_is_empty() {
        for model in [P1, P2] {
            let error = HardwareError::new(model, 0);
            assert!(error.is_empty());
            assert!(!error.any());
            assert_eq!(flags(error), [false; 5]);
            assert_eq!(error.raw(), 0);
        }
    }

    #[test]
    fn raw_keeps_every_bit() {
        for model in [P1, P2] {
            assert_eq!(HardwareError::new(model, 0xFF).raw(), 0xFF);
        }
    }

    #[test]
    fn each_model_decodes_with_its_protocol() {
        let raw = 0b0011_1101;
        let p1 = HardwareError::new(P1, raw);
        let p2 = HardwareError::new(P2, raw);
        assert_eq!(Ax12a::hardware_error(raw), p1);
        assert_eq!(Rx28::hardware_error(raw), p1);
        assert_eq!(Mx64::hardware_error(raw), p1);
        assert_eq!(Mx64V2::hardware_error(raw), p2);
        assert_eq!(Xm430W210::hardware_error(raw), p2);
        assert_eq!(Xw430T200::hardware_error(raw), p2);
        assert_eq!(Xw540T140::hardware_error(raw), p2);
        assert_eq!(Xw540T260::hardware_error(raw), p2);
        assert_ne!(p1, p2);
    }

    #[test]
    fn display_says_none_when_empty() {
        assert_eq!(HardwareError::new(P1, 0).to_string(), "none");
        assert_eq!(HardwareError::new(P2, 0).to_string(), "none");
    }

    #[test]
    fn display_lists_the_set_conditions_lowest_bit_first() {
        assert_eq!(
            HardwareError::new(P1, 0b0010_0001).to_string(),
            "input voltage, overload"
        );
        assert_eq!(
            HardwareError::new(P2, 0b0011_1101).to_string(),
            "input voltage, overheating, motor encoder, electrical shock, overload"
        );
    }

    #[test]
    fn display_shows_unknown_bits() {
        assert_eq!(
            HardwareError::new(P1, 0b0000_1000).to_string(),
            "unknown bits 0x08"
        );
        assert_eq!(
            HardwareError::new(P2, 0b1000_0100).to_string(),
            "overheating, unknown bits 0x80"
        );
    }
}
