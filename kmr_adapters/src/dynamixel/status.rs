//! What an actuator's status packet reports, decoded per protocol, and the
//! request it answered (ROBOTIS e-Manual, Protocol 1 and 2, "Error").

#![cfg_attr(
    not(feature = "dynamixel"),
    expect(
        dead_code,
        reason = "only the SDK backend decodes every status packet, the scripted one builds some"
    )
)]

use core::fmt;

/// An error a status packet reported, and the request it answered.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct StatusError {
    /// What the error byte says.
    pub(crate) report: StatusReport,
    /// The request the reply answered.
    pub(crate) request: Request,
}

impl fmt::Display for StatusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.report)?;
        if self.request.is_empty() {
            Ok(())
        } else {
            write!(f, ", in reply to {}", self.request)
        }
    }
}

/// The error byte of a status packet, decoded per protocol.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum StatusReport {
    /// Protocol 1: one error per bit, several at once.
    P1(P1Errors),
    /// Protocol 2: the alert bit and an error code.
    P2 {
        /// Bit 7: a hardware fault, which Hardware Error Status details.
        alert: bool,
        /// Bits 0 to 6; `None` when they are all zero.
        code: Option<P2Error>,
    },
    /// A report the SDK decodes and this build of kmr does not name.
    Unnamed,
}

impl fmt::Display for StatusReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::P1(errors) => write!(f, "the actuator reports {errors}"),
            Self::P2 {
                alert: true,
                code: Some(code),
            } => write!(
                f,
                "the actuator reports a hardware alert and refused the instruction: {code}"
            ),
            Self::P2 {
                alert: true,
                code: None,
            } => f.write_str("the actuator reports a hardware alert"),
            Self::P2 {
                alert: false,
                code: Some(code),
            } => write!(f, "the actuator refused the instruction: {code}"),
            Self::P2 {
                alert: false,
                code: None,
            } => f.write_str("the actuator reports an error with no code"),
            Self::Unnamed => {
                f.write_str("the actuator reports an error this build of kmr does not name")
            }
        }
    }
}

/// One Protocol 1 error bit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum P1Error {
    /// Bit 0: input voltage outside its limits.
    InputVoltage,
    /// Bit 1: goal position outside the angle limits.
    AngleLimit,
    /// Bit 2: temperature above its limit.
    Overheating,
    /// Bit 3: an instruction value outside its range.
    Range,
    /// Bit 4: the actuator saw a checksum mismatch in the packet sent.
    Checksum,
    /// Bit 5: the load is above the torque set.
    Overload,
    /// Bit 6: an undefined instruction, or Action without Reg Write.
    Instruction,
}

impl P1Error {
    /// Every error, lowest bit first.
    pub(crate) const ALL: [Self; 7] = [
        Self::InputVoltage,
        Self::AngleLimit,
        Self::Overheating,
        Self::Range,
        Self::Checksum,
        Self::Overload,
        Self::Instruction,
    ];

    /// The error, named with its article.
    const fn phrase(self) -> &'static str {
        match self {
            Self::InputVoltage => "an input voltage error",
            Self::AngleLimit => "an angle limit error",
            Self::Overheating => "an overheating error",
            Self::Range => "a range error",
            Self::Checksum => "a checksum error",
            Self::Overload => "an overload error",
            Self::Instruction => "an instruction error",
        }
    }
}

/// The errors one Protocol 1 status packet reports.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct P1Errors {
    /// Each error at its e-Manual bit.
    // DECISION: the error byte as sent, read only through `contains` and
    // `undefined`: seven bools would not keep bit 7, which the e-Manual
    // leaves undefined, and `P1Error` names each defined one.
    bits: u8,
}

impl P1Errors {
    /// The errors of Protocol 1 error byte `bits`.
    pub(crate) const fn from_bits(bits: u8) -> Self {
        Self { bits }
    }

    /// With `error` set.
    #[cfg(test)]
    pub(crate) const fn with(self, error: P1Error) -> Self {
        Self {
            bits: self.bits | 1 << error as u8,
        }
    }

    /// Whether `error` is set.
    pub(crate) const fn contains(self, error: P1Error) -> bool {
        self.bits & 1 << error as u8 != 0
    }

    /// Whether bit 7, which the e-Manual leaves undefined, is set.
    pub(crate) const fn undefined(self) -> bool {
        self.bits & 0x80 != 0
    }
}

impl fmt::Display for P1Errors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let undefined = self.undefined().then_some("an undefined error bit");
        let mut set = P1Error::ALL
            .into_iter()
            .filter(|&error| self.contains(error))
            .map(P1Error::phrase)
            .chain(undefined)
            .peekable();
        let Some(first) = set.next() else {
            return f.write_str("an error with no bit set");
        };
        f.write_str(first)?;
        let mut written = 1;
        while let Some(phrase) = set.next() {
            let separator = match (set.peek(), written) {
                (None, 1) => " and ",
                (None, _) => ", and ",
                (Some(_), _) => ", ",
            };
            write!(f, "{separator}{phrase}")?;
            written += 1;
        }
        Ok(())
    }
}

/// A Protocol 2 error code, bits 0 to 6 of the error byte.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum P2Error {
    /// 1: the instruction could not be processed.
    ResultFail,
    /// 2: an undefined instruction, or Action without Reg Write.
    Instruction,
    /// 3: the actuator saw a CRC mismatch in the packet sent.
    Crc,
    /// 4: the value is outside the register's range.
    DataRange,
    /// 5: the data is shorter than the register.
    DataLength,
    /// 6: the value is outside the limit set.
    DataLimit,
    /// 7: a read-only, write-only or undefined address, or an EEPROM write
    /// with the torque on.
    Access,
    /// A code the e-Manual does not define.
    Unknown(u8),
    /// A code the SDK decodes and this build of kmr does not name.
    Unnamed,
}

impl fmt::Display for P2Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ResultFail => f.write_str("result fail (it could not process it)"),
            Self::Instruction => f.write_str("undefined instruction"),
            Self::Crc => f.write_str("CRC error (the packet it received was corrupted)"),
            Self::DataRange => f.write_str("data range (the value is outside the register's range)"),
            Self::DataLength => f.write_str("data length (the data is shorter than the register)"),
            Self::DataLimit => f.write_str("data limit (the value is outside the limit set)"),
            Self::Access => f.write_str(
                "access (the register is read-only, write-only or undefined, or in EEPROM with the torque on)",
            ),
            Self::Unknown(code) => write!(f, "error code {code}, which the e-Manual does not define"),
            Self::Unnamed => f.write_str("an error code this build of kmr does not name"),
        }
    }
}

/// A Dynamixel instruction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Instruction {
    /// Ping.
    Ping,
    /// Read.
    Read,
    /// Write.
    Write,
    /// Reboot, Protocol 2.
    Reboot,
    /// Sync Read, Protocol 2.
    SyncRead,
    /// Sync Write.
    SyncWrite,
    /// Bulk Read.
    BulkRead,
    /// Bulk Write, Protocol 2.
    BulkWrite,
}

impl fmt::Display for Instruction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Ping => "Ping",
            Self::Read => "Read",
            Self::Write => "Write",
            Self::Reboot => "Reboot",
            Self::SyncRead => "Sync Read",
            Self::SyncWrite => "Sync Write",
            Self::BulkRead => "Bulk Read",
            Self::BulkWrite => "Bulk Write",
        })
    }
}

/// What the request a status packet answered did; `None` where unknown.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Request {
    /// The instruction.
    pub(crate) instruction: Option<Instruction>,
    /// The register's ROBOTIS name.
    pub(crate) register: Option<&'static str>,
    /// The control-table address, for a register with no name.
    pub(crate) addr: Option<u16>,
}

impl Request {
    /// Nothing known.
    #[cfg(test)]
    pub(crate) const NONE: Self = Self {
        instruction: None,
        register: None,
        addr: None,
    };

    /// Whether nothing is known.
    const fn is_empty(&self) -> bool {
        self.instruction.is_none() && self.register.is_none() && self.addr.is_none()
    }
}

/// `Write Torque Enable`, `Read at address 64`, `Ping`.
impl fmt::Display for Request {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match (self.instruction, self.register, self.addr) {
            (Some(instruction), Some(register), _) => write!(f, "{instruction} {register}"),
            (Some(instruction), None, Some(addr)) => write!(f, "{instruction} at address {addr}"),
            (Some(instruction), None, None) => write!(f, "{instruction}"),
            (None, Some(register), _) => write!(f, "a request on {register}"),
            (None, None, Some(addr)) => write!(f, "a request at address {addr}"),
            (None, None, None) => f.write_str("an unknown request"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p1(errors: &[P1Error]) -> StatusReport {
        StatusReport::P1(
            errors
                .iter()
                .fold(P1Errors::default(), |set, &error| set.with(error)),
        )
    }

    fn p2(alert: bool, code: Option<P2Error>) -> StatusReport {
        StatusReport::P2 { alert, code }
    }

    #[test]
    fn p1_errors_sit_at_their_e_manual_bits() {
        for (bit, error) in P1Error::ALL.into_iter().enumerate() {
            let errors = P1Errors::from_bits(1 << bit);
            assert!(errors.contains(error), "{error:?}");
            assert_eq!(errors, P1Errors::default().with(error));
            assert!(!errors.undefined());
        }
        assert!(P1Errors::from_bits(0x80).undefined());
    }

    #[test]
    fn each_p1_error_is_named_in_words() {
        let names = [
            "the actuator reports an input voltage error",
            "the actuator reports an angle limit error",
            "the actuator reports an overheating error",
            "the actuator reports a range error",
            "the actuator reports a checksum error",
            "the actuator reports an overload error",
            "the actuator reports an instruction error",
        ];
        for (error, name) in P1Error::ALL.into_iter().zip(names) {
            assert_eq!(p1(&[error]).to_string(), name);
        }
    }

    #[test]
    fn several_p1_errors_are_all_named() {
        assert_eq!(
            p1(&[P1Error::InputVoltage, P1Error::Overload]).to_string(),
            "the actuator reports an input voltage error and an overload error"
        );
        assert_eq!(
            StatusReport::P1(P1Errors::from_bits(0x85)).to_string(),
            "the actuator reports an input voltage error, an overheating error, and an undefined error bit"
        );
        assert_eq!(
            StatusReport::P1(P1Errors::default()).to_string(),
            "the actuator reports an error with no bit set"
        );
    }

    #[test]
    fn each_p2_code_is_named_in_words() {
        let cases = [
            (P2Error::ResultFail, "result fail (it could not process it)"),
            (P2Error::Instruction, "undefined instruction"),
            (
                P2Error::Crc,
                "CRC error (the packet it received was corrupted)",
            ),
            (
                P2Error::DataRange,
                "data range (the value is outside the register's range)",
            ),
            (
                P2Error::DataLength,
                "data length (the data is shorter than the register)",
            ),
            (
                P2Error::DataLimit,
                "data limit (the value is outside the limit set)",
            ),
            (
                P2Error::Access,
                "access (the register is read-only, write-only or undefined, or in EEPROM with the torque on)",
            ),
            (
                P2Error::Unknown(9),
                "error code 9, which the e-Manual does not define",
            ),
            (
                P2Error::Unnamed,
                "an error code this build of kmr does not name",
            ),
        ];
        for (code, name) in cases {
            assert_eq!(
                p2(false, Some(code)).to_string(),
                format!("the actuator refused the instruction: {name}")
            );
        }
    }

    #[test]
    fn the_p2_alert_is_named_with_or_without_a_code() {
        assert_eq!(
            p2(true, None).to_string(),
            "the actuator reports a hardware alert"
        );
        assert_eq!(
            p2(true, Some(P2Error::DataLimit)).to_string(),
            "the actuator reports a hardware alert and refused the instruction: data limit (the value is outside the limit set)"
        );
        assert_eq!(
            p2(false, None).to_string(),
            "the actuator reports an error with no code"
        );
    }

    #[test]
    fn the_request_answered_follows_the_report() {
        let write = Request {
            instruction: Some(Instruction::Write),
            register: Some("Torque Enable"),
            addr: Some(24),
        };
        let error = StatusError {
            report: p1(&[P1Error::InputVoltage]),
            request: write,
        };
        assert_eq!(
            error.to_string(),
            "the actuator reports an input voltage error, in reply to Write Torque Enable"
        );
        let raw = Request {
            register: None,
            ..write
        };
        assert_eq!(raw.to_string(), "Write at address 24");
        let unnamed = Request {
            instruction: None,
            ..write
        };
        assert_eq!(unnamed.to_string(), "a request on Torque Enable");
        let bare = StatusError {
            request: Request::NONE,
            ..error
        };
        assert_eq!(
            bare.to_string(),
            "the actuator reports an input voltage error"
        );
    }

    #[test]
    fn no_status_message_prints_hex() {
        let reports = (0..=u8::MAX)
            .map(|bits| StatusReport::P1(P1Errors::from_bits(bits)))
            .chain([P2Error::Unknown(0x7F), P2Error::Access].map(|code| p2(true, Some(code))));
        for report in reports {
            assert!(!report.to_string().contains("0x"), "{report}");
        }
    }
}
