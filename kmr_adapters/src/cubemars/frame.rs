//! AK60-6 MIT mode frames, ported from cubemars_rs `src/main.rs`: 8 data
//! bytes, p/v/kp/kd/t packed 16/12/12/12/12 bits over the model's ranges.
//!
//! Values cross here in the engine's sign convention; the motor's is the
//! opposite for p, v and t, so both directions negate them. The zero frame
//! `FF×7 FE` rewrites the motor's stored zero and has no constructor here.

/// Position range of the model, rad.
pub(crate) const P_MAX: f32 = 12.5;
/// Velocity range of the model, rad/s.
pub(crate) const V_MAX: f32 = 45.0;
/// Torque range of the model, N·m.
pub(crate) const T_MAX: f32 = 15.0;
/// Highest position gain the frame carries.
pub(crate) const KP_MAX: f32 = 500.0;
/// Highest velocity gain the frame carries.
pub(crate) const KD_MAX: f32 = 5.0;

/// Enter MIT mode: torque on, answers with the state.
pub(crate) const ENTER: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC];
/// Exit MIT mode: limp, answers with the state.
pub(crate) const EXIT: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD];

/// One MIT command, engine sign convention.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Mit {
    /// Position, rad, on the motor's own scale.
    pub(crate) p: f32,
    /// Velocity, rad/s.
    pub(crate) v: f32,
    /// Position gain.
    pub(crate) kp: f32,
    /// Velocity gain.
    pub(crate) kd: f32,
    /// Feed-forward torque, N·m.
    pub(crate) t: f32,
}

/// The field of a [`Mit`] outside the model's range, or not finite.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum OutOfRange {
    /// Position.
    P,
    /// Velocity.
    V,
    /// Position gain.
    Kp,
    /// Velocity gain.
    Kd,
    /// Torque.
    T,
}

/// One reply, engine sign convention.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Feedback {
    /// Motor id, `data[0]`.
    pub(crate) id: u8,
    /// Position, rad, on the motor's own scale.
    pub(crate) p: f32,
    /// Velocity, rad/s.
    pub(crate) v: f32,
    /// Torque, N·m.
    pub(crate) t: f32,
    /// `data[6]`, read as °C as cubemars_rs does (verify on hardware).
    pub(crate) temperature: u8,
    /// `data[7]`, the error code; 0 is no fault (verify on hardware).
    pub(crate) fault: u8,
}

/// `value` as a `bits`-bit unsigned over `-max..=max`, truncated as
/// cubemars_rs does; `None` outside the range or not finite. Never clipped.
fn quantize(value: f32, max: f32, bits: u32) -> Option<u32> {
    from_range(value, -max, max, bits)
}

/// `value` as a `bits`-bit unsigned over `min..=max`.
fn from_range(value: f32, min: f32, max: f32, bits: u32) -> Option<u32> {
    if !(min..=max).contains(&value) {
        return None;
    }
    let top = ((1u32 << bits) - 1) as f32;
    Some(((value - min) * top / (max - min)) as u32)
}

/// The `bits`-bit unsigned `raw` back over `-max..=max`.
fn dequantize(raw: u32, max: f32, bits: u32) -> f32 {
    let top = ((1u32 << bits) - 1) as f32;
    raw as f32 * (2.0 * max) / top - max
}

/// The 8 data bytes of `command`, signs negated for the motor.
pub(crate) fn pack(command: Mit) -> Result<[u8; 8], OutOfRange> {
    let p = quantize(-command.p, P_MAX, 16).ok_or(OutOfRange::P)?;
    let v = quantize(-command.v, V_MAX, 12).ok_or(OutOfRange::V)?;
    let kp = from_range(command.kp, 0.0, KP_MAX, 12).ok_or(OutOfRange::Kp)?;
    let kd = from_range(command.kd, 0.0, KD_MAX, 12).ok_or(OutOfRange::Kd)?;
    let t = quantize(-command.t, T_MAX, 12).ok_or(OutOfRange::T)?;
    Ok([
        (p >> 8) as u8,
        (p & 0xFF) as u8,
        (v >> 4) as u8,
        (((v & 0xF) << 4) | (kp >> 8)) as u8,
        (kp & 0xFF) as u8,
        (kd >> 4) as u8,
        (((kd & 0xF) << 4) | (t >> 8)) as u8,
        (t & 0xFF) as u8,
    ])
}

/// The reply in `data`, signs negated back.
pub(crate) fn unpack(data: [u8; 8]) -> Feedback {
    let [id, p_high, p_low, v_high, mixed, t_low, temperature, fault] = data;
    let p = (u32::from(p_high) << 8) | u32::from(p_low);
    let v = (u32::from(v_high) << 4) | u32::from(mixed >> 4);
    let t = (u32::from(mixed & 0xF) << 8) | u32::from(t_low);
    Feedback {
        id,
        p: -dequantize(p, P_MAX, 16),
        v: -dequantize(v, V_MAX, 12),
        t: -dequantize(t, T_MAX, 12),
        temperature,
        fault,
    }
}

/// The reply bytes of a motor in state `feedback`: the inverse of
/// [`unpack`], for the scripted transport.
#[cfg(test)]
pub(crate) fn reply(feedback: Feedback) -> [u8; 8] {
    let raw = |value: f32, max: f32, bits: u32| quantize(-value, max, bits).unwrap_or(0);
    let p = raw(feedback.p, P_MAX, 16);
    let v = raw(feedback.v, V_MAX, 12);
    let t = raw(feedback.t, T_MAX, 12);
    [
        feedback.id,
        (p >> 8) as u8,
        (p & 0xFF) as u8,
        (v >> 4) as u8,
        (((v & 0xF) << 4) | (t >> 8)) as u8,
        (t & 0xFF) as u8,
        feedback.temperature,
        feedback.fault,
    ]
}

/// The command in `data`, signs negated back: the inverse of [`pack`], for
/// the tests.
#[cfg(test)]
pub(crate) fn command(data: [u8; 8]) -> Mit {
    let [p_high, p_low, v_high, v_kp, kp_low, kd_high, kd_t, t_low] = data;
    let p = (u32::from(p_high) << 8) | u32::from(p_low);
    let v = (u32::from(v_high) << 4) | u32::from(v_kp >> 4);
    let kp = (u32::from(v_kp & 0xF) << 8) | u32::from(kp_low);
    let kd = (u32::from(kd_high) << 4) | u32::from(kd_t >> 4);
    let t = (u32::from(kd_t & 0xF) << 8) | u32::from(t_low);
    let gain = |raw: u32, max: f32| raw as f32 * max / 4095.0;
    Mit {
        p: -dequantize(p, P_MAX, 16),
        v: -dequantize(v, V_MAX, 12),
        kp: gain(kp, KP_MAX),
        kd: gain(kd, KD_MAX),
        t: -dequantize(t, T_MAX, 12),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// cubemars_rs `si_to_param`, verbatim but for the clamp: the oracle the
    /// packing is checked against.
    fn si_to_param(value: f32, min: f32, max: f32, bits: u32) -> u32 {
        let value = value.clamp(min, max);
        ((value - min) * ((1u32 << bits) - 1) as f32 / (max - min)) as u32
    }

    /// cubemars_rs `send_mit_command`'s bytes for the same command.
    fn reference(p: f32, v: f32, kp: f32, kd: f32, t: f32) -> [u8; 8] {
        let p = si_to_param(-p, -12.5, 12.5, 16);
        let v = si_to_param(-v, -45.0, 45.0, 12);
        let kp = si_to_param(kp, 0.0, 500.0, 12);
        let kd = si_to_param(kd, 0.0, 5.0, 12);
        let t = si_to_param(-t, -15.0, 15.0, 12);
        [
            (p >> 8) as u8,
            (p & 0xFF) as u8,
            (v >> 4) as u8,
            (((v & 0xF) << 4) | (kp >> 8)) as u8,
            (kp & 0xFF) as u8,
            (kd >> 4) as u8,
            (((kd & 0xF) << 4) | (t >> 8)) as u8,
            (t & 0xFF) as u8,
        ]
    }

    #[test]
    fn packing_matches_cubemars_rs() {
        let cases = [
            (0.0, 0.0, 150.0, 1.0, 0.0),
            (1.25, -3.0, 500.0, 5.0, 2.5),
            (-12.5, 45.0, 0.0, 0.0, -15.0),
            (12.5, -45.0, 37.0, 0.3, 15.0),
        ];
        for (p, v, kp, kd, t) in cases {
            let packed = pack(Mit { p, v, kp, kd, t }).unwrap();
            assert_eq!(packed, reference(p, v, kp, kd, t), "{p} {v} {kp} {kd} {t}");
        }
    }

    #[test]
    fn signs_are_negated_on_the_wire() {
        let up = pack(Mit {
            p: 1.0,
            v: 2.0,
            kp: 0.0,
            kd: 0.0,
            t: 3.0,
        })
        .unwrap();
        // The motor's +1 rad is above the middle code; the engine's is below.
        assert!(u16::from_be_bytes([up[0], up[1]]) < 0x7FFF);
        let back = command(up);
        assert!((back.p - 1.0).abs() < 1e-3);
        assert!((back.v - 2.0).abs() < 0.03);
        assert!((back.t - 3.0).abs() < 0.01);
    }

    #[test]
    fn out_of_range_fields_are_refused_never_clipped() {
        let base = Mit {
            p: 0.0,
            v: 0.0,
            kp: 150.0,
            kd: 1.0,
            t: 0.0,
        };
        let cases = [
            (Mit { p: 12.51, ..base }, OutOfRange::P),
            (Mit { p: -12.51, ..base }, OutOfRange::P),
            (
                Mit {
                    p: f32::NAN,
                    ..base
                },
                OutOfRange::P,
            ),
            (Mit { v: 45.1, ..base }, OutOfRange::V),
            (Mit { kp: -0.1, ..base }, OutOfRange::Kp),
            (Mit { kp: 500.1, ..base }, OutOfRange::Kp),
            (Mit { kd: 5.1, ..base }, OutOfRange::Kd),
            (
                Mit {
                    t: f32::INFINITY,
                    ..base
                },
                OutOfRange::T,
            ),
            (Mit { t: -15.1, ..base }, OutOfRange::T),
        ];
        for (command, field) in cases {
            assert_eq!(pack(command), Err(field), "{command:?}");
        }
    }

    #[test]
    fn feedback_round_trips_within_one_step() {
        let state = Feedback {
            id: 3,
            p: -2.0,
            v: 7.5,
            t: -1.25,
            temperature: 31,
            fault: 0,
        };
        let back = unpack(reply(state));
        assert_eq!((back.id, back.temperature, back.fault), (3, 31, 0));
        assert!((back.p - state.p).abs() <= 25.0 / 65535.0);
        assert!((back.v - state.v).abs() <= 90.0 / 4095.0);
        assert!((back.t - state.t).abs() <= 30.0 / 4095.0);
    }

    #[test]
    fn mode_frames_are_ff_then_the_code() {
        assert_eq!(&ENTER[..7], &[0xFF; 7]);
        assert_eq!(&EXIT[..7], &[0xFF; 7]);
        assert_eq!((ENTER[7], EXIT[7]), (0xFC, 0xFD));
    }
}
