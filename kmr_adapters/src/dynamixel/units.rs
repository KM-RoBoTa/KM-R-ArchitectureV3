//! Per-model conversions between raw register values and `f32` SI units
//! (FR-021).
//!
//! Constants are the ROBOTIS e-Manual control tables. Zero rad is the
//! model's center position (research R10). A value with no raw counterpart
//! is `None`, never clamped (research R11).

use core::f32::consts::TAU;

use kmr_core::state::{Q, Qd, Tau};

use super::config::Model;

/// Radians per second in one rpm.
const RAD_S_PER_RPM: f32 = TAU / 60.0;

/// Register encoding and scale of one model.
#[derive(Clone, Copy, Debug)]
struct Scale {
    /// Highest position register value; the lowest is 0.
    max_position: u32,
    /// Position register value at zero rad.
    center: u32,
    /// Radians per position unit.
    rad_per_position: f32,
    /// Radians per second per velocity unit.
    rad_s_per_velocity: f32,
    /// Velocity is a 10-bit magnitude with bit 10 set for clockwise
    /// (Protocol 1), not two's complement.
    clockwise_bit: bool,
    /// Newton metres per current unit; `None` without a current register or
    /// a published torque constant.
    nm_per_current: Option<f32>,
}

/// AX and RX control table: 1024 units over 300°, 0.111 rpm, no current
/// register.
const AX: Scale = Scale {
    max_position: 1023,
    center: 512,
    rad_per_position: 300.0_f32.to_radians() / 1024.0,
    rad_s_per_velocity: 0.111 * RAD_S_PER_RPM,
    clockwise_bit: true,
    nm_per_current: None,
};

/// X control table in position mode: 4096 units per turn, 0.229 rpm,
/// 2.69 mA, with the model's torque constant in N·m per A.
const fn x_series(nm_per_amp: f32) -> Scale {
    Scale {
        max_position: 4095,
        center: 2048,
        rad_per_position: TAU / 4096.0,
        rad_s_per_velocity: 0.229 * RAD_S_PER_RPM,
        clockwise_bit: false,
        nm_per_current: Some(2.69e-3 * nm_per_amp),
    }
}

/// The scale of `model`: the one place its constants are chosen.
///
/// Torque constants are the e-Manual stall torque over stall current at
/// 12 V, so approximate: the real torque-current curve is not linear and
/// shifts with voltage and temperature.
const fn scale(model: Model) -> Scale {
    match model {
        Model::Ax12a | Model::Rx28 => AX,
        Model::Xm430W210 => x_series(1.304),
        Model::Xw430T200 => x_series(1.769),
        Model::Xw540T140 => x_series(1.408),
        Model::Xw540T260 => x_series(1.939),
    }
}

/// Highest position register value of `model`; the lowest is 0.
pub(crate) const fn max_position(model: Model) -> u32 {
    scale(model).max_position
}

/// Position register value `raw` of `model`, in rad. `None` outside
/// `0..=max_position`: a homing offset or another operating mode moved it.
pub(crate) fn position_to_si(model: Model, raw: i32) -> Option<Q> {
    let scale = scale(model);
    let raw = u32::try_from(raw)
        .ok()
        .filter(|&raw| raw <= scale.max_position)?;
    Some(Q(
        (raw as f32 - scale.center as f32) * scale.rad_per_position
    ))
}

/// Goal position register value of `model` for `q`, rounded to the nearest
/// unit. `None` when `q` is not finite or lands outside the register range.
pub(crate) fn position_to_raw(model: Model, q: Q) -> Option<u32> {
    let scale = scale(model);
    let raw = (q.0 / scale.rad_per_position + scale.center as f32).round();
    (0.0..=scale.max_position as f32)
        .contains(&raw)
        .then_some(raw as u32)
}

/// Position register value of `model` nearest to `q`, unchecked, for an
/// error that names it: NaN is 0, an infinity saturates.
pub(crate) fn nearest_position(model: Model, q: Q) -> i64 {
    let scale = scale(model);
    (q.0 / scale.rad_per_position + scale.center as f32).round() as i64
}

/// Present velocity register value `raw` of `model`, in rad/s.
pub(crate) fn velocity_to_si(model: Model, raw: i32) -> Qd {
    let scale = scale(model);
    let units = if !scale.clockwise_bit {
        raw
    } else if raw & 0x400 == 0 {
        raw & 0x3FF
    } else {
        -(raw & 0x3FF)
    };
    Qd(units as f32 * scale.rad_s_per_velocity)
}

/// Present current register value `raw` of `model`, in N·m. `None` when the
/// model has no current register or no known torque constant.
pub(crate) fn current_to_si(model: Model, raw: i16) -> Option<Tau> {
    scale(model)
        .nm_per_current
        .map(|nm_per_unit| Tau(f32::from(raw) * nm_per_unit))
}

#[cfg(test)]
mod tests {
    use core::f32::consts::{FRAC_PI_2, PI};

    use super::*;

    const MODELS: [Model; 6] = [
        Model::Ax12a,
        Model::Rx28,
        Model::Xm430W210,
        Model::Xw430T200,
        Model::Xw540T140,
        Model::Xw540T260,
    ];

    const X_MODELS: [Model; 4] = [
        Model::Xm430W210,
        Model::Xw430T200,
        Model::Xw540T140,
        Model::Xw540T260,
    ];

    /// `position_to_si` of an in-range `raw`.
    fn si(model: Model, raw: u32) -> Q {
        position_to_si(model, i32::try_from(raw).unwrap()).unwrap()
    }

    fn assert_close(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() <= 1e-5 * expected.abs().max(1.0),
            "{actual} != {expected}"
        );
    }

    #[test]
    fn center_is_zero_rad() {
        for model in MODELS {
            let center = scale(model).center;
            assert_eq!(si(model, center), Q(0.0), "{model:?}");
            assert_eq!(position_to_raw(model, Q(0.0)), Some(center), "{model:?}");
        }
    }

    #[test]
    fn every_raw_position_round_trips() {
        for model in MODELS {
            for raw in 0..=scale(model).max_position {
                let q = si(model, raw);
                assert_eq!(position_to_raw(model, q), Some(raw), "{model:?}");
            }
        }
    }

    #[test]
    fn si_round_trip_is_within_half_a_unit() {
        for model in MODELS {
            let unit = scale(model).rad_per_position;
            for step in -250..=250_i16 {
                let q = Q(f32::from(step) * 0.01);
                let Some(raw) = position_to_raw(model, q) else {
                    continue;
                };
                let error = (si(model, raw).0 - q.0).abs();
                assert!(error <= unit / 2.0 + 1e-6, "{model:?} {q:?}");
            }
        }
    }

    #[test]
    fn x_range_is_minus_pi_up_to_one_unit_short_of_pi() {
        for model in X_MODELS {
            let unit = scale(model).rad_per_position;
            assert_eq!(si(model, 0), Q(-PI));
            assert_eq!(position_to_raw(model, Q(-PI)), Some(0));
            assert_eq!(position_to_raw(model, Q(PI - unit)), Some(4095));
            assert_eq!(position_to_raw(model, Q(PI)), None);
            assert_eq!(position_to_raw(model, Q(-PI - unit)), None);
            assert_eq!(position_to_raw(model, Q(-PI - unit * 0.4)), Some(0));
        }
    }

    #[test]
    fn ax_range_is_150_degrees_each_side() {
        let edge = 150.0_f32.to_radians();
        for model in [Model::Ax12a, Model::Rx28] {
            let unit = scale(model).rad_per_position;
            assert_close(si(model, 0).0, -edge);
            assert_eq!(position_to_raw(model, Q(-edge)), Some(0));
            assert_eq!(position_to_raw(model, Q(edge - unit)), Some(1023));
            assert_eq!(position_to_raw(model, Q(edge)), None);
            assert_eq!(position_to_raw(model, Q(-edge - unit)), None);
        }
    }

    #[test]
    fn non_finite_positions_are_rejected() {
        for model in MODELS {
            for q in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                assert_eq!(position_to_raw(model, Q(q)), None, "{model:?}");
            }
        }
    }

    #[test]
    fn ax_and_x_scale_differently() {
        assert_eq!(position_to_raw(Model::Xm430W210, Q(FRAC_PI_2)), Some(3072));
        assert_eq!(position_to_raw(Model::Ax12a, Q(FRAC_PI_2)), Some(819));
        assert_eq!(position_to_raw(Model::Rx28, Q(FRAC_PI_2)), Some(819));
        assert_close(si(Model::Xw540T260, 3072).0, FRAC_PI_2);
    }

    #[test]
    fn a_position_outside_the_register_range_is_no_reading() {
        for model in MODELS {
            let max = i32::try_from(scale(model).max_position).unwrap();
            assert_eq!(position_to_si(model, -1), None, "{model:?}");
            assert_eq!(position_to_si(model, i32::MIN), None, "{model:?}");
            assert_eq!(position_to_si(model, max + 1), None, "{model:?}");
            assert!(position_to_si(model, max).is_some(), "{model:?}");
        }
    }

    #[test]
    fn x_velocity_is_signed() {
        let unit = 0.229 * TAU / 60.0;
        for model in X_MODELS {
            assert_close(velocity_to_si(model, 100).0, 100.0 * unit);
            assert_close(velocity_to_si(model, -100).0, -100.0 * unit);
        }
    }

    #[test]
    fn ax_velocity_bit_10_is_clockwise() {
        let unit = 0.111 * TAU / 60.0;
        for model in [Model::Ax12a, Model::Rx28] {
            assert_close(velocity_to_si(model, 100).0, 100.0 * unit);
            assert_close(velocity_to_si(model, 0x400 | 100).0, -100.0 * unit);
            assert_eq!(velocity_to_si(model, 0x400), Qd(0.0));
        }
    }

    #[test]
    fn x_current_uses_the_model_torque_constant() {
        let amps = 1000.0 * 2.69e-3;
        let cases = [
            (Model::Xm430W210, 1.304),
            (Model::Xw430T200, 1.769),
            (Model::Xw540T140, 1.408),
            (Model::Xw540T260, 1.939),
        ];
        for (model, nm_per_amp) in cases {
            let tau = current_to_si(model, 1000).map(|tau| tau.0);
            let back = current_to_si(model, -1000).map(|tau| tau.0);
            assert_close(tau.unwrap(), amps * nm_per_amp);
            assert_close(back.unwrap(), -amps * nm_per_amp);
        }
    }

    #[test]
    fn ax_has_no_current() {
        assert_eq!(current_to_si(Model::Ax12a, 1000), None);
        assert_eq!(current_to_si(Model::Rx28, 1000), None);
    }
}
