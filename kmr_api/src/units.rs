//! Unit markers that convert into the state values (SRS FR-2.2).
//!
//! `Q` is rad on a revolute actuator and mm on a prismatic one; `Qd` is rad/s or mm/s. A marker
//! does not know the kind of the actuator it is written to: `Q::from(Deg(..))` on a prismatic
//! actuator compiles and is wrong.
//!
//! [`Celsius`] and [`Volts`] carry actuator telemetry, read-only (SRS FR-5.11): they convert
//! into nothing.

use crate::{Q, Qd};

/// An angle in degrees, into `Q` as rad.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Deg(pub f32);

/// A length in centimetres, into `Q` as mm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cm(pub f32);

/// A length in metres, into `Q` as mm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct M(pub f32);

/// An angular speed in degrees per second, into `Qd` as rad/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DegPerS(pub f32);

/// A speed in centimetres per second, into `Qd` as mm/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CmPerS(pub f32);

/// A speed in metres per second, into `Qd` as mm/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MPerS(pub f32);

/// A temperature in degrees Celsius, as an actuator reports it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Celsius(pub f32);

/// A voltage in volts, as an actuator reports its input.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Volts(pub f32);

impl From<Deg> for Q {
    fn from(v: Deg) -> Self {
        Q(v.0.to_radians())
    }
}

impl From<Cm> for Q {
    fn from(v: Cm) -> Self {
        Q(v.0 * 10.0)
    }
}

impl From<M> for Q {
    fn from(v: M) -> Self {
        Q(v.0 * 1000.0)
    }
}

impl From<DegPerS> for Qd {
    fn from(v: DegPerS) -> Self {
        Qd(v.0.to_radians())
    }
}

impl From<CmPerS> for Qd {
    fn from(v: CmPerS) -> Self {
        Qd(v.0 * 10.0)
    }
}

impl From<MPerS> for Qd {
    fn from(v: MPerS) -> Self {
        Qd(v.0 * 1000.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() <= f32::EPSILON * b.abs().max(1.0)
    }

    #[test]
    fn positions_convert_to_rad_and_mm() {
        assert!(close(Q::from(Deg(180.0)).0, PI));
        assert!(close(Q::from(Deg(-90.0)).0, -PI / 2.0));
        assert!(close(Q::from(M(1.0)).0, 1000.0));
        assert!(close(Q::from(Cm(1.0)).0, 10.0));
    }

    #[test]
    fn speeds_convert_to_rad_and_mm_per_second() {
        assert!(close(Qd::from(DegPerS(180.0)).0, PI));
        assert!(close(Qd::from(MPerS(1.0)).0, 1000.0));
        assert!(close(Qd::from(CmPerS(1.0)).0, 10.0));
    }

    #[test]
    fn zero_stays_zero() {
        assert_eq!(Q::from(Deg(0.0)), Q(0.0));
        assert_eq!(Qd::from(DegPerS(0.0)), Qd(0.0));
    }
}
