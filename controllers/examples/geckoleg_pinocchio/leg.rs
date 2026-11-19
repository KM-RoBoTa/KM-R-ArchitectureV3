//! What the geckoleg_pinocchio example and its tests share: the contexts of the C++ controllers and the joint
//! limits handed to the actuators.

// DECISION: the example's own module, which the tests include through `#[path]`, so they check
// the very limits the example declares; `controllers` has no library to share it from.

/// Lowest motor angle of each joint over the gait, rad: hip, knee, ankle.
///
/// The Pinocchio simulation of two cycles at 5 ms (`check_gait` in cpp/geckoleg.cpp,
/// lo = [-0.0064, -0.6188, -0.5490]), rounded outward to 0.01 rad; `check_gait_cpp` stops the run
/// at startup when the gait leaves it.
pub const RANGE_LO: [f64; 3] = [-0.01, -0.62, -0.55];
/// Highest motor angle of each joint over the gait, rad: hip, knee, ankle.
///
/// As [`RANGE_LO`], from hi = [-0.0046, 0.5046, 0.4628].
pub const RANGE_HI: [f64; 3] = [0.0, 0.51, 0.47];
/// Peak joint speed over the gait at SPEED 1, rad/s: 7.234 simulated, rounded up.
pub const PEAK: f64 = 7.3;

/// `Gait` of cpp/geckoleg.cpp, the context of `walk_cpp`. Only C++ reads it.
#[repr(C)]
#[allow(dead_code)]
pub struct Gait {
    /// Model time per wall second.
    pub speed: f64,
    /// Last goals, rad, hip, knee, ankle: where the next Newton steps start.
    pub prev: [f64; 3],
    /// Wall time of the next printout, s.
    pub next_print: f64,
}

impl Gait {
    /// The gait at the all-zero posture, before its first printout.
    pub const fn new(speed: f64) -> Self {
        Self {
            speed,
            prev: [0.0; 3],
            next_print: 0.0,
        }
    }
}

/// `Limits` of cpp/geckoleg.cpp, the context of `check_gait_cpp`. Only C++ reads it.
#[repr(C)]
#[allow(dead_code)]
pub struct Limits {
    /// Lowest motor angle of each joint, rad.
    pub lo: [f64; 3],
    /// Highest motor angle of each joint, rad.
    pub hi: [f64; 3],
    /// Peak joint speed at SPEED 1, rad/s.
    pub peak: f64,
}

/// The limits the actuators get, which `check_gait_cpp` checks the gait against.
pub const LIMITS: Limits = Limits {
    lo: RANGE_LO,
    hi: RANGE_HI,
    peak: PEAK,
};
