//! `robot!`: names the robot once, in the user's main.rs (SRS FR-6.1).

/// Names the robot this crate drives and gives it `N`, `State` and `Robot`.
///
/// A KM-RoBoTa preset, or the path of a URDF descriptor (relative to the
/// crate's `Cargo.toml`), read at compile time, with or without its hardware:
///
/// ```ignore
/// kmr_api::robot!(XmBench);
/// kmr_api::robot!(model = "models/arm.urdf");
/// kmr_api::robot!(model = "models/arm.urdf", hardware = { dynamixel: ARM.at([0, 1, 2]) });
/// ```
///
/// `hardware = { brand: part, .. }` names one part per brand, each the
/// member that drives the engine indices the part gives: `dynamixel`, a
/// placed `Table`. A brand named twice does not build. Together the parts
/// cover `0..N` exactly once, which `run()` checks before any I/O. The
/// syntax is assumed, pending human (kmr_api T069).
///
/// Use the generated items instead of importing `kmr_api::{N, State, Robot}`.
///
/// Until `kmr_core` takes N as a const generic, N must equal the core's
/// actuator count: anything else is a compile error. `Rx28Chain`, ten RX-28,
/// builds with the `joints-10` feature only.
///
/// ```compile_fail
/// kmr_api::robot!(model = "tests/fixtures/three.urdf");
/// ```
///
/// Nor does a brand named twice:
///
/// ```compile_fail,E0428
/// use kmr_adapters::dynamixel::{Actuator, Bus, Mode, Model, Table, Torque, Zero};
///
/// const fn xm430(id: u8) -> Actuator {
///     Actuator {
///         bus: 0,
///         id,
///         model: Model::Xm430W210,
///         mode: Mode::Position,
///         torque: Torque::On,
///         zero: Zero::DEFAULT,
///     }
/// }
///
/// const ARM: Table<1, 2> = Table {
///     buses: [Bus { label: "arm", baud: 1_000_000, read_every: None }],
///     actuators: [xm430(1), xm430(2)],
/// };
///
/// kmr_api::robot!(
///     model = "tests/fixtures/four.urdf",
///     hardware = { dynamixel: ARM.at([0, 1]), dynamixel: ARM.at([2, 3]) }
/// );
///
/// fn main() {}
/// ```
#[macro_export]
macro_rules! robot {
    ($preset:ident) => {
        $crate::robot!(@items <$crate::presets::$preset as $crate::presets::Preset>::N);
        /// Bus count of this robot's hardware table.
        #[doc(hidden)]
        #[allow(dead_code)]
        const __NB: usize = $crate::presets::$preset::HARDWARE.buses.len();
        /// Actuator count of this robot's hardware table.
        #[doc(hidden)]
        #[allow(dead_code)]
        const __M: usize = $crate::presets::$preset::HARDWARE.actuators.len();
        /// This robot's hardware table.
        #[doc(hidden)]
        #[allow(dead_code)]
        const __TABLE: $crate::Table<__NB, __M> = $crate::presets::$preset::HARDWARE;
        impl $crate::Spec for __Spec {
            type Drivers = $crate::HCons<$crate::__DxlAdapter<__NB>, $crate::HNil>;

            fn bind(
                plugins: &[$crate::Binding],
            ) -> ::core::result::Result<Self::Drivers, $crate::UserError> {
                let hardware = $crate::HCons {
                    head: __TABLE.in_order(),
                    tail: $crate::HNil,
                };
                $crate::__bind(hardware, plugins)
            }
        }
    };
    (model = $path:literal) => {
        $crate::robot!(@items $crate::__actuator_count!($path));
        impl $crate::Spec for __Spec {
            type Drivers = $crate::HCons<$crate::__NoHardware, $crate::HNil>;

            fn bind(
                plugins: &[$crate::Binding],
            ) -> ::core::result::Result<Self::Drivers, $crate::UserError> {
                $crate::__bind_none(plugins)
            }
        }
    };
    (model = $path:literal, hardware = { $($brand:ident : $part:expr),+ $(,)? }) => {
        $crate::robot!(@items $crate::__actuator_count!($path));
        /// This robot's hardware, one module per brand.
        #[doc(hidden)]
        mod __hardware {
            $(
                /// The part of this robot's hardware of one brand.
                #[allow(non_snake_case)]
                pub(super) mod $brand {
                    #[allow(unused_imports)]
                    use super::super::*;
                    /// Bus and actuator counts of the part.
                    const SHAPE: (usize, usize) = $crate::__brand::$brand::shape(&$part);
                    /// The type of the part.
                    pub(in super::super) type Part =
                        $crate::__brand::$brand::Hardware<{ SHAPE.0 }, { SHAPE.1 }>;
                    /// The part.
                    pub(in super::super) const PART: Part = $part;
                }
            )+
        }
        impl $crate::Spec for __Spec {
            type Drivers =
                <$crate::robot!(@parts $($brand),+) as $crate::Brands>::Drivers;

            fn bind(
                plugins: &[$crate::Binding],
            ) -> ::core::result::Result<Self::Drivers, $crate::UserError> {
                $crate::__bind($crate::robot!(@list $($brand),+), plugins)
            }
        }
    };
    (@parts $brand:ident $(, $rest:ident)*) => {
        $crate::HCons<__hardware::$brand::Part, $crate::robot!(@parts $($rest),*)>
    };
    (@parts) => {
        $crate::HNil
    };
    (@list $brand:ident $(, $rest:ident)*) => {
        $crate::HCons {
            head: __hardware::$brand::PART,
            tail: $crate::robot!(@list $($rest),*),
        }
    };
    (@list) => {
        $crate::HNil
    };
    (@items $n:expr) => {
        /// Actuator count of this robot.
        #[allow(dead_code)]
        pub const N: usize = $n;
        /// The state handle of this robot.
        #[allow(dead_code)]
        pub type State<'a> = $crate::State<'a>;
        /// The builder of this robot.
        #[allow(dead_code)]
        pub type Robot = $crate::Robot<__Spec>;
        /// This robot, as the type parameter of its builder.
        #[doc(hidden)]
        pub struct __Spec;
        const _: () = assert!(
            N == $crate::N,
            "kmr: this robot's actuator count differs from kmr_core's; N as a const generic is pending in kmr_core"
        );
    };
}
