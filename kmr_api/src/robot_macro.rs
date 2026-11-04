//! `robot!`: names the robot once, in the user's main.rs (SRS FR-6.1).

/// Names the robot this crate drives and gives it `N`, `State` and `Robot`.
///
/// A KM-RoBoTa preset, whose hardware table comes with it, or the path of a
/// URDF descriptor (relative to the crate's `Cargo.toml`), read at compile
/// time, whose actuators `Robot::add_actuator` declares:
///
/// ```ignore
/// kmr_api::robot!(XmBench);
/// kmr_api::robot!(model = "models/arm.urdf");
/// ```
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
            type Adapter = $crate::__DxlAdapter<__NB>;
            const PRESET: ::core::option::Option<&'static str> =
                ::core::option::Option::Some(::core::stringify!($preset));

            fn bind(
                plugins: &[$crate::Binding],
            ) -> ::core::result::Result<Self::Adapter, $crate::UserError> {
                $crate::__bind_dynamixel(__TABLE, plugins)
            }

            fn declare(
                _declared: $crate::__Declared,
            ) -> ::core::result::Result<Self::Adapter, $crate::UserError> {
                $crate::__declare_none(::core::stringify!($preset))
            }
        }
    };
    (model = $path:literal) => {
        $crate::robot!(@items $crate::__actuator_count!($path));
        impl $crate::Spec for __Spec {
            type Adapter = $crate::__DxlAdapter<{ $crate::N }>;
            const PRESET: ::core::option::Option<&'static str> = ::core::option::Option::None;

            fn bind(
                plugins: &[$crate::Binding],
            ) -> ::core::result::Result<Self::Adapter, $crate::UserError> {
                $crate::__bind_none(plugins)
            }

            fn declare(
                declared: $crate::__Declared,
            ) -> ::core::result::Result<Self::Adapter, $crate::UserError> {
                $crate::__declare_dynamixel(declared)
            }
        }
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
