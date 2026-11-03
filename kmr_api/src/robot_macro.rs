//! `robot!`: names the robot once, in the user's main.rs (SRS FR-6.1).

/// Names the robot this crate drives and gives it `N`, `State` and `Robot`.
///
/// A KM-RoBoTa preset, or the path of a `.urdf`, `.ron` or `.json` descriptor
/// (relative to the crate's `Cargo.toml`), read at compile time by
/// `kmr_model`, with or without its hardware:
///
/// ```ignore
/// kmr_api::robot!(XmBench);
/// kmr_api::robot!(XmBench, hardware = |hw| hw.read_only(3).baud("bus0", 3_000_000));
/// kmr_api::robot!(model = "models/arm.urdf");
/// kmr_api::robot!(model = "models/arm.urdf", hardware = ARM);
/// kmr_api::robot!(model = "models/arm.urdf", hardware = { dynamixel: ARM.at([0, 1, 2]) });
/// ```
///
/// `|hw| ..` changes a preset's table through its `const fn` overrides; an
/// override that names an index or a label the table does not have does not
/// build. `hardware = ARM` is one [`dynamixel::Table`](crate::dynamixel::Table),
/// one row per actuator in model order: a row count other than `N` does not
/// build. Both forms are assumed, pending human (D7 A).
///
/// `hardware = { brand: part, .. }` names one part per brand, each the
/// member that drives the engine indices the part gives: `dynamixel`, a
/// placed [`dynamixel::Table`](crate::dynamixel::Table). A brand named
/// twice does not build, nor do parts that do not cover `0..N` exactly
/// once. The syntax is assumed, pending human (kmr_api T069).
///
/// The model gives `N`, the `model` module of constants (names, limits, links)
/// and `actuator!("name")`, the index of an actuator. Use the generated items
/// instead of importing `kmr_api::{N, State, Robot}`.
///
/// Until `kmr_core` takes N as a const generic, N must equal the core's
/// actuator count: anything else is a compile error (kmr_api/tests/ui/n_differs_from_core.rs).
/// `Rx28Chain`, ten RX-28, builds with the `joints-10` feature only. More than 64
/// actuators never build (SRS write mask; D4 A, assumed, pending human).
///
/// A brand named twice does not build:
///
/// ```compile_fail,E0428
/// use kmr_api::dynamixel::{Actuator, Bus, Mode, Model, Table, Torque, Zero};
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
///
/// Nor do parts that leave an actuator without a member:
///
/// ```compile_fail,E0080
/// use kmr_api::dynamixel::{Actuator, Bus, Mode, Model, Table, Torque, Zero};
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
/// kmr_api::robot!(model = "tests/fixtures/four.urdf", hardware = { dynamixel: ARM.at([0, 1]) });
///
/// fn main() {}
/// ```
#[macro_export]
macro_rules! robot {
    ($preset:ident) => {
        $crate::robot!($preset, hardware = |hw| hw);
    };
    ($preset:ident, hardware = |$hw:ident| $table:expr) => {
        $crate::__model!($crate; preset = $preset);
        $crate::__robot_items!();
        /// Bus count of this robot's hardware table, the preset's.
        #[doc(hidden)]
        #[allow(dead_code)]
        const __NB: usize = $crate::presets::$preset::HARDWARE.buses.len();
        /// Actuator count of this robot's hardware table, the preset's.
        #[doc(hidden)]
        #[allow(dead_code)]
        const __M: usize = $crate::presets::$preset::HARDWARE.actuators.len();
        /// This robot's hardware table: the preset's, through the overrides.
        #[doc(hidden)]
        #[allow(dead_code)]
        const __TABLE: $crate::Table<__NB, __M> = {
            let $hw = $crate::presets::$preset::HARDWARE;
            $table
        };
        $crate::__robot_items!(@table);
    };
    (model = $path:literal) => {
        $crate::__model!($crate; model = $path);
        $crate::__robot_items!();
        impl $crate::Spec<{ N }> for __Spec {
            type Drivers = $crate::HCons<$crate::__NoHardware, $crate::HNil>;

            const LINK_NAMES: &'static [&'static str] = &model::LINK_NAMES;

            fn bind(
                plugins: &[$crate::Binding],
            ) -> ::core::result::Result<Self::Drivers, $crate::UserError> {
                $crate::__bind_none(plugins)
            }
        }
    };
    (model = $path:literal, hardware = { $($brand:ident : $part:expr),+ $(,)? }) => {
        $crate::__model!($crate; model = $path);
        $crate::__robot_items!();
        /// This robot's hardware, one module per brand.
        #[doc(hidden)]
        mod __hardware {
            $(
                /// The part of this robot's hardware of one brand.
                #[allow(non_snake_case)]
                pub(super) mod $brand {
                    // `$part` names the user's items through this glob: the items below
                    // keep `__` names so none of them shadows one.
                    #[allow(unused_imports)]
                    use super::super::*;
                    /// Bus and actuator counts of the part.
                    const __SHAPE: (usize, usize) = $crate::__brand::$brand::shape(&$part);
                    /// The type of the part.
                    pub(in super::super) type __Part =
                        $crate::__brand::$brand::Hardware<{ __SHAPE.0 }, { __SHAPE.1 }>;
                    /// The part.
                    pub(in super::super) const __PART: __Part = $part;
                }
            )+
        }
        const _: () = $crate::__check_coverage(
            &[$($crate::__brand::$brand::indices(&__hardware::$brand::__PART)),+],
            N,
        );
        impl $crate::Spec<{ N }> for __Spec {
            type Drivers =
                <$crate::robot!(@parts $($brand),+) as $crate::Brands<{ N }>>::Drivers;

            const LINK_NAMES: &'static [&'static str] = &model::LINK_NAMES;

            fn bind(
                plugins: &[$crate::Binding],
            ) -> ::core::result::Result<Self::Drivers, $crate::UserError> {
                $crate::__bind::<{ N }, _>($crate::robot!(@list $($brand),+), plugins)
            }
        }
    };
    (model = $path:literal, hardware = $table:expr) => {
        $crate::__model!($crate; model = $path);
        $crate::__robot_items!();
        // Read off the table's arrays, never its placement, so a row count other than `N`
        // reaches the count check as one E0308 (research R7).
        /// Bus count of this robot's hardware table.
        #[doc(hidden)]
        #[allow(dead_code)]
        const __NB: usize = $table.buses.len();
        /// Actuator count of this robot's hardware table.
        #[doc(hidden)]
        #[allow(dead_code)]
        const __M: usize = $table.actuators.len();
        /// This robot's hardware table.
        #[doc(hidden)]
        #[allow(dead_code)]
        const __TABLE: $crate::Table<__NB, __M> = $table;
        $crate::__robot_items!(@table);
    };
    (@parts $brand:ident $(, $rest:ident)*) => {
        $crate::HCons<__hardware::$brand::__Part, $crate::robot!(@parts $($rest),*)>
    };
    (@parts) => {
        $crate::HNil
    };
    (@list $brand:ident $(, $rest:ident)*) => {
        $crate::HCons {
            head: __hardware::$brand::__PART,
            tail: $crate::robot!(@list $($rest),*),
        }
    };
    (@list) => {
        $crate::HNil
    };
}

/// The items of `robot!` that do not come from the model: the aliases, `__Spec` and the
/// checks on `N`; with `@table`, the count check and `Spec` of a one-table robot. Hidden, so
/// a user cannot set `N` through it (003 review #7).
#[doc(hidden)]
#[macro_export]
macro_rules! __robot_items {
    (@table) => {
        // The count check: one table row per actuator, in model order (FR-007 (SRS FR-6.4)).
        const _: [$crate::dynamixel::Actuator; N] = __TABLE.actuators;
        impl $crate::Spec<{ N }> for __Spec {
            type Drivers = $crate::HCons<$crate::__DxlAdapter<__NB, { N }>, $crate::HNil>;

            const LINK_NAMES: &'static [&'static str] = &model::LINK_NAMES;

            fn bind(
                plugins: &[$crate::Binding],
            ) -> ::core::result::Result<Self::Drivers, $crate::UserError> {
                let hardware = $crate::HCons {
                    head: __TABLE.in_order(),
                    tail: $crate::HNil,
                };
                $crate::__bind::<{ N }, _>(hardware, plugins)
            }
        }
    };
    () => {
        /// The state handle of this robot.
        #[allow(dead_code)]
        pub type State<'a> = $crate::State<'a, { N }>;
        /// The builder of this robot.
        #[allow(dead_code)]
        pub type Robot = $crate::Robot<{ N }, __Spec>;
        /// This robot, as the type parameter of its builder.
        #[doc(hidden)]
        pub struct __Spec;
        const _: () = assert!(
            N == $crate::N,
            "kmr: this robot's actuator count differs from kmr_core's; N as a const generic is pending in kmr_core"
        );
        const _: () = assert!(N >= 1 && N <= 64, "SRS write mask: more than 64 actuators");
    };
}

#[cfg(test)]
mod test;
