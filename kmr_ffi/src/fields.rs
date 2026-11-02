//! The one declaration of what crosses: the state fields and the per-actuator flags.
//!
//! Every per-field and per-flag item is generated from these two lists: the Rust types, the
//! C and C++ headers and the bridges (FR-014 (SRS FR-10.2)). A new field is one line here.

/// Calls `$cb!` with every field of the core's sealed set, as `name: Type` pairs.
///
/// `for_each_field!(cb)` expands to `cb! { q: Q, qd: Qd, tau: Tau }`. Tokens after the callback
/// come first, ended by `;`: `for_each_field!(cb, 4)` expands to `cb! { 4; q: Q, .. }`.
#[macro_export]
macro_rules! for_each_field {
    ($cb:ident) => {
        $cb! { q: Q, qd: Qd, tau: Tau }
    };
    ($cb:ident, $($pre:tt)*) => {
        $cb! { $($pre)*; q: Q, qd: Qd, tau: Tau }
    };
}

/// Calls `$cb!` with every per-actuator flag of one History slot (core K4.1, SRS FR-13.4).
///
/// `envelope` is per actuator: assumed, pending human: D3 A. Tokens after the callback come
/// first, ended by `;`, as in [`for_each_field!`].
#[macro_export]
macro_rules! for_each_flag {
    ($cb:ident) => {
        $cb! { stale, bus_error, offline, rejected, joint_limit, envelope }
    };
    ($cb:ident, $($pre:tt)*) => {
        $cb! { $($pre)*; stale, bus_error, offline, rejected, joint_limit, envelope }
    };
}
