//! Foreign-language controllers for kmr: the C ABI of C and C++ controllers, and the Lua bridge.
//!
//! A C or C++ function is a controller of its own, registered in Rust with `kmr_api::c!`. Each
//! tick kmr_api refills a [`CTime`], a [`CState`] and a [`CSensors`] in place, [`invoke`] calls
//! the function once over them, and kmr_api applies what it staged through the normal setters.
//! The generated `kmr.h` implements the whole controller API as `static inline` functions over
//! that snapshot, so C never calls back into Rust. The builder, the loop and the hardware never
//! cross.
//!
//! This crate holds the plain `#[repr(C)]` types that cross, the one field list they are
//! generated from, the build helper that writes the headers, and the Lua bridge's view and
//! [`call`]. It has no kmr dependency; users reach it through `kmr_api`.
//!
//! # Features
//!
//! All off by default; without them no foreign-language code is compiled.
//!
//! - `build`: the build-script helper that writes `kmr.h`, `kmr.hpp` and the ABI file, and
//!   compiles the user's C and C++ sources.
//! - `lua`: `lua::Bridge`, a vendored Lua 5.4; needs only a C compiler. Tables are 1-based.
//!
//! # Safety of the foreign side
//!
//! - A crash or abort in foreign code leaves torque as it was. No software guard can prevent
//!   it. Run the controller on the no-hardware adapter first, the pre-flight lesson of SRS
//!   FR-5.10 (FR-025 (SRS NFR-2)).
//! - C and C++ memory safety is the user's. The `kmr_*` functions check every index against N
//!   and every depth against the History; a raw access to the structs does not.
//! - Slow, allocating or blocking foreign code is allowed: kmr counts the overruns and does not
//!   enforce a language's cost (FR-005 (SRS FR-10.3)). The Lua bridge runs its VM, which
//!   allocates, on every call: that cost is the user's.

#![forbid(clippy::unwrap_used)]
#![deny(clippy::panic, clippy::indexing_slicing)]
#![warn(missing_docs)]

mod c;
mod call;
mod fields;
#[cfg(any(feature = "lua", test))]
mod script;
mod view;

#[cfg(feature = "lua")]
pub mod lua;

// Also under test, so the default test run checks the headers.
#[cfg(any(feature = "build", test))]
pub mod build;

pub use c::{
    CAxes, CDuration, CFn, CImu, CMic, CSensors, CSlot, CState, CTickFaults, CTime, REASON_LEN,
    Request, TICK_OVERRUN, TICK_REMOTE_TIMEOUT, TICK_SENSOR_MISSING, TICK_UNAUTHORIZED, invoke,
};
pub use call::{Foreign, ForeignError, Status, call};
pub use view::{AGE_NONE, ActuatorFlags, Mask, ReadView, Slot, TimeView, WriteBuf, Written};
