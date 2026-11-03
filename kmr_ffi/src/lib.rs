//! Foreign-language controller math for kmr: C, C++, Python and Lua.
//!
//! A Rust controller stays the one registered on a phase. It hands one tick's math to a
//! foreign function: kmr_api's glue copies the state into a [`ReadView`] and the clock into a
//! [`TimeView`], the function fills a [`WriteBuf`], and the glue applies only the flagged
//! entries through the normal setters. The builder, the loop and the hardware never cross.
//!
//! This crate holds the plain `#[repr(C)]` types that cross and the one field list they are
//! generated from. It has no kmr dependency; users reach it through `kmr_api::ffi`.
//!
//! # Safety of the foreign side
//!
//! - A crash or abort in foreign code leaves torque as it was. No software guard can prevent
//!   it. Run the controller on the no-hardware adapter first, the pre-flight lesson of SRS
//!   FR-5.10 (FR-025 (SRS NFR-2)).
//! - C and C++ memory safety is the user's. Every array is N long and the History is
//!   `HISTORY_DEPTH` slots long; `kmr_set_<field>` ignores an index past N, a raw array write
//!   does not.
//! - Slow, allocating or blocking foreign code is allowed: kmr counts the overruns and does not
//!   enforce a language's cost (FR-005 (SRS FR-10.3)).

#![forbid(clippy::unwrap_used)]
#![deny(clippy::panic, clippy::indexing_slicing)]
#![warn(missing_docs)]

mod call;
mod fields;
mod view;

// Also under test, so the default test run checks the headers.
#[cfg(any(feature = "build", test))]
pub mod build;

pub use call::{CFn, Foreign, ForeignError, Status, call};
pub use view::{AGE_NONE, ActuatorFlags, Mask, ReadView, Slot, TimeView, WriteBuf, Written};
