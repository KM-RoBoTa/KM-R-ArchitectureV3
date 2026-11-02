//! Foreign-language controller math for kmr: C, C++, Python and Lua.
//!
//! A Rust controller stays the one registered on a phase. It hands one tick's math to a
//! foreign function: kmr_api's glue copies the state into a [`ReadView`] and the clock into a
//! [`TimeView`], the function fills a [`WriteBuf`], and the glue applies only the flagged
//! entries through the normal setters. The builder, the loop and the hardware never cross.
//!
//! This crate holds the plain `#[repr(C)]` types that cross and the one field list they are
//! generated from. It has no kmr dependency; users reach it through `kmr_api::ffi`.

#![forbid(clippy::unwrap_used)]
#![deny(clippy::panic, clippy::indexing_slicing)]
#![warn(missing_docs)]

mod fields;
mod view;

pub use view::{AGE_NONE, ActuatorFlags, Mask, ReadView, Slot, TimeView, WriteBuf, Written};
