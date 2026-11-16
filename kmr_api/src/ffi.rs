//! Controllers written in another language (SRS FR-10), each behind its own feature, all off by
//! default: [`c`] for C and C++ (`ffi`), [`lua`] (`lua`), [`python`] (`python`).

#[cfg(feature = "ffi")]
pub mod c;

#[cfg(feature = "lua")]
pub mod lua;

#[cfg(feature = "python")]
pub mod python;

#[cfg(any(feature = "python", feature = "lua"))]
mod snapshot;
