//! The CubeMars adapter: AK60-6 and AK80-8 actuators in MIT mode on raw
//! SocketCAN, one transaction group per CAN interface (SRS FR-5.1, FR-5.3),
//! on the cubemars_rs SDK.
//!
//! The hardware table always builds; the adapter needs the `cubemars`
//! feature. The SDK owns the frames, the zero tracking and the limits; the
//! adapter maps the engine's ticks onto its buses.

#[cfg(feature = "cubemars")]
mod adapter;
mod config;

#[cfg(all(test, feature = "cubemars"))]
mod tests;

#[cfg(feature = "cubemars")]
pub use adapter::{CubeMars, CubeMarsError, SocketCan};
pub use config::{Actuator, Bus, DEFAULT_KD, DEFAULT_KP, Limits, Model, Placement, Table};
