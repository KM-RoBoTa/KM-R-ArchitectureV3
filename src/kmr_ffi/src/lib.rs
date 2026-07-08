//! C ABI shim over `kmr_api`.
//!
//! Exposes a flat, `extern "C"` surface that any C-compatible host can link
//! against. The Rust trait `Controller` is replaced by a C function pointer;
//! the owned `RobotState` handed to that callback is passed as an opaque
//! pointer that is only valid for the duration of the call.
//!
//! Symbols (see `include/kmr_ffi.h`):
//!   - `kmr_run`            register a C callback and run the control loop
//!   - `kmr_robot_set_q_at` write one desired joint value through the handle
//!
//! `kmr_core` is never named here — only `kmr_api`'s public surface is wrapped.

use kmr_api::{Controller, RobotState};

/// Result codes returned across the C boundary. Mirrors `kmr_api::Error`.
#[repr(C)]
pub enum KmrStatus {
    /// Operation succeeded.
    Ok = 0,
    /// Index passed to `kmr_robot_set_q_at` was out of range.
    OutOfRange = 1,
    /// A required pointer argument was null.
    NullPointer = 2,
}

/// C callback signature. Receives a borrowed, opaque robot handle that is
/// valid only until the callback returns.
pub type KmrController = extern "C" fn(robot: *mut RobotState);

/// Bridges a C function pointer to the Rust `Controller` trait.
struct CController(KmrController);

impl Controller for CController {
    fn main_controller(&self, mut robot: RobotState) {
        // `robot` lives on this stack frame for the whole call, so the raw
        // pointer the callback receives stays valid until it returns.
        (self.0)(&mut robot as *mut RobotState);
    }
}

/// Register a C callback and run the control loop once.
///
/// `controller` must be non-null and must remain a valid function pointer for
/// the duration of the call. Blocks until the control loop returns.
#[unsafe(no_mangle)]
pub extern "C" fn kmr_run(controller: KmrController) {
    kmr_api::run(CController(controller));
}

/// Write `value` to desired joint `index` through an opaque robot handle.
///
/// # Safety
/// `robot` must be a handle obtained from the `kmr_run` callback and must not
/// be used after that callback returns.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kmr_robot_set_q_at(
    robot: *mut RobotState,
    value: f32,
    index: usize,
) -> KmrStatus {
    let Some(robot) = (unsafe { robot.as_mut() }) else {
        return KmrStatus::NullPointer;
    };
    match robot.set_q_at(value, index) {
        Ok(()) => KmrStatus::Ok,
        Err(_) => KmrStatus::OutOfRange,
    }
}
