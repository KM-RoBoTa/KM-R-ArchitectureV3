//! The one call into foreign code: the C function type, its status and its errors.
//!
//! [`call`] clears the write buffer, runs the foreign function once and, on an error, clears it
//! again, so an error applies nothing (FR-012 (SRS FR-2.5)). The C path is the crate's only
//! `unsafe` block; no heap, no lock, no `dyn` on it (FR-021, FR-022 (SRS NFR-1, NFR-2)).

use core::ffi::c_void;
use core::ptr;

use crate::view::{ReadView, TimeView, WriteBuf};

/// A C or C++ controller body, the signature `foreign!` declares and `kmr_ffi.h` documents.
///
/// A function item declared in an `extern "C"` block is not this type until cast:
/// `swing as CFn<N, D>`.
pub type CFn<const N: usize, const D: usize> = unsafe extern "C" fn(
    *const ReadView<N, D>,
    *mut WriteBuf<N>,
    *const TimeView,
    *mut c_void,
) -> i32;

/// How a foreign call ended: the codes 0, 1, 2 of the C ABI (assumed, pending human: D13 A).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    /// 0: the tick goes on.
    Continue,
    /// 1: the forwarder raises `shutdown!`.
    Shutdown,
    /// 2: the forwarder raises `emergency_stop!`.
    EmergencyStop,
}

impl Status {
    /// The status of a C return code; `None` for anything outside `0..=2`.
    pub const fn from_code(rc: i32) -> Option<Self> {
        match rc {
            0 => Some(Self::Continue),
            1 => Some(Self::Shutdown),
            2 => Some(Self::EmergencyStop),
            _ => None,
        }
    }
}

/// A foreign call or a bridge load that failed. Nothing of the call is applied.
// DECISION: `String` payloads, as data-model.md says; built only on the error path of the
// bridges, never on the C path.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ForeignError {
    /// A C or C++ body returned a negative or unknown code; a bridge's wrong return is `-1`.
    #[error("foreign controller returned {0}")]
    Code(i32),
    /// A Python exception, as text.
    #[error("python: {0}")]
    Python(String),
    /// A Lua error, as text.
    #[error("lua: {0}")]
    Lua(String),
    /// A bridge failed to start or to load its script.
    #[error("foreign controller did not load: {0}")]
    Load(String),
}

/// Something [`call`] can run once per tick: a [`CFn`] or a script bridge.
///
/// `ctx` is the controller's context; the C path passes it as `void *`, null for `None`. The
/// bridges ignore it: a script keeps its own data.
pub trait Foreign<const N: usize, const D: usize> {
    /// Runs the body once over `read` and `time`, writing into `buf`.
    ///
    /// # Errors
    ///
    /// [`ForeignError`] when the body fails; [`call`] then clears `buf`.
    fn invoke<C>(
        &mut self,
        read: &ReadView<N, D>,
        buf: &mut WriteBuf<N>,
        time: &TimeView,
        ctx: Option<&mut C>,
    ) -> Result<Status, ForeignError>;
}

impl<const N: usize, const D: usize> Foreign<N, D> for CFn<N, D> {
    fn invoke<C>(
        &mut self,
        read: &ReadView<N, D>,
        buf: &mut WriteBuf<N>,
        time: &TimeView,
        ctx: Option<&mut C>,
    ) -> Result<Status, ForeignError> {
        let ctx = ctx.map_or(ptr::null_mut(), |c| ptr::from_mut(c).cast::<c_void>());
        // SAFETY: every pointer comes from a live reference of the exact type the C side
        // declares, valid for the whole call: `read` and `time` shared, `buf` and `ctx`
        // exclusive (or `ctx` null). The body keeping a pointer past the call, writing through
        // a `const` one or past N is the user's C memory safety (crate docs, FR-025).
        let rc = unsafe { (*self)(read, buf, time, ctx) };
        Status::from_code(rc).ok_or(ForeignError::Code(rc))
    }
}

impl<const N: usize, const D: usize, F: Foreign<N, D>> Foreign<N, D> for &mut F {
    fn invoke<C>(
        &mut self,
        read: &ReadView<N, D>,
        buf: &mut WriteBuf<N>,
        time: &TimeView,
        ctx: Option<&mut C>,
    ) -> Result<Status, ForeignError> {
        (**self).invoke(read, buf, time, ctx)
    }
}

/// Runs `f` once: clears `buf`, calls, and on an error clears `buf` again.
///
/// kmr_api's glue fills `read` and `time` before and applies the flagged entries of `buf`
/// after (data-model.md § One call).
///
/// # Errors
///
/// What `f` returns; `buf` is then all zero with no entry flagged (FR-012 (SRS FR-2.5)).
pub fn call<const N: usize, const D: usize, F, C>(
    mut f: F,
    read: &ReadView<N, D>,
    buf: &mut WriteBuf<N>,
    time: &TimeView,
    ctx: Option<&mut C>,
) -> Result<Status, ForeignError>
where
    F: Foreign<N, D>,
{
    *buf = WriteBuf::default();
    let out = f.invoke(read, buf, time, ctx);
    if out.is_err() {
        *buf = WriteBuf::default();
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A C body that writes nothing and returns 0.
    extern "C" fn noop(
        _: *const ReadView<4, 4>,
        _: *mut WriteBuf<4>,
        _: *const TimeView,
        _: *mut c_void,
    ) -> i32 {
        0
    }

    #[test]
    fn codes_map_to_statuses() {
        assert_eq!(Status::from_code(0), Some(Status::Continue));
        assert_eq!(Status::from_code(1), Some(Status::Shutdown));
        assert_eq!(Status::from_code(2), Some(Status::EmergencyStop));
        assert_eq!(Status::from_code(3), None);
        assert_eq!(Status::from_code(-1), None);
    }

    #[test]
    fn a_c_body_is_called_through_the_one_unsafe_block() {
        let mut buf = WriteBuf::default();
        let out = call(
            noop as CFn<4, 4>,
            &ReadView::default(),
            &mut buf,
            &TimeView::default(),
            None::<&mut ()>,
        );
        assert_eq!(out, Ok(Status::Continue));
        assert_eq!(buf, WriteBuf::default());
    }
}
