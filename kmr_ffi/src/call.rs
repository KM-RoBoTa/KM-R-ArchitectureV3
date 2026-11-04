//! The one call into foreign code: the C function type, its status and its errors.
//!
//! [`call`] clears the write buffer, runs the foreign function once and, on an error, clears it
//! again, so an error applies nothing (FR-012 (SRS FR-2.5)). [`foreign!`](crate::foreign)
//! declares the C bodies. The C path is the crate's only `unsafe` block; no heap, no lock, no
//! `dyn` on it (FR-021, FR-022 (SRS NFR-1, NFR-2)).

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

/// Declares C and C++ controller bodies built by the build helper, and checks they were built
/// for this robot.
///
/// Once per crate, after `robot!`, whose `N` it resolves in scope: `foreign! { depth = D; fn
/// swing; fn other; }`. kmr_api's `ffi::foreign!` passes the core's `HISTORY_DEPTH` as `D`. It
/// includes `OUT_DIR/kmr_ffi_abi.rs` from the build helper and fails `cargo check` with
/// `error[E0080]` when the C sources were built for another N or depth (SC-005 (SRS FR-10.2,
/// FR-6.1)). Each name becomes a `const` of type [`CFn<N, D>`](CFn), the C function itself,
/// so it passes to [`call`] as is.
// DECISION: a `const` per name over an extern block in a hidden module, so `call(swing, ..)`
// needs no `as CFn<..>` cast at the call site.
#[macro_export]
macro_rules! foreign {
    (depth = $depth:expr; $(fn $name:ident;)+) => {
        ::core::include!(::core::concat!(::core::env!("OUT_DIR"), "/kmr_ffi_abi.rs"));
        const _: () = ::core::assert!(FFI_N == N, "kmr_ffi: C sources were built for another N");
        const _: () = ::core::assert!(
            FFI_DEPTH == $depth,
            "kmr_ffi: C sources were built for another History depth"
        );
        #[doc(hidden)]
        mod __kmr_ffi_c {
            #[allow(unused_imports)]
            use super::*;
            // SAFETY: the build helper compiled these bodies against `kmr_ffi.h` of the same
            // build, whose prototypes are this signature; the asserts above pin its N and depth.
            // Calling one stays `unsafe`: `call` does it once, over references of these exact
            // types.
            unsafe extern "C" {
                $(
                    pub(super) fn $name(
                        read: *const $crate::ReadView<N, { $depth }>,
                        write: *mut $crate::WriteBuf<N>,
                        time: *const $crate::TimeView,
                        ctx: *mut ::core::ffi::c_void,
                    ) -> i32;
                )+
            }
        }
        $(
            #[doc = ::core::concat!("The C or C++ controller body `", ::core::stringify!($name), "`.")]
            #[allow(non_upper_case_globals)]
            const $name: $crate::CFn<N, { $depth }> = __kmr_ffi_c::$name;
        )+
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::view::Written;

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

    /// A C body that sets `q[1]` = 5 and returns the code in `ctx` (an `i32`).
    unsafe extern "C" fn write_then(
        _: *const ReadView<4, 4>,
        w: *mut WriteBuf<4>,
        _: *const TimeView,
        ctx: *mut c_void,
    ) -> i32 {
        // SAFETY: `call` passes a live exclusive buffer and, in these tests, an `i32` context.
        let (w, rc) = unsafe { (&mut *w, *ctx.cast::<i32>()) };
        w.q[1] = 5.0;
        w.written.q |= 1 << 1;
        rc
    }

    /// A C body that counts its calls in a `u32` context.
    unsafe extern "C" fn count(
        _: *const ReadView<4, 4>,
        _: *mut WriteBuf<4>,
        _: *const TimeView,
        ctx: *mut c_void,
    ) -> i32 {
        // SAFETY: the test passes an exclusive `u32` context.
        unsafe { *ctx.cast::<u32>() += 1 };
        0
    }

    /// A C body that returns 0 for a null context and -1 otherwise.
    extern "C" fn null_ctx(
        _: *const ReadView<4, 4>,
        _: *mut WriteBuf<4>,
        _: *const TimeView,
        ctx: *mut c_void,
    ) -> i32 {
        if ctx.is_null() { 0 } else { -1 }
    }

    /// One call of [`write_then`] returning `rc`, on a dirty buffer.
    fn run(rc: i32) -> (Result<Status, ForeignError>, WriteBuf<4>) {
        let mut buf = WriteBuf::default();
        buf.qd[3] = 9.0;
        buf.written.qd = 1 << 3;
        let mut rc = rc;
        let out = call(
            write_then as CFn<4, 4>,
            &ReadView::default(),
            &mut buf,
            &TimeView::default(),
            Some(&mut rc),
        );
        (out, buf)
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

    #[test]
    fn an_error_code_clears_the_writes() {
        let (out, buf) = run(-7);
        assert_eq!(out, Err(ForeignError::Code(-7)));
        assert_eq!(buf, WriteBuf::default());
        assert_eq!(buf.written, Written::default());
    }

    #[test]
    fn statuses_keep_the_writes_of_the_call_only() {
        for (rc, status) in [
            (0, Status::Continue),
            (1, Status::Shutdown),
            (2, Status::EmergencyStop),
        ] {
            let (out, buf) = run(rc);
            assert_eq!(out, Ok(status));
            assert_eq!(buf.q[1], 5.0);
            assert_eq!(buf.written.q, 1 << 1);
            assert_eq!(buf.written.qd, 0, "the buffer is cleared before the call");
        }
    }

    #[test]
    fn an_unknown_code_is_an_error() {
        let (out, buf) = run(3);
        assert_eq!(out, Err(ForeignError::Code(3)));
        assert_eq!(buf, WriteBuf::default());
    }

    #[test]
    fn the_context_reaches_the_body() {
        let mut n = 0u32;
        let mut buf = WriteBuf::default();
        for _ in 0..1000 {
            let out = call(
                count as CFn<4, 4>,
                &ReadView::default(),
                &mut buf,
                &TimeView::default(),
                Some(&mut n),
            );
            assert_eq!(out, Ok(Status::Continue));
        }
        assert_eq!(n, 1000);
    }

    #[test]
    fn no_context_passes_null() {
        let mut buf = WriteBuf::default();
        let mut x = 0u8;
        let f = null_ctx as CFn<4, 4>;
        let read = ReadView::default();
        let time = TimeView::default();
        assert_eq!(
            call(f, &read, &mut buf, &time, None::<&mut u8>),
            Ok(Status::Continue)
        );
        assert_eq!(
            call(f, &read, &mut buf, &time, Some(&mut x)),
            Err(ForeignError::Code(-1))
        );
    }
}
