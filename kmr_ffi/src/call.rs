//! The script bridges' call: the status, the errors and the body trait the Lua bridge
//! implements.
//!
//! [`call`] clears the write buffer, runs the body once and, on an error, clears it again, so an
//! error applies nothing (FR-012 (SRS FR-2.5)). C and C++ controllers do not go through it: they
//! are called by [`invoke`](crate::invoke) over the [`CState`](crate::CState) snapshot.
// DECISION: kept only for the Lua bridge, removed later with it; its view types stay with it.

use crate::view::{ReadView, TimeView, WriteBuf};

/// How a script call ended: the codes 0, 1, 2 of the bridges (assumed, pending human: D13 A).
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
    /// The status of a return code; `None` for anything outside `0..=2`.
    pub const fn from_code(rc: i32) -> Option<Self> {
        match rc {
            0 => Some(Self::Continue),
            1 => Some(Self::Shutdown),
            2 => Some(Self::EmergencyStop),
            _ => None,
        }
    }
}

/// A script call or a bridge load that failed. Nothing of the call is applied.
// DECISION: `String` payloads, as data-model.md says; built only on the error path of the
// bridges.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ForeignError {
    /// A wrong return value of a script is `-1`.
    #[error("foreign controller returned {0}")]
    Code(i32),
    /// A Lua error, as text.
    #[error("lua: {0}")]
    Lua(String),
    /// A bridge failed to start or to load its script.
    #[error("foreign controller did not load: {0}")]
    Load(String),
}

/// Something [`call`] can run once per tick: a script bridge.
///
/// `ctx` is the controller's context; the bridges ignore it: a script keeps its own data.
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
    use crate::view::Written;

    #[test]
    fn codes_map_to_statuses() {
        assert_eq!(Status::from_code(0), Some(Status::Continue));
        assert_eq!(Status::from_code(1), Some(Status::Shutdown));
        assert_eq!(Status::from_code(2), Some(Status::EmergencyStop));
        assert_eq!(Status::from_code(3), None);
        assert_eq!(Status::from_code(-1), None);
    }

    /// A body that sets `q[1]` = 5 and returns its result.
    struct WriteThen(Result<Status, ForeignError>);

    impl Foreign<4, 4> for WriteThen {
        fn invoke<C>(
            &mut self,
            _: &ReadView<4, 4>,
            buf: &mut WriteBuf<4>,
            _: &TimeView,
            _: Option<&mut C>,
        ) -> Result<Status, ForeignError> {
            buf.q[1] = 5.0;
            buf.written.q |= 1 << 1;
            self.0.clone()
        }
    }

    /// One call of [`WriteThen`] returning `out`, on a dirty buffer.
    fn run(out: Result<Status, ForeignError>) -> (Result<Status, ForeignError>, WriteBuf<4>) {
        let mut buf = WriteBuf::default();
        buf.qd[3] = 9.0;
        buf.written.qd = 1 << 3;
        let got = call(
            WriteThen(out),
            &ReadView::default(),
            &mut buf,
            &TimeView::default(),
            None::<&mut ()>,
        );
        (got, buf)
    }

    #[test]
    fn an_error_clears_the_writes() {
        let (out, buf) = run(Err(ForeignError::Code(-7)));
        assert_eq!(out, Err(ForeignError::Code(-7)));
        assert_eq!(buf, WriteBuf::default());
        assert_eq!(buf.written, Written::default());
    }

    #[test]
    fn statuses_keep_the_writes_of_the_call_only() {
        for status in [Status::Continue, Status::Shutdown, Status::EmergencyStop] {
            let (out, buf) = run(Ok(status));
            assert_eq!(out, Ok(status));
            assert_eq!(buf.q[1], 5.0);
            assert_eq!(buf.written.q, 1 << 1);
            assert_eq!(buf.written.qd, 0, "the buffer is cleared before the call");
        }
    }
}
