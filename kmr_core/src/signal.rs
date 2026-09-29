// ===========================================================================
// signal.rs — the ONLY channel from user controller code back into the
// control loop.
//
// A signal is emitted with the `shutdown!` / `emergency_stop!` macros so it can
// be raised from ANY depth: a nested helper that never received `&mut State`, a
// `.map()` closure, a controller on its own thread, or a ctrlc/SIGINT handler.
// That reach is the whole point — a `State` method could not be called there.
//
//   shutdown!("battery low");        // graceful: one last write, then exit
//   emergency_stop!("estop button"); // immediate: skip write, cut torque
//
// ── Why a macro, not a free fn ───────────────────────────────────────────────
// The project forbids heap types (`Box`/`Vec`/`Arc`/… — see clippy.toml): the
// control loop must not allocate. So the metadata behind the bus pointer cannot
// be a heap `Box<SignalMeta>`; it must be a `'static`. A per-emit `'static` that
// still carries a call-site reason + file + line can only be minted at the CALL
// SITE — that is exactly what a `macro_rules!` does. Each expansion drops a
// `static SignalMeta` into the caller and hands the bus a thin pointer to it.
//
// Bonus: because nothing allocates, the whole emit path is async-signal-safe —
// it is genuinely sound to call from a raw signal handler, not just from the
// `ctrlc` crate's dedicated handler thread.
//
// ── Mechanism ────────────────────────────────────────────────────────────────
// A process-global, LOCK-FREE bus. A Mutex is wrong here twice over: poisoning
// silently drops a signal if a holder panicked, and a Mutex is not async-signal
// -safe. Atomics are the correct class.
//
//   LEVEL: AtomicU8                 0 = None, 1 = Shutdown, 2 = EmergencyStop
//   META:  AtomicPtr<SignalMeta>    -> the FIRST winner's 'static metadata
//
// `compare_exchange` raises the level and never lowers it: an emergency stop can
// never be downgraded to a graceful shutdown, and a prior estop is never lost.
// META points only ever at a `'static`, so there is nothing to free — `reset`
// just nulls it.
//
// ── Source location: DEBUG ONLY ──────────────────────────────────────────────
// `file!()`/`line!()` are baked into the `static SignalMeta` only under
// `debug_assertions`. A release build stores `"<release>"` / `0`; diagnostics
// are a debug affordance and never touch the release layout.
// ===========================================================================

use std::sync::atomic::{AtomicPtr, AtomicU8, Ordering};
use std::{fmt, ptr};

// ── Priority levels ─────────────────────────────────────────────────────────
pub(crate) const NONE: u8 = 0;
pub(crate) const SHUTDOWN: u8 = 1;
pub(crate) const EMERGENCY_STOP: u8 = 2;

// ── Global bus ───────────────────────────────────────────────────────────────
static LEVEL: AtomicU8 = AtomicU8::new(NONE);
static META: AtomicPtr<SignalMeta> = AtomicPtr::new(ptr::null_mut());

/// Metadata attached to a signal — reason, source file, and line.
///
/// Always lives in a `'static` minted by the emit macros; never heap-allocated.
/// In release builds `file`/`line` are placeholders (see the module header).
#[derive(Debug, Clone, Copy)]
pub struct SignalMeta {
    reason: &'static str,
    file: &'static str,
    line: u32,
}

impl SignalMeta {
    /// `const` so the emit macros can place it in a `static`. Not for user code;
    /// call `shutdown!` / `emergency_stop!` instead.
    #[doc(hidden)]
    pub const fn new(reason: &'static str, file: &'static str, line: u32) -> Self {
        Self { reason, file, line }
    }
}

/// The kind of stop requested.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignalKind {
    /// Graceful shutdown — write one final desired state, then exit.
    Shutdown,
    /// Emergency stop — skip `write_state`, cut torque immediately.
    EmergencyStop,
}

/// A stop signal drained by the runtime after a controller tick.
///
/// Raise one with the [`shutdown!`] / [`emergency_stop!`] macros — never
/// construct this directly.
#[derive(Debug, Clone, Copy)]
pub struct Signal {
    pub(crate) kind: SignalKind,
    pub(crate) meta: SignalMeta,
}

impl Signal {
    /// The kind of stop requested.
    pub(crate) fn kind(&self) -> SignalKind {
        self.kind
    }

    // ── Macro-backing emitters (public, hidden — call the macros instead) ────

    #[doc(hidden)]
    pub fn _emit_shutdown(meta: &'static SignalMeta) {
        emit(SHUTDOWN, meta);
    }

    #[doc(hidden)]
    pub fn _emit_emergency_stop(meta: &'static SignalMeta) {
        emit(EMERGENCY_STOP, meta);
    }

    // ── Runtime-only hooks (not visible to users) ────────────────────────────

    /// Reset the bus. Call ONCE at runtime entry.
    ///
    /// The bus is process-global and outlives any single run. Without this a
    /// signal left over from a previous `start()` would trigger an immediate
    /// exit on the next run. META only ever points at a `'static`, so there is
    /// nothing to free — just null it.
    pub(crate) fn reset() {
        META.store(ptr::null_mut(), Ordering::Release);
        LEVEL.store(NONE, Ordering::Release);
    }

    /// Drain the bus. Returns `None` on the happy path.
    ///
    /// Called by the runtime after every controller tick. The happy-path load
    /// is a single `AtomicU8` read (~1 ns) so it is cheap even at a 10 kHz tick.
    pub(crate) fn drain() -> Option<Signal> {
        let level = LEVEL.load(Ordering::Acquire);
        if level == NONE {
            return None; // fast bail, every tick
        }

        let kind = match level {
            EMERGENCY_STOP => SignalKind::EmergencyStop,
            _ => SignalKind::Shutdown,
        };

        // META is published AFTER LEVEL is raised, so a concurrent emit from
        // another thread can leave a window where LEVEL is set but META is still
        // null. Fall back to a placeholder rather than crash — the signal itself
        // is never lost, only its diagnostics are momentarily absent. In
        // practice drain runs on the main loop a tick later; the window is never
        // observed.
        let ptr = META.load(Ordering::Acquire);
        let meta = if ptr.is_null() {
            SignalMeta::new("<meta pending>", "<unknown>", 0)
        } else {
            // SAFETY: a non-null META always points at a `'static SignalMeta`
            // minted by an emit macro. A `'static` is valid for the whole
            // program, so the deref can never dangle.
            *unsafe { &*ptr }
        };

        Some(Signal { kind, meta })
    }
}

/// Raise the bus to `level`, publishing `meta` iff this call is the one that
/// raised it. Monotonic: never lowers the level, so estop can never be
/// downgraded and a prior estop is never lost. Allocation-free → async-signal-
/// safe.
fn emit(level: u8, meta: &'static SignalMeta) {
    let mut cur = LEVEL.load(Ordering::Relaxed);
    loop {
        if cur >= level {
            return; // already at/above this level — first winner keeps its meta
        }
        match LEVEL.compare_exchange_weak(cur, level, Ordering::AcqRel, Ordering::Relaxed) {
            Ok(_) => {
                // We raised the level; publish our thin 'static pointer.
                META.store(
                    meta as *const SignalMeta as *mut SignalMeta,
                    Ordering::Release,
                );
                return;
            }
            Err(actual) => cur = actual, // lost the race; re-check against reality
        }
    }
}

impl fmt::Display for Signal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let kind = match self.kind {
            SignalKind::Shutdown => "shutdown",
            SignalKind::EmergencyStop => "EMERGENCY STOP",
        };
        write!(
            f,
            "{kind}: {} (at {}:{})",
            self.meta.reason, self.meta.file, self.meta.line
        )
    }
}

// ── Emit macros ──────────────────────────────────────────────────────────────
//
// Each expansion mints a per-call-site `static SignalMeta` (const, zero alloc)
// and hands the bus a thin pointer to it. `file!()`/`line!()` are folded in only
// under `debug_assertions`.

/// Request a graceful shutdown from inside a controller.
///
/// Callable from any depth. In debug builds the call site's file/line are
/// recorded; in release they are omitted.
#[macro_export]
macro_rules! shutdown {
    ($reason:expr $(,)?) => {{
        #[cfg(debug_assertions)]
        static META: $crate::SignalMeta = $crate::SignalMeta::new($reason, file!(), line!());
        #[cfg(not(debug_assertions))]
        static META: $crate::SignalMeta = $crate::SignalMeta::new($reason, "<release>", 0);
        $crate::Signal::_emit_shutdown(&META);
    }};
}

/// Request an emergency stop from inside a controller.
///
/// Always wins over a graceful shutdown and is never downgraded. In debug builds
/// the call site's file/line are recorded; in release they are omitted.
#[macro_export]
macro_rules! emergency_stop {
    ($reason:expr $(,)?) => {{
        #[cfg(debug_assertions)]
        static META: $crate::SignalMeta = $crate::SignalMeta::new($reason, file!(), line!());
        #[cfg(not(debug_assertions))]
        static META: $crate::SignalMeta = $crate::SignalMeta::new($reason, "<release>", 0);
        $crate::Signal::_emit_emergency_stop(&META);
    }};
}

#[cfg(test)]
mod test {
    use super::*;

    // These tests mutate one process-global bus, so they must not run in
    // parallel. `cargo test` shares a process per test binary, hence one fn.
    #[test]
    fn bus_semantics() {
        // 1. depth: macro emits from a nested helper with no State in scope.
        fn helper() {
            crate::shutdown!("battery low");
        }
        Signal::reset();
        helper();
        let s = Signal::drain().expect("a signal was emitted");
        assert_eq!(s.kind(), SignalKind::Shutdown);

        // 2. estop wins over shutdown, order-independent, no downgrade.
        Signal::reset();
        crate::shutdown!("graceful first");
        crate::emergency_stop!("estop second");
        crate::shutdown!("graceful third");
        let s = Signal::drain().expect("a signal was emitted");
        assert_eq!(s.kind(), SignalKind::EmergencyStop);

        // 3. reset clears stale signals across runs.
        Signal::reset();
        assert!(Signal::drain().is_none());

        // 4. estop from a background thread is never lost under contention.
        //    Fixed array, no `Vec` (a disallowed type).
        Signal::reset();
        let handles: [std::thread::JoinHandle<()>; 8] = std::array::from_fn(|i| {
            std::thread::spawn(move || {
                if i == 3 {
                    crate::emergency_stop!("thread estop");
                } else {
                    crate::shutdown!("thread shutdown");
                }
            })
        });
        for h in handles {
            h.join().expect("thread joined");
        }
        let s = Signal::drain().expect("a signal was emitted");
        assert_eq!(s.kind(), SignalKind::EmergencyStop);
    }
}
