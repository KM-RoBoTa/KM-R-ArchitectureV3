//! SIGTERM (a service stop) and SIGHUP (a lost SSH session) end a run like
//! Ctrl-C does: the engine's handler raises `shutdown!` for all three.
//!
//! Own test binary with one test: the handler installs once per process.

use std::process::Command;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;

use kmr_core::{EachTick, Last, Robot, RobotState, Sensors, Time};

const TICK_BUDGET: u32 = 5_000;

#[test]
fn sigterm_then_sighup_each_end_a_run_as_a_shutdown() {
    for kill in ["kill -TERM $PPID", "kill -HUP $PPID"] {
        let ticks = AtomicU32::new(0);
        let lasts = AtomicU32::new(0);

        let result = Robot::new()
            .set_dt(Duration::from_millis(1))
            .add_control_fn(EachTick, |_t: &Time, _s: &mut RobotState, _se: &Sensors| {
                let n = ticks.fetch_add(1, Ordering::Relaxed) + 1;
                assert!(n < TICK_BUDGET, "the loop ignored `{kill}`");
                if n == 1 {
                    // `$PPID` of the shell is this test process.
                    let sent = Command::new("sh").args(["-c", kill]).status();
                    assert!(sent.is_ok_and(|status| status.success()), "`{kill}` failed");
                }
            })
            .add_control_fn(Last, |_t: &Time, _s: &mut RobotState, _se: &Sensors| {
                lasts.fetch_add(1, Ordering::Relaxed);
            })
            .run();

        assert_eq!(result, Ok(()), "{kill}");
        // A shutdown completes the tick that saw it.
        assert_eq!(lasts.load(Ordering::Relaxed), ticks.load(Ordering::Relaxed));
    }
}
