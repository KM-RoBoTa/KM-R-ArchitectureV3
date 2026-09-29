//! A process that installed its own Ctrl-C handler before `run()`.
//!
//! The handler of a process can be installed once and the tracing subscriber
//! too: this file is its own test binary, hence its own process, and holds
//! ONE test. A second one would find both already in place.

use std::fmt::{self, Write};
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;

use kmr_core::{EachTick, Robot, RobotState, Sensors, Time, shutdown};
use tracing::field::{Field, Visit};
use tracing::span::{Attributes, Id, Record};
use tracing::{Event, Level, Metadata, Subscriber};

static CTRL_C_WARNINGS: AtomicU32 = AtomicU32::new(0);

/// Looks for "Ctrl-C" in what is written to it, without storing the text.
struct Finder(bool);

impl Write for Finder {
    fn write_str(&mut self, chunk: &str) -> fmt::Result {
        self.0 |= chunk.contains("Ctrl-C");
        Ok(())
    }
}

impl Visit for Finder {
    fn record_debug(&mut self, field: &Field, value: &dyn fmt::Debug) {
        if field.name() == "message" {
            let _ = write!(self, "{value:?}");
        }
    }
}

/// Counts the warnings about Ctrl-C, drops everything else.
struct Warnings;

impl Subscriber for Warnings {
    fn enabled(&self, metadata: &Metadata<'_>) -> bool {
        *metadata.level() <= Level::WARN
    }

    fn event(&self, event: &Event<'_>) {
        let mut finder = Finder(false);
        event.record(&mut finder);
        if finder.0 && *event.metadata().level() == Level::WARN {
            CTRL_C_WARNINGS.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn new_span(&self, _span: &Attributes<'_>) -> Id {
        Id::from_u64(1)
    }
    fn record(&self, _span: &Id, _values: &Record<'_>) {}
    fn record_follows_from(&self, _span: &Id, _follows: &Id) {}
    fn enter(&self, _span: &Id) {}
    fn exit(&self, _span: &Id) {}
}

#[test]
fn a_run_that_cannot_wire_ctrl_c_says_so() {
    static TICKS: AtomicU32 = AtomicU32::new(0);

    // Ends the run by itself: with a handler that raises nothing, nothing
    // else would. The tick budget keeps a loop that ignores it from hanging.
    fn stop(_t: &Time, _s: &mut RobotState, _se: &Sensors) {
        let n = TICKS.fetch_add(1, Ordering::Relaxed) + 1;
        assert!(n < 500, "the loop ignored the signal");
        if n >= 3 {
            shutdown!("test: enough ticks");
        }
    }

    tracing::subscriber::set_global_default(Warnings).expect("first subscriber of the process");
    ctrlc::set_handler(|| {}).expect("first handler of the process");

    for run in 1..=2 {
        let result = Robot::new()
            .set_dt(Duration::from_micros(200))
            .add_control_fn(EachTick, stop)
            .run();

        assert_eq!(result, Ok(()));
        // Every run: the handler in place is still not the one of the engine.
        assert_eq!(CTRL_C_WARNINGS.load(Ordering::Relaxed), run);
    }
}
