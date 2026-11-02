//! `minimal`: the engine alone, for the footprint (research R11, R21 P10; FR-011 (SRS NFR-8 e)).
//!
//! Three no-op controllers and a stop counter, no histogram; after `run()`
//! it writes peak RSS and the stack high-water mark to its report.

use std::process::ExitCode;
use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
use std::time::Duration;

use kmr_api::{EachTick, First, Last, Sensors, Time};
use kmr_bench::plan::{self, BadPlan, RunPlan};
use kmr_bench::report::{self, Flags, Outcome, ReportError, RunInfo};
use kmr_bench::{procfs, setup};

kmr_api::robot!(XmBench);

/// Ticks to run, set by `main` before `run()`.
static PLANNED: AtomicU64 = AtomicU64::new(0);
/// Ticks whose `Last` finished; written by [`stop`] only.
static DONE: AtomicU64 = AtomicU64::new(0);

/// Does nothing.
fn noop(_: &Time, _: &mut State, _: &Sensors) {}

/// Counts the tick and raises `shutdown!` after the planned count.
fn stop(_: &Time, _: &mut State, _: &Sensors) {
    let done = DONE.load(Relaxed) + 1;
    DONE.store(done, Relaxed);
    if done == PLANNED.load(Relaxed) {
        kmr_api::shutdown!("planned ticks done");
    }
}

fn main() -> ExitCode {
    let plan = match RunPlan::from_env() {
        Ok(plan) => plan,
        Err(bad) => return failed_plan(bad),
    };
    PLANNED.store(plan.ticks, Relaxed);
    let result = setup::base::<__Spec>(Duration::from_nanos(plan.dt_ns))
        .add_controller(First, noop)
        .add_controller(EachTick, noop)
        .add_controller(Last, noop)
        .add_controller(Last, stop)
        .run();
    let done = DONE.load(Relaxed);
    let reason = match &result {
        Err(e) => report::text_of(e),
        Ok(()) => report::text_of("stopped before the planned ticks"),
    };
    let outcome = match result {
        Err(_) => Outcome::Failed(reason.as_str()),
        Ok(()) if done < plan.ticks => Outcome::Incomplete(reason.as_str()),
        Ok(()) => Outcome::Complete,
    };
    let (vm_hwm_kb, stack_kb) = (procfs::vm_hwm_kb(), procfs::stack_kb());
    let written = report::write_file(plan.out_path(), |r| {
        r.run_block(&RunInfo {
            measurement: plan.measurement.as_str(),
            cell: plan.cell.as_str(),
            dt_ns: plan.dt_ns,
            ticks: plan.ticks,
            warmup: plan.warmup,
            work_ns: plan.work_ns,
            write: plan.write.as_str(),
            outcome,
            flags: Flags::empty().with_if(Flags::SIMULATED, true),
            ticks_done: done,
            time_surface: "absent",
            calibration: None,
            bench_storage_bytes: storage_bytes(),
        })?;
        r.text("robot", "XmBench")?;
        r.int("robot_n", u64::try_from(N).unwrap_or(u64::MAX))?;
        r.text("buses", "none")?;
        r.text("controllers", "noop/first,each,last+stop/last")?;
        r.int_opt("timer_slack_ns", procfs::timer_slack_ns())?;
        if let Some(mut f) = r.figures(outcome, None)? {
            f.int_opt("vm_hwm_kb", vm_hwm_kb)?;
            f.int_opt("stack_kb", stack_kb)?;
        }
        Ok(())
    });
    exit(written, outcome)
}

/// Bytes of this binary's statics.
fn storage_bytes() -> u64 {
    u64::try_from(size_of_val(&PLANNED) + size_of_val(&DONE)).unwrap_or(u64::MAX)
}

/// Writes the `failed` report of a bad plan when its path is known.
fn failed_plan(bad: BadPlan) -> ExitCode {
    let reason = report::text_of(bad);
    let Some(out) = plan::out_from_env() else {
        eprintln!("minimal: {}", reason.as_str());
        return ExitCode::FAILURE;
    };
    let written = report::write_file(plan::as_path(&out), |r| r.bad_plan(reason.as_str()));
    exit(written, Outcome::Failed(reason.as_str()))
}

/// 0 for `complete` and `incomplete`, 1 for `failed` or a report that could not be written.
fn exit(written: Result<(), ReportError>, outcome: Outcome<'_>) -> ExitCode {
    match written {
        Err(e) => {
            eprintln!("minimal: report not written: {e:?}");
            ExitCode::FAILURE
        }
        Ok(()) if matches!(outcome, Outcome::Failed(_)) => ExitCode::FAILURE,
        Ok(()) => ExitCode::SUCCESS,
    }
}
