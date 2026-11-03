//! The body of `tick` and `tick_hw`: one measured run, one report (FR-003, FR-008, FR-012, FR-024
//! (SRS NFR-8, NFR-1, NFR-2)).
//!
//! The binaries name the robot with `robot!` and the bus plugins; everything
//! else lives here, so `SERIAL` is the only line two benches differ in.

use std::process::ExitCode;
use std::sync::atomic::Ordering::Relaxed;
use std::time::{Duration, Instant};

use kmr_api::{Bus, EachTick, First, Last, Spec};

use crate::plan::{self, BadPlan, RunPlan};
use crate::report::{self, Calibration, Flags, Outcome, ReportError, RunInfo};
use crate::{alloc, calib, probe, procfs, setup};

/// What a binary tells the run about its robot and buses.
pub struct Bench<'a> {
    /// Binary name, for the stderr line of a report that could not be written.
    pub binary: &'a str,
    /// The robot named in `robot!`.
    pub robot: &'a str,
    /// The `N` that `robot!` emits.
    pub robot_n: usize,
    /// Bus plugins, none for a simulated run.
    pub plugins: &'a [Bus],
    /// The `buses` value of the report: `none`, or `by_id:<serial>` per plugin.
    pub buses: &'a str,
}

/// The `controllers` value of the report.
const CONTROLLERS: &str = "probe+noop/first,each,last";

/// Reads the plan, runs the probed robot of spec `H` and writes the report; 0 unless `failed`.
pub fn main<const N: usize, H: Spec<N>>(bench: &Bench<'_>) -> ExitCode {
    let plan = match RunPlan::from_env() {
        Ok(plan) => plan,
        Err(bad) => return failed_plan(bench.binary, bad),
    };
    let _ = probe::EPOCH.set(Instant::now());
    let ps_before = calib::calibrate();
    let probe_cost_ns = calib::probe_cost_ns(probe::now_ns);
    probe::configure(&plan, ps_before);

    let mut robot = setup::base::<N, H>(Duration::from_nanos(plan.dt_ns));
    for bus in bench.plugins {
        robot = robot.add_plugins(*bus);
    }
    let result = robot
        .add_controller(First, probe::start)
        .add_controller(First, probe::noop)
        .add_controller(EachTick, probe::work)
        .add_controller(EachTick, probe::noop)
        .add_controller(Last, probe::noop)
        .add_controller(Last, probe::end)
        .run();

    let ps_after = calib::calibrate();
    let (vm_hwm_kb, stack_kb) = (procfs::vm_hwm_kb(), procfs::stack_kb());
    let timer_slack_ns = procfs::timer_slack_ns();
    let done = probe::TICKS_DONE.load(Relaxed);
    let allocs = alloc::allocs();
    let drift = calib::drift_permille(ps_before, ps_after);

    let reason = match &result {
        Err(e) => report::text_of(e),
        Ok(()) => report::text_of("stopped before the planned ticks"),
    };
    let outcome = match result {
        Err(_) => Outcome::Failed(reason.as_str()),
        Ok(()) if done < plan.ticks => Outcome::Incomplete(reason.as_str()),
        Ok(()) => Outcome::Complete,
    };
    let complete = outcome == Outcome::Complete;
    let simulated = bench.plugins.is_empty();
    let bench_overruns = probe::BENCH_OVERRUNS.load(Relaxed);
    let engine_overruns = probe::engine_overruns();
    let flags = Flags::empty()
        .with_if(Flags::SIMULATED, simulated)
        .with_if(
            Flags::INCONSISTENT,
            complete && simulated && engine_overruns != bench_overruns,
        )
        .with_if(
            Flags::UNCALIBRATED,
            complete && drift.is_some_and(calib::uncalibrated),
        )
        .with_if(Flags::ALLOC_SEEN, complete && allocs > 0);

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
            flags,
            ticks_done: done,
            time_surface: "engine",
            calibration: Some(Calibration {
                ps_per_iter_before: ps_before,
                ps_per_iter_after: ps_after,
                drift_permille: drift,
                probe_cost_ns,
            }),
            bench_storage_bytes: probe::bench_storage_bytes()
                .saturating_add(calib::storage_bytes()),
        })?;
        r.text("robot", bench.robot)?;
        r.int("robot_n", u64::try_from(bench.robot_n).unwrap_or(u64::MAX))?;
        r.text("buses", bench.buses)?;
        r.text("controllers", CONTROLLERS)?;
        r.int_opt("timer_slack_ns", timer_slack_ns)?;
        if let Some(mut f) = r.figures(outcome, Some(allocs))? {
            f.int_opt("jitter_p50_ns", probe::LATENESS.p50())?;
            f.int_opt("jitter_p99_ns", probe::LATENESS.p99())?;
            f.int_opt("jitter_max_ns", probe::LATENESS.max())?;
            f.int("bench_overruns", bench_overruns)?;
            f.int("engine_overruns", engine_overruns)?;
            f.int_opt("gap_on_time_min_ns", probe::GAP_ON_TIME.min())?;
            f.int_opt("gap_on_time_p50_ns", probe::GAP_ON_TIME.p50())?;
            f.int_opt("gap_after_late_min_ns", probe::GAP_AFTER_LATE.min())?;
            f.int_opt("vm_hwm_kb", vm_hwm_kb)?;
            f.int_opt("stack_kb", stack_kb)?;
            f.int("engine_tick", probe::ENGINE_TICK.load(Relaxed))?;
            f.int("engine_dt_ns", probe::ENGINE_DT_NS.load(Relaxed))?;
            f.int("engine_elapsed_ns", probe::ENGINE_ELAPSED_NS.load(Relaxed))?;
            f.int_opt("engine_last_overrun_ns", probe::engine_last_overrun_ns())?;
        }
        Ok(())
    });
    exit(bench.binary, written, outcome)
}

/// Writes the `failed` report of a bad plan when its path is known.
fn failed_plan(binary: &str, bad: BadPlan) -> ExitCode {
    let reason = report::text_of(bad);
    let Some(out) = plan::out_from_env() else {
        eprintln!("{binary}: {}", reason.as_str());
        return ExitCode::FAILURE;
    };
    let written = report::write_file(plan::as_path(&out), |r| r.bad_plan(reason.as_str()));
    exit(binary, written, Outcome::Failed(reason.as_str()))
}

/// 0 for `complete` and `incomplete`, 1 for `failed` or a report that could not be written.
fn exit(binary: &str, written: Result<(), ReportError>, outcome: Outcome<'_>) -> ExitCode {
    match written {
        Err(e) => {
            eprintln!("{binary}: report not written: {e:?}");
            ExitCode::FAILURE
        }
        Ok(()) if matches!(outcome, Outcome::Failed(_)) => ExitCode::FAILURE,
        Ok(()) => ExitCode::SUCCESS,
    }
}
