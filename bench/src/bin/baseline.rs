//! `baseline`: the pages touched before `main`, for the footprint (research R11; FR-011 (SRS NFR-8 e)).
//!
//! It never builds a robot nor calls `run()`: peak RSS and the stack pages of
//! a process that only reads them.

use std::process::ExitCode;

use kmr_bench::plan::{self, RunPlan};
use kmr_bench::procfs;
use kmr_bench::report::{self, Flags, Outcome, RunInfo};

fn main() -> ExitCode {
    let (vm_hwm_kb, stack_kb) = (procfs::vm_hwm_kb(), procfs::stack_kb());
    let plan = match RunPlan::from_env() {
        Ok(plan) => plan,
        Err(bad) => {
            let reason = report::text_of(bad);
            let Some(out) = plan::out_from_env() else {
                eprintln!("baseline: {}", reason.as_str());
                return ExitCode::FAILURE;
            };
            if let Err(e) = report::write_file(plan::as_path(&out), |r| r.bad_plan(reason.as_str()))
            {
                eprintln!("baseline: report not written: {e:?}");
            }
            return ExitCode::FAILURE;
        }
    };
    let written = report::write_file(plan.out_path(), |r| {
        r.run_block(&RunInfo {
            measurement: plan.measurement.as_str(),
            cell: plan.cell.as_str(),
            dt_ns: plan.dt_ns,
            ticks: plan.ticks,
            warmup: plan.warmup,
            work_ns: plan.work_ns,
            write: plan.write.as_str(),
            outcome: Outcome::Complete,
            flags: Flags::empty(),
            ticks_done: 0,
            time_surface: "absent",
            calibration: None,
            bench_storage_bytes: 0,
        })?;
        if let Some(mut f) = r.figures(Outcome::Complete, None)? {
            f.int_opt("vm_hwm_baseline_kb", vm_hwm_kb)?;
            f.int_opt("stack_baseline_kb", stack_kb)?;
        }
        Ok(())
    });
    match written {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("baseline: report not written: {e:?}");
            ExitCode::FAILURE
        }
    }
}
