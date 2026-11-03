//
// bench is the reference benchmark: a crate of small binaries that link
// kmr_api like any user would (black-box, never kmr_core), plus the shell
// scripts that run them on the bench host. Drawn as it is on this branch.

!element kmr.bench {
    benchSetup = component "bench: setup" "base(dt): the robot every bench binary starts from, through kmr_api's builder only, Warn on every limit (D2 B, the one place; SRS NFR-8, NFR-1)." "Rust"
    benchPlan = component "bench: run plan" "What one process measures, read once from KMR_BENCH_* before run(); a bad variable fails with bad plan: <var> (SRS NFR-2)." "Rust"
    benchRecorder = component "bench: recorder" "The statics the probe controllers write each tick and main reads after run(): epoch, grid slot, last end and lateness, tick and overrun counts, lateness and overruns split into read and write-only ticks by the actuators' age at First (SRS FR-3.4). All-zero, so in .bss; one writer, relaxed load and store." "Rust statics, atomics"
    benchMeasure = component "bench: measurement" "Fixed histograms of ns samples, the replay of the engine's tick grid from the bench's own stamps, and the calibrated workload of the budget method with its drift check." "Rust, no heap"
    benchReport = component "bench: report" "Writes a report's [run] and [figures] blocks to KMR_BENCH_OUT through a fixed buffer, and reads footprint and host facts from /proc/self." "Rust, no heap"
    benchBins = component "bench: binaries" "tick (the probed 1 kHz run, simulated) and tick_hw (the same on the reference chain, feature hardware: read and write-only ticks apart, read cadence and declared costs), minimal (the engine with three no-op controllers and a stop counter) and baseline (a process that never builds a robot): peak RSS and stack for the footprint (SRS NFR-8 e)." "Rust binaries"
    benchResults = component "bench: results tool" "results compare <A> <B> and results summary <dir>: reads report sets into fixed arrays, integers only, never a verdict; refuses across hosts, robots or kernels (SRS NFR-11, NFR-1)." "Rust binary"
    benchScripts = component "bench: scripts" "profile.env (every setting and bench_run), meta.sh (the [meta] block), setup-check.sh (the host against SRS NFR-1), cell.sh, ceiling.sh, budget.sh (--hw: the simulated search, then tick_hw on the reference chain, io_write_tick_ns and io_read_ns next to the engine overhead), footprint.sh, presort.sh (default build against the presort-off build: cost simulated, --gain on the chain through results compare), all.sh, compile-time.sh (clean and incremental builds, the registration sweep from gen/registrations.rs.in)." "Bash"

    benchBins -> benchSetup "Build their robot with"
    benchBins -> benchPlan "Read their plan with"
    benchBins -> benchReport "Write their figures with"
    benchSetup -> kmr.control "Builds the robot through kmr_api's builder, as a user would" "kmr_api"
    benchReport -> benchMeasure "Reads the figures of"
    benchReport -> benchPlan "Writes to the output path of"
    benchScripts -> benchBins "Builds and runs"
    benchScripts -> benchResults "Summarizes the reports with"
    benchResults -> benchReport "Reads the reports written by" "TOML files"

    group "bench: run plan" {
        benchRunPlan = component "plan::RunPlan" "dt, ticks, warm-up, work, write mode, output path; from_env() or parse()." "Rust struct" {
            tags "Code"
        }
        benchWriteMode = component "plan::WriteMode" "Which actuators the probe writes each tick." "Rust enum" {
            tags "Code"
        }
    }

    group "bench: measurement" {
        benchHistogram = component "hist::Histogram" "20 000 buckets of 100 ns plus overflow, static-ready." "Rust struct" {
            tags "Code"
        }
        benchGrid = component "grid" "slot_start, deadline, lateness, overran, next_slot: the engine's absolute grid, replayed." "Rust fns" {
            tags "Code"
        }
        benchCalib = component "calib" "spin, calibrate, iters_for, drift_permille, uncalibrated." "Rust fns" {
            tags "Code"
        }
    }

    group "bench: report" {
        benchBuf = component "buf::Buf<CAP>" "Fixed-capacity byte buffer for write!." "Rust struct" {
            tags "Code"
        }
        benchOutcome = component "report::Outcome" "complete or failed with a reason, the report's status." "Rust enum" {
            tags "Code"
        }
        benchProcfs = component "procfs" "vm_hwm_kb, stack_kb, timer_slack_ns over a chunked line scanner." "Rust fns" {
            tags "Code"
        }
    }

    group "bench: results tool" {
        benchParse = component "results::parse::Report" "One report as byte spans into a 64 KiB buffer; unknown keys kept." "Rust struct" {
            tags "Code"
        }
        benchCompare = component "results::compare::compare" "Every figure of two sets side by side." "Rust fn" {
            tags "Code"
        }
        benchSummary = component "results::summary::summary" "One set, every cell side by side, naming what is missing." "Rust fn" {
            tags "Code"
        }
        benchExit = component "results::Exit" "0 printed, 1 unreadable input, 2 refusal." "Rust enum" {
            tags "Code"
        }
    }

    benchRunPlan -> benchWriteMode "Holds"
    benchCompare -> benchParse "Reads"
    benchSummary -> benchParse "Reads"
    benchCompare -> benchExit "Returns"
    benchSummary -> benchExit "Returns"
    benchOutcome -> benchBuf "Is written through"
}
