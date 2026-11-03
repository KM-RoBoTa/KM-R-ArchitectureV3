//! The `tick` binary end to end: a complete run, a bad plan, Ctrl-C, the allocation counter
//! (quickstart §2, §3, §4; FR-008, FR-012, FR-024 (SRS NFR-8, NFR-1, NFR-2)).
//!
//! Each case runs the binary in its own process with `Command::env`, its output in
//! files read back into fixed buffers.

use std::fs::File;
use std::io::Read;
use std::path::Path;
use std::process::{Command, Stdio};
use std::time::Duration;

use kmr_bench::buf::Buf;

/// Largest report or log read back, in bytes.
const CAP: usize = 64 * 1024;

/// The simulation warning kmr prints once per run without a bus plugin.
const SIM_WARNING: &str = "no bus plugin: simulating";

/// The line a debug build of kmr logs after each late tick.
const DEBUG_OVERRUN_LINE: &str = "tick finished after its deadline";

/// One finished run: exit code, report and log (stdout and stderr together).
struct Run {
    code: Option<i32>,
    report: Buf<CAP>,
    log: Buf<CAP>,
}

impl Run {
    /// The raw value of `key`, quotes kept; `None` when absent.
    fn key(&self, key: &str) -> Option<&str> {
        self.report
            .as_str()
            .lines()
            .find_map(|line| line.strip_prefix(key)?.strip_prefix(" = "))
    }

    /// The integer value of `key`.
    fn int(&self, key: &str) -> Option<u64> {
        self.key(key)?.parse().ok()
    }

    /// Whether `flags` holds `flag`.
    fn flagged(&self, flag: &str) -> bool {
        self.key("flags")
            .is_some_and(|v| v.trim_matches('"').split(',').any(|f| f == flag))
    }

    /// Whether the report has a `[figures]` block.
    fn has_figures(&self) -> bool {
        self.report.as_str().lines().any(|l| l == "[figures]")
    }

    /// How many times the simulation warning shows in the log.
    fn sim_warnings(&self) -> usize {
        self.log.as_str().matches(SIM_WARNING).count()
    }
}

/// Runs `tick` with `vars`, sends SIGINT after `interrupt_after` when given, and reads its output.
fn run(case: &str, vars: &[(&str, &str)], interrupt_after: Option<Duration>) -> Run {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR"));
    let mut name = Buf::<128>::new();
    assert!(name.push(b"tick_runs-") && name.push(case.as_bytes()));
    let report = dir.join(Path::new(name.as_str()).with_extension("toml"));
    let log = dir.join(Path::new(name.as_str()).with_extension("log"));
    let _ = std::fs::remove_file(&report);
    let out = File::create(&log).expect("log file");
    let err = out.try_clone().expect("log file");
    let mut child = Command::new(env!("CARGO_BIN_EXE_tick"))
        .env_remove("RUST_LOG")
        .env("KMR_BENCH_OUT", &report)
        .envs(vars.iter().copied())
        .stdin(Stdio::null())
        .stdout(out)
        .stderr(err)
        .spawn()
        .expect("tick starts");
    if let Some(after) = interrupt_after {
        std::thread::sleep(after);
        let mut pid = Buf::<16>::new();
        assert!(std::fmt::Write::write_fmt(&mut pid, format_args!("{}", child.id())).is_ok());
        let sent = Command::new("kill")
            .args(["-INT", pid.as_str()])
            .status()
            .expect("kill runs");
        assert!(sent.success());
    }
    let code = child.wait().expect("tick ends").code();
    Run {
        code,
        report: read(&report, true),
        log: read(&log, false),
    }
}

/// The file at `path`, empty when missing. A file over [`CAP`] fails when `whole`, else keeps its
/// first bytes: a debug build logs every late tick, and the lines that matter come first.
fn read(path: &Path, whole: bool) -> Buf<CAP> {
    let mut buf = Buf::new();
    let Ok(mut file) = File::open(path) else {
        return buf;
    };
    let mut chunk = [0u8; 4096];
    loop {
        let n = file.read(&mut chunk).expect("readable");
        if n == 0 {
            return buf;
        }
        if !buf.push(&chunk[..n]) {
            assert!(!whole, "{} over {CAP} bytes", path.display());
            let room = CAP - buf.len();
            buf.push(&chunk[..room.min(n)]);
            return buf;
        }
    }
}

#[test]
fn six_thousand_ticks_complete_with_every_jitter_figure() {
    let r = run("complete", &[("KMR_BENCH_TICKS", "6000")], None);
    let text = r.report.as_str();
    assert_eq!(r.code, Some(0), "{text}");
    assert_eq!(r.key("status"), Some("\"complete\""), "{text}");
    assert!(r.flagged("simulated"), "{text}");
    for key in ["jitter_p50_ns", "jitter_p99_ns", "jitter_max_ns"] {
        assert!(r.int(key).is_some(), "{key} missing:\n{text}");
    }
    assert_eq!(r.int("ticks_done"), Some(6000));
    // DECISION: a debug build of kmr logs each late tick at DEBUG (kmr_core runtime.rs), and
    // that line allocates on the loop thread; the test binary is a debug build, so the zero is
    // asserted only when no such line was logged. Release runs log at INFO and never hit it.
    if !r.log.as_str().contains(DEBUG_OVERRUN_LINE) {
        assert_eq!(r.int("loop_thread_allocs"), Some(0), "{text}");
    }
    assert_eq!(r.sim_warnings(), 1, "{}", r.log.as_str());
    for kmr_line in ["kmr_core", "INFO", "WARN", SIM_WARNING] {
        assert!(
            !text.contains(kmr_line),
            "kmr output in the report:\n{text}"
        );
    }
}

#[test]
fn the_engine_count_agrees_at_one_ms() {
    let r = run("engine-1ms", &[("KMR_BENCH_TICKS", "3000")], None);
    let text = r.report.as_str();
    assert_eq!(r.key("time_surface"), Some("\"engine\""), "{text}");
    let bench = r.int("bench_overruns").expect("bench_overruns");
    assert_eq!(r.int("engine_overruns"), Some(bench), "{text}");
    assert!(!r.flagged("inconsistent"), "{text}");
    for key in ["engine_tick", "engine_dt_ns", "engine_elapsed_ns"] {
        assert!(r.int(key).is_some(), "{key} missing:\n{text}");
    }
    assert_eq!(r.int("engine_dt_ns"), Some(1_000_000));
    assert_eq!(r.int("engine_tick"), Some(2999));
}

/// DECISION: at 10 µs the bench count is checked as a lower bound of the engine's, not for
/// equality: a tick that ends between the bench's last stamp and the engine's own end stamp
/// is late for the engine only (probe.rs `ORIGIN`). The `inconsistent` flag still shows it.
#[test]
fn the_engine_count_bounds_the_bench_count_at_ten_us() {
    let r = run(
        "engine-10us",
        &[("KMR_BENCH_TICKS", "21000"), ("KMR_BENCH_DT_NS", "10000")],
        None,
    );
    let text = r.report.as_str();
    assert_eq!(r.key("status"), Some("\"complete\""), "{text}");
    let bench = r.int("bench_overruns").expect("bench_overruns");
    let engine = r.int("engine_overruns").expect("engine_overruns");
    assert!(engine >= bench, "{text}");
    assert_eq!(r.flagged("inconsistent"), engine != bench, "{text}");
}

#[test]
fn a_bad_plan_fails_before_the_run() {
    let r = run("bad-plan", &[("KMR_BENCH_TICKS", "abc")], None);
    assert_eq!(r.code, Some(1));
    assert_eq!(r.key("status"), Some("\"failed\""));
    assert_eq!(r.key("reason"), Some("\"bad plan: KMR_BENCH_TICKS\""));
    assert!(!r.has_figures());
}

#[test]
fn a_dt_below_the_floor_fails_with_the_api_error() {
    let r = run("dt-5us", &[("KMR_BENCH_DT_NS", "5000")], None);
    let text = r.report.as_str();
    assert_eq!(r.code, Some(1), "{text}");
    assert_eq!(r.key("status"), Some("\"failed\""), "{text}");
    assert_eq!(r.int("dt_ns"), Some(5000));
    assert_eq!(
        r.key("reason"),
        Some("\"dt of 5µs is out of range; give `.set_dt(..)` a value from 10 µs to 1 s\""),
        "{text}"
    );
    assert!(!r.has_figures(), "{text}");
}

#[test]
fn ctrl_c_ends_the_run_incomplete_without_figures() {
    let r = run(
        "ctrl-c",
        &[("KMR_BENCH_TICKS", "600000")],
        Some(Duration::from_secs(2)),
    );
    let text = r.report.as_str();
    assert_eq!(r.code, Some(0), "{text}");
    assert_eq!(r.key("status"), Some("\"incomplete\""), "{text}");
    assert!(r.int("ticks_done").is_some_and(|done| done < 600_000));
    assert!(!r.has_figures(), "{text}");
}

#[test]
fn one_allocation_inside_the_window_is_counted() {
    let r = run(
        "inject",
        &[("KMR_BENCH_TICKS", "1100"), ("KMR_BENCH_INJECT_AT", "1010")],
        None,
    );
    let text = r.report.as_str();
    assert_eq!(r.key("status"), Some("\"complete\""), "{text}");
    if r.log.as_str().contains(DEBUG_OVERRUN_LINE) {
        assert!(
            r.int("loop_thread_allocs").is_some_and(|n| n >= 1),
            "{text}"
        );
    } else {
        assert_eq!(r.int("loop_thread_allocs"), Some(1), "{text}");
    }
    assert!(r.flagged("alloc_seen"), "{text}");
}
