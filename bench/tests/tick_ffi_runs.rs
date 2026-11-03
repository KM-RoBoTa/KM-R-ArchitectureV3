//! The `tick_ffi` binary end to end (feature `ffi`): a complete run whose `EachTick` no-op is
//! the empty C body, no allocation on the loop thread (kmr_ffi SC-003; FR-022 (SRS NFR-11)).

use std::process::{Command, Stdio};

/// The line a debug build of kmr logs after each late tick, which allocates.
const DEBUG_OVERRUN_LINE: &str = "tick finished after its deadline";

#[test]
fn six_thousand_ticks_through_the_empty_c_body_complete() {
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR"));
    let report = dir.join("tick_ffi_runs-complete.toml");
    let _ = std::fs::remove_file(&report);
    let out = Command::new(env!("CARGO_BIN_EXE_tick_ffi"))
        .env_remove("RUST_LOG")
        .env("KMR_BENCH_OUT", &report)
        .env("KMR_BENCH_TICKS", "6000")
        .stdin(Stdio::null())
        .output()
        .expect("tick_ffi runs");
    let text = std::fs::read_to_string(&report).expect("a report");
    let logged =
        |bytes: &[u8]| std::str::from_utf8(bytes).is_ok_and(|s| s.contains(DEBUG_OVERRUN_LINE));
    assert_eq!(out.status.code(), Some(0), "{text}");
    assert!(text.contains("status = \"complete\""), "{text}");
    assert!(
        text.contains("controllers = \"probe+ffi_empty/each\""),
        "{text}"
    );
    assert!(text.contains("ticks_done = 6000"), "{text}");
    if !logged(&out.stdout) && !logged(&out.stderr) {
        assert!(text.contains("loop_thread_allocs = 0"), "{text}");
    }
}
