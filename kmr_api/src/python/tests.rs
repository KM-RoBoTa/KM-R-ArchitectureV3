//! The phase names, the reload swap and the parameter check, without a run.

use super::*;
use crate::{EachTick, PostInit, PreInit};

/// Writes `body` to a fresh script for test `name`; returns its folder and file name.
fn script(name: &str, body: &str) -> (String, String) {
    let dir = std::env::temp_dir().join(format!("kmr_api_py_{name}_{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir is writable");
    std::fs::write(dir.join("s.py"), body).expect("temp dir is writable");
    (dir.to_string_lossy().into_owned(), "s.py".to_owned())
}

/// Calls the runtime's current function with its arguments; returns `ctx.v`.
fn run_once(rt: &Runtime) -> Option<i64> {
    Python::attach(|py| {
        let call = rt.call.as_ref()?;
        call.function.bind(py).call1(call.args.bind(py)).ok()?;
        rt.objects.ctx.bind(py).getattr("v").ok()?.extract().ok()
    })
}

/// Rewrites the script, then polls and reloads as the watcher and the loop would.
fn edit(rt: &mut Runtime, dir: &str, body: &str) {
    std::fs::write(Path::new(dir).join("s.py"), body).expect("temp dir is writable");
    let mut seen = Seen {
        modified: None,
        source: String::new(),
    };
    rt.watch.0.poll(&mut seen);
    Python::attach(|py| rt.reload(py));
}

#[test]
fn a_phase_is_named_in_snake_case() {
    assert_eq!(phase_name::<EachTick>(), "each_tick");
    assert_eq!(phase_name::<PostInit>(), "post_init");
    assert_eq!(phase_name::<PreInit>(), "pre_init");
}

#[test]
fn an_edit_swaps_the_function_and_keeps_ctx() {
    let (dir, file) = script(
        "swap",
        "def each_tick(ctx):\n    ctx.v = (ctx.v or 0) + 1\n",
    );
    let mut rt = Script::__load(&dir, &file, None, 4)
        .expect("loads")
        .runtime::<EachTick>();
    assert_eq!(run_once(&rt), Some(1));

    edit(
        &mut rt,
        &dir,
        "def each_tick(ctx):\n    ctx.v = ctx.v + 10\n",
    );
    assert_eq!(run_once(&rt), Some(11), "the new body, the old ctx");
}

#[test]
fn a_broken_edit_keeps_the_previous_version() {
    let (dir, file) = script("broken", "def each_tick(ctx):\n    ctx.v = 1\n");
    let mut rt = Script::__load(&dir, &file, None, 4)
        .expect("loads")
        .runtime::<EachTick>();

    edit(&mut rt, &dir, "def each_tick(ctx:\n");
    assert_eq!(run_once(&rt), Some(1), "a syntax error");
    edit(&mut rt, &dir, "def other(ctx):\n    ctx.v = 2\n");
    assert_eq!(run_once(&rt), Some(1), "the function gone");
}

#[test]
fn an_unknown_parameter_leaves_no_call() {
    let (dir, file) = script("param", "def each_tick(state, foo):\n    pass\n");
    let rt = Script::__load(&dir, &file, None, 4)
        .expect("loads")
        .runtime::<EachTick>();
    assert!(rt.call.is_none());
}

#[test]
fn the_globals_hold_n_and_the_depth() {
    let (dir, file) = script(
        "globals",
        "def each_tick(ctx):\n    ctx.v = N * 100 + HISTORY_DEPTH\n",
    );
    let rt = Script::__load(&dir, &file, None, 7)
        .expect("loads")
        .runtime::<EachTick>();
    assert_eq!(run_once(&rt), Some(7 * 100 + HISTORY_DEPTH as i64));
}
