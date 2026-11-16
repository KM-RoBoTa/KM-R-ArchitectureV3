//! The reload swap, the parameter names and the globals, without a run.

use super::*;
use crate::EachTick;

/// Writes `body` to a fresh script for test `name`; returns its folder and file name.
fn script(name: &str, body: &str) -> (String, String) {
    let dir = std::env::temp_dir().join(format!("kmr_api_lua_{name}_{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir is writable");
    std::fs::write(dir.join("s.lua"), body).expect("temp dir is writable");
    (dir.to_string_lossy().into_owned(), "s.lua".to_owned())
}

/// Calls the runtime's current function; returns `ctx.v`.
fn run_once(rt: &Runtime) -> Option<i64> {
    let call = rt.call.as_ref()?;
    call.function.call::<()>(()).ok()?;
    rt.vm.objects.ctx.get("v").ok()
}

/// Rewrites the script, then polls and reloads as the watcher and the loop would.
fn edit(rt: &mut Runtime, dir: &str, body: &str) {
    std::fs::write(Path::new(dir).join("s.lua"), body).expect("temp dir is writable");
    let mut seen = Seen {
        modified: None,
        source: String::new(),
    };
    rt.vm.watch.0.poll(&mut seen);
    rt.reload();
}

/// Loads `body` as test `name`'s script on `EachTick`.
fn load(name: &str, body: &str, n: usize) -> (Runtime, String) {
    let (dir, file) = script(name, body);
    let rt = Script::__load(&dir, &file, None, n)
        .expect("loads")
        .runtime::<EachTick>();
    (rt, dir)
}

#[test]
fn an_edit_swaps_the_function_and_keeps_ctx() {
    let (mut rt, dir) = load(
        "swap",
        "function each_tick(ctx)\n  ctx.v = (ctx.v or 0) + 1\nend\n",
        4,
    );
    assert_eq!(run_once(&rt), Some(1));

    edit(
        &mut rt,
        &dir,
        "each_tick = function(ctx)\n  ctx.v = ctx.v + 10\nend\n",
    );
    assert_eq!(run_once(&rt), Some(11), "the new body, the old ctx");
}

#[test]
fn a_broken_edit_keeps_the_previous_version() {
    let (mut rt, dir) = load("broken", "function each_tick(ctx) ctx.v = 1 end\n", 4);

    edit(&mut rt, &dir, "function each_tick(ctx\n");
    assert_eq!(run_once(&rt), Some(1), "a syntax error");
    edit(&mut rt, &dir, "function other(ctx) ctx.v = 2 end\n");
    assert_eq!(run_once(&rt), Some(1), "the function gone");
    edit(&mut rt, &dir, "error('at load')\n");
    assert_eq!(run_once(&rt), Some(1), "an error at load");
}

#[test]
fn an_unknown_parameter_leaves_no_call() {
    let (rt, _) = load("param", "function each_tick(state, foo) end\n", 4);
    assert!(rt.call.is_none());
    let (rt, _) = load("vararg", "function each_tick(...) end\n", 4);
    assert!(rt.call.is_none());
    let (rt, _) = load("local", "local function each_tick(ctx) end\n", 4);
    assert!(rt.call.is_none(), "a local is not the script's function");
}

#[test]
fn parameters_are_matched_by_name() {
    let (rt, _) = load(
        "names",
        "function each_tick(ctx, robot, time)\n  ctx.v = N * 10 + time:tick() + #robot:temperatures()\nend\n",
        3,
    );
    assert_eq!(run_once(&rt), Some(30));
}

#[test]
fn the_parameter_list_is_read_from_the_definition() {
    let src = "x = 1\nfunction each_tick( time ,\n state)\nend\nlocal each = 2\n";
    assert_eq!(
        param_names(src, 2, "each_tick"),
        Some(vec!["time".to_owned(), "state".to_owned()])
    );
    assert_eq!(
        param_names("move = function(ctx) end", 1, "move"),
        Some(vec!["ctx".to_owned()])
    );
    assert_eq!(param_names("function f() end", 1, "f"), Some(vec![]));
    assert_eq!(param_names("function ff(a) end", 1, "f"), None);
}

#[test]
fn the_globals_hold_n_and_the_depth() {
    let (rt, _) = load(
        "globals",
        "function each_tick(ctx)\n  ctx.v = N * 100 + HISTORY_DEPTH\nend\n",
        7,
    );
    assert_eq!(run_once(&rt), Some(7 * 100 + HISTORY_DEPTH as i64));
}

#[test]
fn a_state_error_is_recognised() {
    let (rt, _) = load(
        "state_error",
        "function each_tick(state, ctx)\n\
         local ok, e = pcall(state.q_at, state, N)\n\
         local ok2, e2 = pcall(error, 'other')\n\
         ctx.v = (not ok and StateError(e) and not StateError(e2)) and 1 or 0\n\
         end\n",
        2,
    );
    assert_eq!(run_once(&rt), Some(1));
}
