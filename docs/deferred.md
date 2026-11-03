# Deferred

_Written by agents when a task hits a forbidden decision (see
`CLAUDE.md`), and by me at step 9. Each entry re-enters the cycle at
step 4. Three open entries on one module = `needs contract rework`.
Never delete a row; set status `done` with the date._

| date | module | step | what was needed | why forbidden | smallest unblocking change | status |
|---|---|---|---|---|---|---|
| 2026-11-03 | kmr_ffi | T020, T021 | `python.rs` and `lua.rs` implement `Foreign` and return `ForeignError`/`Status`, all defined in `call.rs` by T007 | T007 is one unit with the crate's only `unsafe` block (`unsafe extern "C" fn` type and the call); agents may not write `unsafe` beyond the counting allocator | the maintainer writes or authorizes T007 (`call.rs`); the bridges then need no `unsafe` of their own | open |
| 2026-11-03 | controllers | T016 | search 2 (`kmr_core\|kmr_model\|kmr_adapters\|dxl`) hits controllers/src/main.rs:11, a comment naming `kmr_adapters/src/presets/rx28_chain.rs` | main.rs is the maintainer's file (owner `mine`); the task says fix the hit in controllers/ | reword the RX-28 comment without the crate path, or drop it when T010 rewrites main.rs; then rerun T016 | open |
