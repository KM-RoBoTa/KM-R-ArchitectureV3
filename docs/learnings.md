# Learnings

_One line per step ended, appended by the agent at step end. What to do differently
next time, or "nothing new"._

| date | module | step | learning |
|---|---|---|---|
| 2026-11-03 | kmr_core | 5 | `no_std` was cheap once logging went through one crate-local shim; gate std at the entry points, not item by item |
| 2026-11-03 | kmr_adapters | 5 | a dxl_rs gap (no register write, a `Vec` per sync read) surfaces only when a test counts or writes; write the scripted half and defer the SDK half at once |
| 2026-11-03 | kmr_api | 5 | nothing new |
| 2026-11-03 | kmr_model | 5 | nothing new |
| 2026-11-03 | controllers | 5 | everything left waits on the maintainer's main.rs; one deferral row covers the chain |
| 2026-11-03 | kmr_ffi | 5 | the example crates need a cross Python and arm gcc for cross builds; native gates only |
| 2026-11-03 | bench | 5 | nothing new |
