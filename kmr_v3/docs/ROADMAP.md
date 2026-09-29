# Roadmap

## Architecture note (2026-08-03)

Not full ECS. Single robot entity, fixed joint count, no dynamic component
set — a runtime query system would just re-derive at runtime what the
compiler already knows via the `HList` controller list. Staying on 100%
static dispatch (HList, monomorphized controllers) instead of `dyn`/query.

Real reason this stays open, not YAGNI: users want to attach their own data
(e.g. for ML) and inject extra entities alongside the robot. Future work:
extend the closed, compile-time-known set (more `HList`-style structs), not
a runtime entity registry — keep the 1kHz hot path static.

## todo

- [x] implement the schedule
- [x] implement time
- [x] Shutdown system (ctrl c, signals, etc)
- [x] Fix the slop schedule system
- [x] Document what's already present. All documentation for all methods in
    core. 
- [x] Learn the basics of macros https://lukaswirth.dev/tlborm/decl-macros/macros-methodical.html
- [ ] implement the runtime ctrl loop (schedule and time management included)
- [ ] Think carefully: threaded controller result hand-off. If a threaded
    controller's command gets written by an inline controller, a late result can
    cause unpredictable behavior (e.g. jitter in a sin wave) and the user won't
    know since the core owns the write (IoC). For now default to the `&mut T`
    passthrough: the threaded controller never writes state nor sends a command
    internally in the core — the user decides who consumes its output.

## Open-source transition (in order)

- [x] Add a temporary MIT license (not GPL) while the final license is decided
- [ ] Decide the final license
- [ ] Refactor the API boundary: drop the DTO layer (duplicated `Q`/`Qd`/`Tau`,
    `FieldConv`). `kmr_core` exposes only what is really needed and may use
    advanced Rust; `kmr_api` keeps the thin wrappers and the convenience layer,
    which must stay easy to read for users.
- [ ] Only after the boundary refactor: go back to a single cargo workspace
- [ ] Clippy heap disallowance in all modules EXCEPT the API
- [ ] Later: redesign the API so controllers can be written in other languages
    (Python, Lua, C/C++, …), with a focus on Python and C++. Rust stays the
    main language, always.
