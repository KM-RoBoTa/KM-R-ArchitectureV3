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

## Today

- [x] implement the schedule
- [x] implement time
- [ ] Shutdown system (ctrl c, signals, etc)
- [ ] implement the runtime ctrl loop (schedulle and time management included)
