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
- [ ] Learn the basics of macros https://lukaswirth.dev/tlborm/decl-macros/macros-methodical.html
- [ ] implement the runtime ctrl loop (schedule and time management included)
- [ ] Think carefully: threaded controller result hand-off. If a threaded
    controller's command gets written by an inline controller, a late result can
    cause unpredictable behavior (e.g. jitter in a sin wave) and the user won't
    know since the core owns the write (IoC). For now default to the `&mut T`
    passthrough: the threaded controller never writes state nor sends a command
    internally in the core — the user decides who consumes its output.

## Missing (from ROS2 inspiration)

- [ ] **TF (transform tree)** — where is frame A relative to frame B at time T.
    `robot.tf().lookup(CAMERA, GRIPPER)`
- [ ] **Services** — request/response call from outside the loop.
    `Robot::new().add_service("set_mode", handler)`
- [ ] **Actions** — long goal with feedback + cancel.
    `Robot::new().add_action("move_to", handler)`
- [ ] **Pub/Sub** — first-class topics (today: hand-rolled per-example over zenoh).
    `robot.publish("joint_state", &state)` / `.subscribe("cmd", on_cmd)`
- [ ] **Launch** — start/compose multiple nodes as one system.
    `kmr launch system.toml`
- [ ] **Hardware swap** — sim/real switch (adapter exists, switch doesn't).
    `Robot::new().hardware(Sim)`
