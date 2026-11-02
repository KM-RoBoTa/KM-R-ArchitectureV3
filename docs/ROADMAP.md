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
- [x] implement the runtime ctrl loop (schedule and time management included)
- [ ] Runtime, what the loop still misses:
    - spawn the threaded controllers (blocked by the hand-off question below)
    - tick rate per controller
    - real-time sleep (`clock_nanosleep(TIMER_ABSTIME)` or spin tail)
    - decide: should an emergency stop make `run()` return `Err`? Needs a new
      variant in `ApiError` and `UserError`
    - the `NoControllers` check in `Robot::run()`
    - `complete_api_example` and `controllers/src/main.rs` now run until
      Ctrl-C
- [ ] Hardware layer (SRS delivery step 1), in progress. Design in
    [decision 0002](decisions/0002-hal-contract.md) and
    `specs/001-kmr-adapters/`:
    - done: the contract, the list of members and their claims, `run_on`, the
      runtime call points, the per-field write mask, the opt-in hold
    - done in `kmr_adapters`: the Dynamixel member in position mode,
      Protocol 1 and 2, on a strict scripted backend; hardware tables checked
      at compile time with const overrides; presets with declared costs; bus
      plugins bound by label; torque off on every actuator; software zero
    - the bench run of the first motor (shutdown, emergency stop, read-only
      actuator, ten runs in a row, the missing udev rule)
    - `dxl_rs`: push the branch the adapter uses (`force_torque_off`,
      Protocol 1 register reads) and depend on it by git at a pinned rev; one
      10-byte sync read per group instead of three; fail the build instead of
      falling back to stub bindings when the C SDK is missing
    - CI: cross-build `controllers` with the `dynamixel` feature for aarch64
      and armv7
    - measure the read and write cost on the bench and pick the default read
      cadence, a placeholder of 4 ticks today
    - velocity and current modes: need the operating-mode register, which
      `dxl_rs` does not expose (SRS section 8)
    - swap `dxl_rs` for `dxl_rs_v2` behind the `Backend` trait once it has a
      bus: removes the SDK's per-tick allocations and its ~34 ms stall on a
      missing servo
    - accept or overrule the SRS deviations listed in decision 0002, and
      bring it up to date with the list of members
- [ ] Robot model (`kmr_model`), what this version leaves out:
    - several robots in one `.ron` descriptor (SRS FR-6.5); a descriptor
      with several root links is refused
    - joints driven by several actuators: `<mimic>` and multi-joint
      transmissions are refused (SRS Definitions, "Index space")
    - non-1-DOF joints, `floating` and `planar` (SRS section 8)
    - the center of mass of the robot from the link masses (SRS FR-8.3)
    - split `kmr_model` into a library and a thin macro crate, so a build
      script can read a descriptor (kmr_ffi, pending D12)
- [ ] Think carefully: threaded controller result hand-off. If a threaded
    controller's command gets written by an inline controller, a late result can
    cause unpredictable behavior (e.g. jitter in a sin wave) and the user won't
    know since the core owns the write (IoC). For now default to the `&mut T`
    passthrough: the threaded controller never writes state nor sends a command
    internally in the core — the user decides who consumes its output.
- [ ] Rename the core `State<const N>` (`state/history.rs`): it shares its
    name with `kmr_api::State`, the user-facing handle over `RobotState`, and
    one tick's sample reads as the whole state. Candidate: `Sample`, which is
    what `record_sample` already calls it.

## Open-source transition (in order)

- [x] Add a temporary MIT license (not GPL) while the final license is decided
- [ ] Decide the final license
- [x] Refactor the API boundary: drop the DTO layer (duplicated `Q`/`Qd`/`Tau`,
    `FieldConv`). `kmr_core` exposes only what is really needed and may use
    advanced Rust; `kmr_api` keeps the thin wrappers and the convenience layer,
    which must stay easy to read for users.
- [x] Only after the boundary refactor: go back to a single cargo workspace
- [x] Clippy heap disallowance in all modules EXCEPT the API
- [ ] Later: redesign the API so controllers can be written in other languages
    (Python, Lua, C/C++, …), with a focus on Python and C++. Rust stays the
    main language, always.
