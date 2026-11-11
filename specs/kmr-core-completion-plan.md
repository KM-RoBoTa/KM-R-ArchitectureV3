# kmr_core completion plan

Oct 27, 2026. Plan only, no code. Every item is HUMAN: `kmr_core/` is human-only (`.aiignore`).
Paths are under `kmr_core/src/` unless named otherwise. Inv = touches invariant code (AGENTS.md).

## 1. Where the core stands

The builder, the six-phase HList schedule, the absolute-deadline clock and the signal bus work.
The loop runs startup, ticks, and exits on shutdown or emergency stop (SRS FR-1.4, FR-3.2, FR-4.1 to
FR-4.4). The HAL contract and `NoHardware` exist in `hal.rs`, but the runtime calls none of it.
History is never filled, Desired is never applied. `cargo test -p kmr_core`: 74 passed
(66 unit, 6 integration, 2 doctests), 3 doctests ignored, 0 failed. Workspace: all green.
`cargo clippy -p kmr_core`: 6 warnings (5 missing docs, 1 private bound).

## 2. Work items

| # | What | SRS ids | Files | Dep | Size | Inv |
|---|---|---|---|---|---|---|
| 1 | HUMAN: replace `todo!()` by `unimplemented!("SRS FR-2.7: ...")` markers | NFR-2: no panic, markers only | state/history.rs:24, state/robot_state.rs:59 | – | S | no |
| 2 | HUMAN: per-field write mask in Desired; build a `Command`; clear values and mask after apply (T008) | FR-2.5: unwritten actuator gets no command, clear after apply; Def. write mask | state/desired.rs, state/field.rs, state/robot_state.rs | – | M | yes |
| 3 | HUMAN: push one History slot per tick, a copy of prev(0); seed it by `read_all` | FR-2.3: one slot per tick; FR-2.6: failed read keeps last sample | state/history.rs, state/robot_state.rs | – | M | no |
| 4 | HUMAN: per-actuator sample age in ticks, stored with the slot, readable by controllers | FR-2.1: age in ticks exposed | state/history.rs, state/robot_state.rs | 3 | S | no |
| 5 | HUMAN: read cadence knob `set_read_every(ticks)`, forwarded as `Overrides`; no default; 0 is an error (T009) | FR-5.3: cadence per adapter, engine exposes it | config/user_config.rs, robot.rs | – | S | no |
| 6 | HUMAN: `dt` range check, unset told apart from zero | FR-3.1: outside [10 µs, 1 s] run() errors before I/O; unset → 1 ms + warning | clock.rs, runtime.rs:301, error.rs | – | S | no |
| 7 | HUMAN: `RunError<E> { Api, Hal }`, `ApiError::ZeroReadCadence`, `DtOutOfRange` (T010) | FR-6.4: bad dt or registration is a run() error | error.rs, robot.rs | 5, 6 | S | no |
| 8 | HUMAN: `run_on<A: Hal>`; `run()` = `run_on(NoHardware)`, signature kept for `kmr_api` (T011) | FR-5.1: core never imports a vendor SDK; sec. 7 User API | robot.rs | 7 | S | no |
| 9 | HUMAN: wire the call points of the contract table: `check` at finalization, `open`/`zero` at Init, `ping`..`torque_on` at PostInit, `read` before First, write after Last (T012) | FR-3.4: read → First → EachTick → Last → write → sleep; FR-5.2: engine bring-up before user controllers | runtime.rs:160, 188, 276, 327 | 2, 3, 8 | L | no |
| 10 | HUMAN: exit paths. Shutdown: tick's write, `hold`, `torque_off`, `close`. Estop: poll before the write, then `torque_off`, `close`. Bring-up error after `open`: same, then return it | FR-4.1: finish Last, hold; FR-4.2: cut torque at next phase boundary; FR-5.6: torque off, flush, close | runtime.rs:327 | 9 | M | no |
| 11 | HUMAN: drop guard after `open` (panic → `torque_off`, `close`); SIGTERM → `shutdown!` (ctrlc `termination`) | FR-4.3: platform stop maps to shutdown; FR-5.6 | runtime.rs, Cargo.toml | 10 | S | no |
| 12 | HUMAN: recording-HAL loop tests: shutdown, estop in EachTick, `ping` error, `check` error (T013) | FR-3.4, FR-4.1, FR-4.2, FR-5.2 | runtime.rs (tests) | 10 | M | no |
| 13 | HUMAN: fault bitset per History slot: adapter faults plus overrun, joint limit, sensor missing, remote timeout, unauthorized; getters by depth; one setter `kmr_api` can reach | FR-13.4: typed per-tick flags, no fault silent; NFR-12: bitset per slot | state/history.rs, state/robot_state.rs, hal.rs | 3, 9 | M | no |
| 14 | HUMAN: Time surface: `tick()`, `overrun_count()`, `last_overrun()`, read-only getters `kmr_api` can forward | FR-3.3: elapsed, dt, tick, overrun count, last and total overrun, as Duration | clock.rs | – | S | no |
| 15 | HUMAN: empty controller list rejected by type-state, not at run time | NFR-4: type-state over runtime checks; sec. 8 decided | robot.rs:215 | – | S | no |
| 16 | HUMAN: real-time sleep (`clock_nanosleep` absolute, or spin tail) | NFR-1: jitter measured on the RP4 bench | runtime.rs:48 | – | M | no |
| 17 | HUMAN: no logging on the hot path (per-tick `trace!`/`debug!`) | NFR-1: no lock, no heap on the control path | runtime.rs:224, 235, 247, 284 | – | S | no |
| 18 | HUMAN: `JOINTS` from the declared robot; drop `RobotState<DEPTH>`; depth as capacity const plus runtime check | FR-6.1: N from the descriptor; decision 0001 accepted (sec. 8) | state.rs:6, state/robot_state.rs:23, 108, hal.rs:23 | kmr_model | L | yes if N becomes generic |
| 19 | HUMAN: home state and the out-of-place check at PostInit | FR-2.7: home source open; FR-5.2: flag out-of-place actuators (Superseded 2026-11-11: SRS FR-2.7 closed home as a kmr_api preset, `kmr_api::presets::home::Home`, and FR-5.2 has no out-of-place check; core T093 removed the stubs, `StateError::Undecided` and the warning.) | state/robot_state.rs:58, runtime.rs | 9, 18 | M | no |
| 20 | HUMAN: pre-write limit stage: per-actuator policy clamp / warn / abort (`emergency_stop!` "JointLimit"); envelopes enforced before every write | FR-4.5: limits per actuator, default undecided; FR-13.5: envelopes before write | runtime.rs, config/user_config.rs | 9, 13, 18 | M | no |
| 21 | HUMAN: `Kp`, `Kd` fields; `From` both ways with `f32` | FR-2.2: Kp, Kd decided; From/Into both ways | state/field.rs, state/joint_state.rs, state/desired.rs, hal.rs | 2 | M | yes |
| 22 | HUMAN: spawn `Threaded` controllers after PostInit; `Send + 'static` bounds; a `&Time` copy, no shared clock | FR-1.2: threaded TBD, never a lock the loop takes | controllers.rs:101, robot.rs:183, runtime.rs:320 | 9 | L | yes |
| 23 | HUMAN: Sensors read each tick; sensor, payload and battery bring-up at Init | FR-5.7: Sensors read per tick; FR-5.2: Init brings up sensors, payloads, battery | sensors.rs, hal.rs, runtime.rs:160 | 9 | L | no |
| 24 | HUMAN: `no_std` opt-in: `std` feature, public Host seam, own monotonic instant, tracing no-op, ctrlc and tracing-subscriber gated, threads std-only | NFR-7: no_std without alloc, CI on one MCU; sec. 4: target implements Host | Cargo.toml, lib.rs, runtime.rs:19, 33, clock.rs:30, signal.rs:44 | 22 | L | yes (signal.rs) |
| 25 | HUMAN: dynamic scheduler gate: each registration yields a label, surface unchanged | FR-1.6: dynamic mode is roadmap; FR-1.7: label usable as `.before(C)` | robot.rs, schedule.rs, controllers.rs | – | M | yes |
| 26 | HUMAN: cache-line aligned SoA slot; `repr(transparent)` on `Q`, `Qd`, `Tau` | NFR-9: SoA, aligned; FR-10.2: plain repr(C) at the FFI | state/history.rs, state/joint_state.rs | – | S | no |
| 27 | HUMAN: direct HAL access from user code | FR-5.9: user may reach the HAL | robot.rs, controllers.rs | 8 | M | yes |

## 3. Blocked by an open decision (SRS section 8)

- #7: "Return value of run() after an emergency stop" (shape of `Ok` vs `Err`).
- #18: "`HISTORY_DEPTH` owner"; "Multi-robot descriptor (FR-6.5)"; "Model crate (FR-8.3)": `kmr_model` does not exist. Also fix-plan decision 3 (const generic N or a build-wide const).
- #19: "Home state (FR-2.7)". (Superseded 2026-11-11: SRS FR-2.7 closed home as a kmr_api preset, `kmr_api::presets::home::Home`.)
- #20: "FR-4.5 joint-limit default policy". The mechanism can land; the default path stays `unimplemented!`.
- #21: no decision; "Runtime operating-mode override" decides whether Kp/Kd are runtime writes.
- #22: "Threaded controllers as a whole (FR-1.2)" and "Threaded context".
- #23: "Sensors handle surface (FR-5.7)"; "Payload registration (FR-8.4)".
- #24: "Section 4 reference MCU target for `kmr_core` CI".
- #25: "FR-1.7 controller label: zero-sized marker or const id".
- #26: "`State<N>` rename to Sample" (naming of the slot only).
- #27: not in section 8; fix-plan decision 5. Hardware declaration by top-level macros: design still planned by the human.

## 4. Risks and impossibilities

1. **FR-6.1 is impossible as written.** A downstream `build.rs` (or a macro in `main.rs`) cannot set a const of a crate already compiled. `JOINTS` is a literal in `state.rs:6`. Ways out: a const generic N, or a build-wide value read with `option_env!` (`.cargo/config.toml`). Generic N breaks `Mask([u64; JOINTS.div_ceil(64)])` on stable (`hal.rs:23, 27`): it needs a fixed word cap.
2. **Top-level hardware macros vs static dispatch.** `kmr_api::Robot::run()` (`kmr_api/src/robot.rs:235`) cannot name a type defined in `main.rs`. Pick-up needs a link-time symbol (unsafe `extern` static) or a link-section registry. A missing declaration is then a link error, not the NFR-4 compile hint. The core is fine with `run_on<A>`; the risk is in `kmr_api`/`kmr_adapters`.
3. **Threads allocate.** `std::thread::spawn` boxes the closure. Spawning in the core breaks "kmr_core: no heap" (SRS section 4). Spawn from `kmr_api`, or write the bring-up exception down. Also: the loop mutates `robot.config.time` (`runtime.rs:264, 281`) while `ThreadFn` takes `&Time` (`controllers.rs:82`): a data race once spawned.
4. **no_std seam.** SRS section 4 says the Host trait is private and the target implements it: both cannot hold (`runtime.rs:33`). `Instant` is std (`runtime.rs:19`, `clock.rs:30`). The bus uses compare-exchange (`signal.rs:171`): no CAS on thumbv6m (STM32F0). nRF52840 (thumbv7em) has it.
5. **SRS vs AGENTS.md on the empty check.** NFR-4 wants a trait bound; AGENTS.md defends a runtime check. The code has neither (`robot.rs:215-221`; `ApiError::NoControllers`, `error.rs:52`, unused). SRS wins; AGENTS.md needs a fix.
6. **Wrong write position.** The `state_transition` TODO sits before `Last` (`runtime.rs:276`). FR-3.4 puts the write after `Last`.
7. **Zero dt is not "unset".** `Time::delta` defaults to zero (`clock.rs`), so `runtime.rs:301` treats an explicit zero as unset. FR-3.1 makes zero an error.
8. **Hot-path logging.** With `RUST_LOG` set, the level is TRACE (`runtime.rs:112`); the fmt subscriber locks stdout and allocates on every tick (NFR-1).
9. **Drop guard limit.** Under `panic = "abort"` no destructor runs: torque stays on. Keep `unwind` in release, or say so in the SRS.
10. **FR-5.9.** The controller signature has no HAL handle (`controllers.rs:54`) and `run_on` owns the adapter. Direct access needs a new handle or stays deferred.
11. **Naming clash.** Renaming the slot `State<N>` to `Sample` collides with `hal::Sample` (`hal.rs:194`).
12. **Stale fix-plan.** `fix-plan.md` puts the robot in the builder (`.robot()`, `.port()`) and adds a default cadence. Both contradict the human decisions (top-level macros, no default cadence).

## 5. Milestones

- **M1 State ready** (#1 to #7, #14, #15). Done when: tests prove the write mask, one slot per tick, the dt range and the type-state check, and no `todo!()` is left in the core.
- **M2 HAL wired** (#8 to #12). Done when: a recording HAL shows the contract order on every exit, and a motor moves on the bench (SRS delivery step 1).
- **M3 Faults and timing** (#13, #16, #17, #26). Done when: controllers read faults, sample age, tick and overrun counts, and RP4 jitter is measured (delivery steps 3, 4).
- **M4 Model-driven** (#18 to #21). Done when: N follows the declared robot and limits are enforced before every write. Needs `kmr_model` and the FR-2.7, FR-4.5 decisions.
- **M5 Gates** (#22 to #25, #27). Done when: threaded controllers spawn, labels exist, and `kmr_core` builds `no_std` on the chosen MCU in CI.
