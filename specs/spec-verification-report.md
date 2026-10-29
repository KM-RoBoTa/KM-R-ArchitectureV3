# Spec verification report (2026-10-27)

7 module specs. 220 issues flagged, 118 confirmed by adversarial judges, 10 refuted, the rest duplicates.

## Verifier verdicts

- **CROSS-MODULE COORDINATION**: Not sound yet. Names and the main dependency direction mostly agree. But four seams have no owner or have conflicting owners: how N reaches kmr_core, how hardware macros reach run(), the engine hooks kmr_api needs (envelope check, Zenoh staging, running hold, api-raised flags), and the stale kmr_adapters spec. Planning should wait until these are settled.
- **SRS FIDELITY**: Not sound yet. Most specs cite real SRS ids and mark open items properly. But spec 001 still contradicts the human's hardware decisions. N cannot reach the core the way SRS FR-6.1 says, and the specs answer that in different ways. Several specs also quietly decide open points (prismatic units, Kp/Kd, envelope action).
- **FEASIBILITY AND SRS SOUNDNESS**: Not yet sound. Most specs are buildable, but four points block planning: where N comes from, which adapter type run() uses, the frozen example that cannot actually run, and a marker rule that panics on paths every run takes. Several SRS lines also contradict each other (one atomic store, the private Host seam, State re-exported unchanged, Context T: Default, the per-actuator write mask, f32-only FFI, mTLS plus counting, replay from a 4-slot ring).
- **PLAN SOUNDNESS AND DELIVERY ORDER**: The order cannot be built as written. Step 1 cannot move a motor without step-3 engine work, kmr_api macros and a settled N and hardware-selection mechanism, and the specs give conflicting answers on both of those. Several success criteria rely on bench measurements that the bench spec does not provide, or that cannot exist before step 4.

## Impossible as written (10)

- **high** · FR-6.1 · kmr_core: The user crate's build.rs cannot set N, because kmr_core is compiled first and holds N as a constant.
- **high** · FR-6.3/FR-6.4 + decision (Bus::register::by_id!) · kmr_api: Nothing specifies how hardware macros written in main.rs reach run(). run() is compiled first and cannot name them. Bus::register::by_id! only works if Bus is a module, and separate macro calls cannot check for duplicate ids at compile time.
- **high** · FR-13.3 'default hold' vs section 7 Zenoh 'no hold' and FR-4.1 · kmr_api: FR-13.3's default 'hold' on a running loop has no HAL call. Hal::hold is shutdown-only. In velocity or current mode, an actuator keeps moving if no new command arrives.
- **high** · FR-6.1 + section 4 Assumptions (N compile-time) · kmr_adapters: A preset or macro in main.rs cannot set N, and making N a const generic hits a stable Rust limit.
- **medium** · FR-13.3 'sender timestamp ... older than K ticks' · kmr_api: FR-13.3 compares a remote sender's timestamp to local ticks. That needs synced clocks, which the SRS does not provide, and K is never defined.
- **medium** · FR-13.2 'dropped and counted' · kmr_api: If Zenoh's access control drops an unauthorized Desired, kmr never sees it, so it cannot count it as FR-13.2 asks.
- **medium** · FR-10.3 / FR-1.2 · kmr_ffi, kmr_core: Rust cannot fail the build when an inline controller calls Python or Lua: user code can call pyo3 or mlua directly.
- **medium** · FR-4.2 / FR-5.6 (decision 0002: every exit ends with torque_off) · kmr_core, kmr_adapters, kmr_ffi: A drop guard only cuts torque on a panic that unwinds. An abort or a segfault leaves the motors powered.
- **low** · FR-10.3 / FR-1.2 · kmr_ffi: Python and Lua may only run from threaded controllers, and those are entirely undecided, so this half of FR-10 can't be built in this version.
- **low** · NFR-8 a, FR-3.1 · bench: With a 10 µs dt floor, the bench can only report the tick-rate ceiling as 'at least 100 kHz'.

## SRS contradicts itself (20)

- **high** · Section 3 State / section 8 'kmr_api::State DTO removed' vs NFR-4, section 7 Controller, NFR-3 · kmr_api: The SRS says kmr_core::State is re-exported unchanged, and also that kmr_api wraps it with helper methods. Both cannot be true without moving helpers into the core or changing the example's imports.
- **medium** · FR-4.4 · kmr_core: FR-4.4 says 'one atomic store', but the first-reason-wins rule of FR-4.3 needs a compare-and-swap, and the code already uses one.
- **medium** · Section 4 Constraints / NFR-7 · kmr_core: Host is a private trait, so a no_std target cannot implement it, and with kmr_api std-only a no_std build has no user API.
- **medium** · FR-3.4 / FR-4.1 · kmr_core: The SRS and the HAL contract disagree on whether the shutdown hold replaces the last tick's write or follows it, and holding prev(0) after the write can undo it.
- **medium** · Definitions 'Context' · kmr_core: The SRS says a context is a T: Default built at run(), but the example hands in an already-initialized value and the code takes T by value.
- **medium** · NFR-1 / Definitions 'Context' · kmr_core: The frozen example locks a Mutex in an EachTick controller, which the SRS forbids on the control path.
- **medium** · FR-8.4 · kmr_core: FR-8.4 orders the frozen example fixed, which clashes with the compatibility decision. The ROADMAP part of this claim is a misread.
- **medium** · SRS FR-6.1 vs FR-6.2, Definitions Group · kmr_model: FR-6.1 says groups come from the descriptor, but FR-6.2 and the Group definition say they never do.
- **medium** · SRS FR-6.1 vs Definitions Actuator / Index space · kmr_model: FR-6.1 implies N equals the non-fixed joints, but the definitions allow passive joints and a many-to-many actuator mapping, so N cannot come from the joint count.
- **medium** · SRS NFR-10, section 8 module list, Definitions Engine · kmr_model: NFR-10 says kmr_core depends on its surrounding crates, but kmr_adapters depends on kmr_core, not the other way round.
- **medium** · SRS section 4, section 7, FR-6.1, module Covers · kmr_model: Three places own descriptor reading: kmr_api ('config loading', Config row), the user's build.rs, and kmr_model. FR-6.3 is also claimed by both kmr_api and kmr_model.
- **medium** · SRS FR-8.3, section 2, section 8 · kmr_model: Center-of-mass work is 'roadmap until kmr_model exists', but step 2 creates kmr_model. No step schedules CoM, and the SRS never says whether CoM depends on pose.
- **medium** · SRS FR-8.1 vs section 2 · kmr_model: Scope says kmr must know the mass of every payload, light ones included, but FR-8.1 lets only medium and heavy payloads register. The SRS also sets no capacity for the heap-free payload list.
- **medium** · FR-6.3 · kmr_api: The SRS only allows binding a port by serial id, but the decided by_dev!("/dev/ttyUSB1") binds by device path. The SRS needs a line allowing it.
- **medium** · Decision group names in Bus::register vs FR-2.4, FR-6.2, Definitions Group · kmr_api: It is unclear what the group names in by_id!(serial, "arm") are. The SRS says groups are never registered, yet the macro registers a name.
- **medium** · Section 3 'API' / section 4 constraints vs kmr_ffi module · kmr_ffi: kmr_ffi breaks two SRS rules: kmr_api is the only crate users import, and only kmr_api may use the heap, yet a Python bridge needs it.
- **medium** · NFR-8 (black-box) vs NFR-8 d, FR-5.3 · bench: A black-box bench cannot measure exact engine overhead per tick, or the gain of the pre-sort.
- **low** · Section 7 Zenoh 'key layout per instance and group' · kmr_api: The Zenoh key layout is 'per group', but groups are never registered, so the gateway has no group list to lay keys out by.
- **low** · Section 4 Constraints; Definitions Context; FR-7.3 · controllers: The SRS allows Arc<Mutex> in user code, but the root clippy.toml flags Arc in controllers; today that is only a warning.
- **low** · Section 2 scope / section 7 FFI row · kmr_ffi: The FFI language list differs: scope and FR-10.1 say Python, C/C++ and Lua, but the section 7 row says only Python and C++.

## SRS wrong or missing (40)

- **high** · SRS FR-6.1, section 7 Config row · kmr_model: The SRS keeps hardware in files (a URDF actuators side file, a .ron with ids and buses, a port-binding file), but the human decided on presets plus one-line macros in main.rs and no config files.
- **medium** · FR-5.9 · kmr_core: FR-5.9 lets user code reach the HAL directly, which clashes with the decision that the API never leaks hardware types.
- **medium** · Section 7 User API row · kmr_core: Section 7 has the builder take HAL adapters and port bindings, but the decision says hardware is declared by macros in main.rs, never through the builder.
- **medium** · Definitions 'Write mask' · kmr_core: The SRS defines one written mask per actuator, but safety needs one per field, as hal.rs already does.
- **medium** · FR-2.5 · kmr_core: The SRS never says when Desired written in PreInit, Init or PostInit gets applied, and the frozen example writes it in Init.
- **medium** · FR-13.4 · kmr_core: The FR-13.4 fault list has no 'rejected' flag, and the SRS does not say how kmr_api raises the remote-timeout and unauthorized flags in the engine.
- **medium** · FR-2.1 / Definitions 'Sample' · kmr_core: The SRS gives one age per History sample, but read cadence is per actuator, so one slot can mix fresh and old values. Age must be per actuator.
- **medium** · SRS section 4 Constraints, constitution II, root clippy.toml · kmr_model: Parsing the descriptor at build time needs heap, but the SRS and constitution allow heap only in kmr_api. kmr_model's build-time code needs a written exemption.
- **medium** · FR-7.1 / section 4 Constraints / section 8 threaded hand-off · kmr_api: The SRS wants the Zenoh translation heap-free and lock-free inside the tick, but Zenoh runs on its own threads, so it needs the same cross-thread hand-off the SRS leaves open.
- **medium** · NFR-2 / HAL 'every exit ends with torque_off' (decision 0002) · kmr_api: The SRS never says what happens when a user controller panics with torque on.
- **medium** · FR-5.10 / FR-4.2 vs FR-10.3 · kmr_ffi: A segfault in user C code kills the process with torque still on. The FR-5.10 pre-flight check covers only adapter SDKs.
- **medium** · NFR-8 a-c · bench: NFR-8 gives no run length, sample count or hold window, so p99, max and 'held without overrun' cannot be compared.
- **medium** · NFR-1 · bench: The reference bench names only the host. With no actuator chain, USB adapter or baud rate, the I/O figures FR-5.3 sends to the bench cannot be reproduced.
- **medium** · NFR-1, NFR-8 · bench: The bench kernel is a version range on a rolling distro, and the RT config, governor and core isolation are not set, so runs cannot be reproduced.
- **medium** · FR-5.3 vs the read-cadence decision · kmr_adapters: The SRS says the adapter declares the read cadence, but the decision says there is no default. Who sets it is unclear.
- **medium** · Section 8 open question 'What hold means per actuator mode' · kmr_adapters: The SRS still lists hold-per-mode as open and never says how long the hold lasts before torque off.
- **medium** · FR-2.2 · kmr_adapters: The SRS never gives units for Q, Qd and Tau. The decision is rad, rad/s and N·m.
- **medium** · NFR-1, FR-5.10 · kmr_adapters: The SRS has no written exemption for vendor SDK call sites. It exists only in the adapter spec.
- **low** · NFR-12 · kmr_core: A 4-slot History ring cannot keep 'every incident'; the record has to come from the Zenoh feed, and the NFR-8 citation is wrong.
- **low** · Section 4 Product perspective · kmr_core: Section 4 gives kmr_api 'config loading', which no longer fits the no-config-files decision.
- **low** · Section 7 HAL row · kmr_core: The SRS HAL row says 'torque on/off per actuator', but hal.rs switches torque for all actuators at once and takes the per-actuator choice from adapter config.
- **low** · FR-5.3 · kmr_core: FR-5.3 says Protocol 1 needs one round trip per actuator, but that is only true for reads: Protocol 1 has a sync write.
- **low** · NFR-10 · kmr_core: NFR-10 says 'sparse data in hash maps', but clippy bans HashMap outside kmr_api; only fixed-capacity, heap-free maps fit.
- **low** · SRS NFR-10 + FR-2.2 · kmr_model: If kmr_core depends on kmr_model, kmr_model cannot use kmr_core's typed Q/Qd/Tau. The SRS does not say how typed limits reach users.
- **low** · SRS Definitions Joint, FR-2.2 · kmr_model: Prismatic joints are a primary form, but the SRS never says what unit Q carries for a linear actuator.
- **low** · SRS FR-4.5 · kmr_model: Joint limits must be mapped onto actuators, but the SRS defines no transmission model, so only plain 1:1 actuator-joint pairs can be mapped.
- **low** · FR-10.4 · controllers: FR-10.4 never says what the ROS2 baseline is, so the onboarding metric can't be repeated the same way twice.
- **low** · FR-10.1 / FR-10.3 / section 7 FFI row · kmr_ffi: A C ABI does not let a Rust controller run Python or Lua. That needs an embedded interpreter, and the SRS never says so.
- **low** · FR-10.3 · kmr_ffi: FR-10.3 says inline C/C++ 'sees the same History/Desired', but Desired is write-only, and shared memory would bypass the write mask.
- **low** · FR-10 / FR-13.4 / FR-13.5 · kmr_ffi: The SRS never says what happens to a NaN or inf written into Desired, and no FR-13.4 flag covers it.
- **low** · NFR-8 b, NFR-1 · bench: NFR-8 asks for jitter percentiles but never says what jitter is.
- **low** · NFR-1 · bench: "CPU rev 1.4 or 1.5" is the Pi 4 board revision, not a CPU revision, and it does not pin the SoC stepping.
- **low** · NFR-1 · bench: The Pi 4 cannot measure its own supply voltage or the room temperature, and the SRS names no instruments.
- **low** · NFR-8 c · bench: The stress-ng load for NFR-8 (c) is not defined, so loaded runs cannot be compared between people.
- **low** · NFR-11 · bench: "Keep up with the fastest bus or sensor I/O" has no number to pass or fail against.
- **low** · NFR-12 · bench: NFR-12 cites NFR-8 (the benchmark) for replay. The right reference is FR-9.2.
- **low** · NFR-8 e · bench: "Stack high-water mark" does not say which thread to measure.
- **low** · Section 4 Constraints · bench: The SRS heap rules name only kmr_core and kmr_api. Other crates such as bench, kmr_adapters and kmr_model are not mentioned.
- **low** · FR-1.6 / FR-1.7 · kmr_core: Turning on the roadmap 'heap' feature anywhere in a build would switch every robot in it to dynamic mode. It should only add .before/.after.
- **low** · Section 8 delivery order gate · kmr_model, kmr_api, kmr_ffi: The 'moves a motor' gate has no clear meaning for steps that move no motor, such as descriptor parsing or Zenoh.

## Modules not coordinated (28)

- **high** · FR-6.1, FR-6.5, Definitions Index space, decision 0001 · kmr_core, kmr_model, kmr_adapters, controllers, kmr_ffi: The specs give incompatible routes for N, and who owns the typed limits is unclear (a possible core/model cycle).
- **high** · FR-6.3, FR-6.4, decision 'Bus::register' macros · kmr_api, kmr_adapters, kmr_core, controllers, bench: The specs give three different answers for how the hardware lines reach run() and for what a build with no hardware line does.
- **high** · FR-5.1, FR-5.8, FR-6.3, decision 'hardware by top-level macros in main.rs' · kmr_adapters, kmr_api, controllers: The adapters spec still puts all hardware config in the adapter layer and a per-machine binding file. kmr_api and controllers put it in main.rs macros, as the human decided.
- **medium** · FR-13.5 · kmr_core: FR-13.5 envelopes are enforced by the engine before every write, but only kmr_api owns FR-13.5, so nobody owns the engine hook.
- **medium** · FR-3.3 · kmr_core: The SRS wants Time readers (tick, overrun_count, ...) in the API, but every Time method is pub(crate) and tick and overrun_count do not exist.
- **medium** · FR-13.4 · kmr_core, kmr_api, kmr_ffi, kmr_adapters: The specs list different fault flags and different setters. No spec says how kmr_api or kmr_ffi may raise a flag.
- **medium** · FR-5.3, decision 'read cadence has no default' · kmr_adapters, kmr_core, kmr_api, bench: Nobody owns the read cadence when the user sets none. The fix-plan adds a default, which breaks the 'no default' decision.
- **medium** · FR-10.1, FR-10.2, section 3 API · kmr_ffi, kmr_api, kmr_core: kmr_ffi needs a feature re-export and handle bridges from kmr_api. The kmr_api spec offers neither, and it says the FFI types come from kmr_core.
- **medium** · Module controllers, decision example compatibility · kmr_api, controllers: kmr_api freezes controllers/src/main.rs, while the controllers spec rewrites that file.
- **medium** · FR-10.4, FR-6.1 · kmr_api, controllers, kmr_model: The '2 files' onboarding target ignores the robot descriptor and the config line that kmr_model makes mandatory.
- **medium** · decision presets and Bus::register macros, FR-2.4 · kmr_api, kmr_adapters, controllers: The specs disagree on how a preset is picked and overridden, and on whether the names in the macro are labels or bus names.
- **medium** · FR-11.3 · kmr_core, kmr_model: No module owns FR-11.3, so no spec requires the HAL, descriptor parser and schedule to stay behind traits for future plugins.
- **medium** · NFR-8 (black-box), NFR-1 · bench, kmr_api, kmr_adapters, kmr_ffi, kmr_core, kmr_model: Other specs expect bench measurements that the black-box bench spec does not provide.
- **medium** · FR-5.3, decision: read cadence has no default · controllers, kmr_adapters: Nobody sets a read cadence for the reference robot. There is no default and presets set none, so run() would reject it on hardware.
- **low** · FR-3.1 · kmr_core: In the SRS, set_dt(0) is a run() error, but the code and AGENTS.md treat a zero dt as unset and fall back to 1 ms.
- **low** · NFR-4 / FR-6.4 · kmr_core: NFR-4 wants the empty-controller check as a compile-time trait bound, but AGENTS.md still defends the runtime check, and no check is implemented.
- **low** · Section 4 Product perspective (ARCHITECTURE.md) · kmr_core: ARCHITECTURE.md says N is the sum of joints, but the SRS says it is the sum of actuators.
- **low** · FR-2.2 (decision: Q::from_deg) · controllers: Q is defined in kmr_core, so a plain Q::from_deg can only live there. In kmr_api it has to be a trait the user imports.
- **low** · FR-10.2 / FR-2.2 vs kmr_core code · kmr_ffi: The core has From<f32> for Q/Qd/Tau but not the reverse that FR-2.2 asks for, and no repr(transparent), so kmr_ffi has to copy.
- **low** · FR-2.1 · kmr_core, kmr_api, kmr_ffi: Core gives each actuator its own sample age. kmr_api and kmr_ffi say one age per sample.
- **low** · FR-2.2, NFR-4 · kmr_core, kmr_api, kmr_adapters, controllers: Q::from_deg is allowed in both kmr_core (FR-009) and kmr_api (FR-015). kmr_api's claim that the trait comes in with the example's import line is also wrong.
- **low** · FR-2.2, Definitions Joint · kmr_core, kmr_api, kmr_adapters, kmr_model: The prismatic unit line differs across specs. Core leaves out N for force, and kmr_model still lists the question as open (Q3).
- **low** · NFR-2, FR-2.3 · kmr_core, kmr_api: Reading History before the first sample is a typed error in kmr_core but None in kmr_api. Neither spec says the Option is a deliberate wrapper.
- **low** · FR-8.4, FR-6.4 · kmr_model, kmr_api, kmr_adapters: kmr_model expects things from kmr_api and the hardware side that no requirement provides: the name lookup, a payload mount-link check, and a hardware-count-equals-N check.
- **low** · decision hardware macros · bench, kmr_api, kmr_adapters: The bench, kmr_api and the adapters plan still pick hardware with a 'dynamixel' Cargo feature, while the macro decision and the fix-plan remove that feature.
- **low** · FR-5.3, FR-5.8, FR-6.4 · kmr_core, kmr_api, kmr_adapters: Adapters FR-001 lists 'torque on/off per actuator' as a contract operation, but hal.rs switches every actuator in one call. The part about core's 'provides' list does not hold.
- **low** · FR-5.5, NFR-8 · bench, controllers, kmr_api, kmr_model: N is one kmr_core const per build, but the bench, the preset robot and the N = 4 example may each need a different N. No spec says how they coexist.
- **low** · FR-1.2, section 8 threaded controllers · kmr_core, kmr_ffi, kmr_api: The core completion plan lists spawning threaded controllers as work, while every spec says they are never spawned in this version.

## Spec vs SRS/decisions (10)

- **high** · FR-6.3, FR-5.3, section 7 User API (vs human decisions) · kmr_adapters: Spec 001 (adapters) and its fix-plan still contradict the hardware decisions. Tasks were already run against that stale spec.
- **medium** · Section 5 marker rule + FR-5.2 out-of-place check · kmr_adapters: An unimplemented! on the PostInit out-of-place check would fail every run, and the spec and contract disagree on it.
- **low** · FR-2.2, section 8 'Kp, Kd join the state fields' · kmr_core, kmr_api: The kmr_core spec defers Kp and Kd 'until the first MIT adapter' in an assumption, but the SRS lists them as decided state fields.
- **low** · FR-13.5, FR-4.5, FR-13.4 · kmr_api: kmr_api makes up an envelope breach policy borrowed from the FR-4.5 joint-limit marker. It also raises an 'envelope fault' flag that neither the SRS nor its own FR-024 lists.
- **low** · FR-2.5, FR-5.2 · kmr_core: The kmr_core question about Desired written in startup phases should quote FR-5.2's 'no tick-0 seed' rule.
- **low** · FR-13.4 · kmr_api: No kmr_api requirement publishes fault flags over Zenoh, though FR-13.4 asks for it.
- **low** · FR-5.1, section 8 · kmr_api: kmr_api FR-026 cites a 2026-10-26 decision that said the opposite, and FR-027 cites FR-5.1 for presets, which FR-5.1 does not cover.
- **low** · NFR-10, section 7 HAL · kmr_adapters: The adapters spec has no NFR-10 requirement, and its FR-001 says torque on/off per actuator while hal.rs switches all actuators at once.
- **low** · NFR-1, NFR-11 · bench: Bench FR-013 says the bench never gates a build, then asks whether it should. FR-001 quietly turns 'CPU rev' into 'board revision'.
- **low** · FR-6.3 (decision) · controllers: Controllers FR-019 requires a preset hardware line, then asks whether the example has a hardware line at all.

## Delivery order (3)

- **medium** · Section 8 delivery order, steps 1 and 3 · kmr_core, kmr_adapters, kmr_api: Step 1 must move a motor, but moving one needs the write mask and tick-order work that the SRS puts in step 3.
- **medium** · NFR-1, NFR-8, FR-3.3, delivery steps 1, 2, 4 · bench, kmr_adapters, kmr_model, kmr_core: Steps 1 and 2 have success criteria that need the bench, but the bench has no delivery step and depends on step 4.
- **low** · Definitions 'Operating mode' + section 8 mode override · kmr_adapters: dxl_rs cannot set velocity or current mode, so only the position hold can be built and tested now.

## Other (7)

- **medium** · FR-4.5; module controllers · controllers: The reference controller nudges joint 0 forward with no bound, so on real hardware it drifts until it hits a mechanical stop.
- **medium** · FR-5.3 / FR-5.10 / NFR-11 · kmr_adapters, bench: On read ticks, three blocking sync reads over USB serial will likely push I/O past 1 ms. SC-003 should treat write ticks and read ticks apart.
- **low** · Section 4 / NFR-7 reference MCU · kmr_core: The no_std MCU target is still open, and the choice matters: a Cortex-M0 part (some STM32) has no compare-and-swap for the signal bus.
- **low** · SRS FR-6.5 · kmr_model: 'Each robot is a contiguous index range' holds only if the descriptor lists each robot's joints in one block, and the SRS does not require or check that.
- **low** · NFR-12 / FR-13.2 key provisioning vs decision 'no config files' · kmr_api: TLS keys have to be files on each machine; confirm that the 'no config files' decision does not cover them.
- **low** · FR-10.3 · kmr_ffi: The SRS bans Python because of locks and allocation, but allows C/C++ on the control path without saying who makes sure that code is lock-free and heap-free.
- **low** · FR-10.2 / FR-6.1 · kmr_ffi: The SRS doesn't say how C code learns N and HISTORY_DEPTH, or how a header built for another N is caught.

## Open questions per module (from the spec agents)

### kmr_core (`specs/002-kmr-core`)
1. FR-013 (SRS FR-2.5): what happens to Desired written in PreInit, Init or PostInit? The frozen example writes it in Init (initialization -> set_all_q). Init runs before torque_on and before the health checks.
   - A: Carry it to the first tick's bulked write (Desired is cleared only after an apply)
   - B: Write it once at the end of PostInit, after torque_on and before arming the clock
   - C: Drop it and clear Desired before tick 0; only tick writes reach hardware
   - Recommended: A: it keeps 'applied once after Last, then cleared' literally true, adds no new write point, and makes the example's Init write mean what its author intended. Safety cost: a value written before the health checks is applied after them, which is the safe order.
2. FR-024 (SRS FR-4.1, FR-3.4): on the shutdown tick, is that tick's Desired written before the hold, or does the hold replace it? SRS FR-3.4 says 'finish Last, write the hold command, return'. contracts/hal.md says 'hold(prev(0)) after that tick's write'. The hold target is the last read Q, which can undo the command just written.
   - A: Hold replaces the shutdown tick's bulked write (SRS FR-3.4 literal reading)
   - B: Bulked write first, then hold to prev(0) (current contracts/hal.md)
   - C: Bulked write first, then hold to that tick's written Desired where the field was written, else prev(0)
   - Recommended: A: one command per actuator on the last tick, no write that is reverted microseconds later, and it matches the SRS text, which is the authority. contracts/hal.md then needs a one-line fix.
3. FR-049 (SRS FR-6.1, FR-6.5): N (JOINTS) is a const in kmr_core, but FR-6.1 says the user's controller crate's build.rs reads the descriptor. A downstream build.rs cannot set an upstream crate's const, so N cannot reach kmr_core as written. Which route delivers N?
   - A: kmr_core's own build.rs reads the descriptor path from an env var set in the workspace .cargo/config.toml [env], via kmr_model as a build-dependency; N stays a const (keeps decision 0001)
   - B: Make N a const generic threaded through Robot, RobotState, the HAL types and kmr_api (reverses the decision 0001 proposal)
   - C: kmr_model generates a small 'robot' crate that kmr_core depends on, swapped per robot with [patch]
   - Recommended: A: it keeps every type and the frozen example unchanged and keeps decision 0001. The cost is one env var per workspace, and the descriptor lookup moves from the user's build.rs to kmr_core's. SRS FR-6.1 needs rewording.

### kmr_model (`specs/003-kmr-model`)
1. Q1 FR-004 (SRS FR-6.1): SRS FR-6.1 says the controller crate's build.rs reads the descriptor. That script runs after kmr_core is compiled, so it cannot set kmr_core's N (JOINTS = 4, kmr_core/src/state.rs:6). Where is the descriptor read?
   - A: One line in the workspace Cargo config names the descriptor. The model's build step reads it and kmr_core's N comes from it. Decision 0001 (constants, accepted at SRS.md:272) stays. The main.rs hardware macros are checked against N at compile time.
   - B: A top-level macro in main.rs reads the descriptor and N becomes a const generic in kmr_core. Everything stays in main.rs, but decision 0001 is reopened (fix-plan decision 3 A).
   - C: Keep the hand-set N for now. The model only checks the descriptor against it, so N does not come from the descriptor yet.
   - Recommended: A
2. Q2 FR-016 (SRS FR-8.1, FR-8.3): Is center-of-mass recomputation with payloads part of this version? FR-8.3 says roadmap 'until that crate exists', and delivery step 2 creates the crate.
   - A: Roadmap. The model keeps link names, parents, masses and mass centers now, and the computation waits.
   - B: A static center of mass at run() finalization, at the zero pose.
   - C: A pose-dependent query a controller may call. The engine never calls it.
   - Recommended: A
3. Q3 FR-017 (SRS FR-2.2, Definitions Joint): Prismatic actuators move in metres, but Q is in rad by the human's units and Q::from_deg makes no sense for them. What unit does Q carry for a prismatic actuator?
   - A: Q holds the actuator's SI position: rad for revolute, m for prismatic. The model records each actuator's kind. No type change.
   - B: Prismatic joints do not build in this version (compile error) until a length type exists.
   - C: Add length types to kmr_core's sealed field set now.
   - Recommended: A

### kmr_api (`specs/004-kmr-api`)
1. FR-013 (SRS section 3 State, section 8): the SRS says kmr_core::State is re-exported by kmr_api unchanged, but NFR-4 puts all ergonomics (aliases, group(), go_home) in kmr_api, never kmr_core, and the example must stay 100% compatible with its current import line. Which is binding for the State handle?
   - A: Keep the kmr_api wrapper over the core state (today's code); read the SRS 're-exported unchanged' as applying to Q/Qd/Tau and the absence of a DTO. Amend SRS Definitions 'State' and section 8.
   - B: Re-export kmr_core::State unchanged and move aliases into an extension trait; the example's import line must change (needs an explicit decision).
   - C: Re-export kmr_core::State unchanged and put aliases/group/go_home in kmr_core; violates NFR-4.
   - Recommended: A: it is the only option that satisfies NFR-4, the orphan rule and the unchanged example at the same time.
2. FR-032 (SRS FR-6.3, FR-6.4 + decision on Bus::register macros): kmr_api::Robot::run() is compiled before the user's main.rs, so it cannot name items that top-level macros define there. How are several one-line declarations gathered for run(), and what does a hardware build with no declaration and no preset do?
   - A: Several one-line macros, gathered at link time into a static table (linker-section registry), no heap, no dyn; a hardware build with neither declaration nor preset fails to build or link.
   - B: One aggregate macro block per binary (variadic lines inside it) defining one well-known symbol; compile-time cross-checks (duplicate serial ids) become possible; missing block = build failure on hardware builds.
   - C: Several one-line macros, gathered at link time; a missing declaration falls back to NoHardware with the open-time warning (today's decision 0002 behaviour).
   - Recommended: B if the human accepts one block, else A: both keep static dispatch and make a forgotten declaration a build error rather than a silent NoHardware run; C keeps the decision-0002 risk of a robot binary that drives nothing.
3. FR-046 (SRS FR-13.3): a silent remote source triggers the user's fallback, 'default hold'. On a running loop, what does hold do? kmr_api never knows actuator modes (hardware hidden), and Desired is shoot-and-forget, so in velocity/current mode 'no new command' keeps the actuator moving.
   - A: Same per-mode hold as shutdown (position -> last read Q, velocity -> 0, current -> 0), applied by the engine/adapter each tick while the source is silent; the loop keeps running. Needs a core 'hold now' call point.
   - B: Raise shutdown! (graceful stop with the per-mode hold, then torque off).
   - C: Only raise the remote-timeout flag; no default action, the user must attach a fallback.
   - Recommended: A: matches the human's per-mode hold semantics, keeps the robot safe in velocity/current modes, and leaves escalation to the user.

### controllers (`specs/005-controllers`)
1. Which file is 'the reference example' of the SRS controllers module (SRS.md:356)? Module paths are controllers/**, but the complete tour lives in kmr_api/examples/complete_api_example.rs, which must stay 100% compatible.
   - A: Only controllers/src/main.rs. The complete tour stays kmr_api's and is unchanged. The SRS fixes asked for it (FR-8.4 comment, todo!, Mutex in EachTick) go to the human as explicit decisions.
   - B: Both. The complete tour must also meet 'no todo!, no lock, no stale comments', so it needs changes. That needs an explicit decision against the compatibility rule.
   - C: Move or copy the complete tour into controllers/ as a second binary and make the copy clean. The original stays as it is.
   - Recommended: A
2. The hardware line (Bus::register::by_id!(...)) goes in main.rs, but controllers depends on kmr_api only and the macros come with the adapters. How does main.rs get them?
   - A: kmr_api re-exports the hardware macros and presets. controllers keeps one dependency. The builder stays hardware-free and the expansion uses hidden paths.
   - B: controllers also depends on kmr_adapters. This breaks the 'kmr_api only' rule in AGENTS.md and ARCHITECTURE.md.
   - C: A small separate declaration crate re-exported by kmr_api.
   - Recommended: A
3. Does the reference example carry a hardware line?
   - A: No hardware line. It runs on NoHardware anywhere and CI can run it, but no motor moves, so it does not meet FR-10.4 ('first moving controller').
   - B: One preset line (KM-RoBoTa robot). It moves a motor. It still cross-builds without the robot (FR-5.5), and without the robot it ends with a readable bring-up error.
   - C: Two binaries: one hardware-free, one with the preset line. This costs an extra file against the FR-10.4 file count.
   - Recommended: B

### kmr_ffi (`specs/006-kmr-ffi`)
1. Q1 (scope, FR-012): should Python and Lua be part of this version? Both need an embedded interpreter that allocates. kmr_ffi falls under the root clippy heap ban, and the threaded hand-off they depend on is open.
   - A: Defer Python/Lua. This version ships C/C++ only, and the Python/Lua path stays an SRS marker until the FR-1.2 hand-off is decided.
   - B: Include Python/Lua now, with the interpreter bridge in kmr_api (heap allowed) and only threaded use.
   - C: Include Python/Lua in kmr_ffi with its own clippy override, treating the interpreter as an SDK boundary.
   - Recommended: A: C/C++ is the only path the SRS allows on the control path, and Python/Lua cannot work before the threaded hand-off exists.
2. Q2 (FR-006): SRS FR-10.2 says 'f32 always'. Can time values and bitsets cross the FFI as integers? An f32 tick index goes wrong after 2^24 ticks (about 4.6 h at 1 kHz), which is the bug SRS FR-3.3 forbids.
   - A: Integers for time and flags: 64-bit nanoseconds for durations, 64-bit tick index and overrun count, bit masks for stale and fault flags. f32 stays for state values only.
   - B: f32 only. No time and no flags over the FFI; the Rust forwarder passes whatever it needs.
   - C: f64 seconds for time, with a written justification per FR-2.2.
   - Recommended: A: exact over long runs, cheap, and consistent with FR-3.3. Read FR-10.2's 'f32 always' as applying to state values.
3. Q3 (safety, FR-017): how does foreign code ask for a shutdown or an emergency stop?
   - A: The foreign function returns a status code, and the Rust forwarder turns it into shutdown!/emergency_stop! at its call site. signal.rs is untouched.
   - B: kmr_ffi exports C functions that raise the bus directly, using a fixed per-kind SignalMeta. This needs a human change near signal.rs.
   - C: No signals from foreign code. The user's Rust forwarder decides everything.
   - Recommended: A: it keeps the lock-free signal invariant human-owned, keeps per-call-site metadata in Rust, and is still one atomic store.

### bench (`specs/007-bench`)
1. FR-014 (SRS FR-5.3, FR-5.10, section 8): does the bench include a fixed reference actuator chain for per-tick I/O and pre-sort cost? SRS NFR-1 names only the host, and specs/001-kmr-adapters SC-002/SC-003 already rely on 'the bench' for I/O.
   - A: Engine-only on NoHardware in this version. Hardware runs come later.
   - B: Engine-only first (P1). Plus one named reference chain (e.g. 4 X-series on one U2D2 at 1 Mbaud), chosen by the human and added to the NFR-1 bench definition.
   - C: Hardware-only. Every run uses the reference chain.
   - Recommended: B
2. FR-007 (SRS NFR-8 d, NFR-11): a black-box bench cannot see the engine's sleep, so it cannot measure engine overhead per tick exactly. How should overhead be measured?
   - A: Pure black-box. An upper bound from probe controllers at First start and Last end, plus the tick-rate ceiling.
   - B: The human adds an opt-in timing probe feature to kmr_core (off by default, zero cost when off). The bench reads it through kmr_api.
   - C: Measure overhead with white-box micro-benchmarks inside kmr_core and leave it out of the black-box set.
   - Recommended: A first; B only if the bound proves too loose
3. FR-013 (SRS NFR-11 'tracked', NFR-1 'figures deferred'): should the bench gate merges or releases on regressions in this version?
   - A: Record only. Reports kept per commit and compared, no gate.
   - B: After the first reference run, the human sets a baseline and a tolerance. A regression beyond it blocks a release.
   - C: Gate only compile time in CI on a dev host. Runtime figures are record only.
   - Recommended: A

### kmr_adapters (`specs/001-kmr-adapters`)
1. How do top-level macros like Bus::register::by_id!("FT8ISZ6H", "arm") in main.rs reach run() with no builder call? kmr_api's run() cannot name an item in the user's binary.
   - Link-section registry (linkme-style distributed slice): heap-free and static, but a missing declaration is found only at run()
   - Macro emits a fixed #[unsafe(no_mangle)] static that kmr_api declares extern: a missing one is a link error, but it needs unsafe
   - One wrapping macro around main that also declares the hardware
   - Recommended: Link-section registry: heap-free, no dyn, no builder call. run() returns an error when it is empty.
2. complete_api_example.rs declares no hardware. What does run() do when no Bus::register macro is present?
   - Run on NoHardware with one warning (example unchanged)
   - Compile error asking for a declaration (the example changes: needs an explicit decision)
   - Recommended: Run on NoHardware with one warning, so the example stays 100% compatible
3. Are the optional names in by_id!/by_dev! bus names from the preset, or groups?
   - Bus names from the preset or custom robot
   - Groups (conflicts with SRS FR-2.4 'never registered' and FR-5.4 'bus not tied to groups')
   - Recommended: Bus names
4. A preset with 10 actuators cannot build: N is JOINTS, a kmr_core const (4) compiled before main.rs. How does N follow the declared robot?
   - N becomes a const generic in kmr_core (human-only), Mask uses a fixed word cap
   - JOINTS set at build time from the preset (cfg/env), outside main.rs
   - Recommended: Const generic N with a fixed Mask word cap: keeps the one-file rule
5. fix-plan.md adds a const default read cadence and builder calls .robot()/.port(). Both break the decisions (no default; hardware never through the builder). Should fix-plan be rewritten to match?
   - Yes: no default cadence (rejected at run()), declarations by top-level macros
   - Keep the fix-plan as is
   - Recommended: Yes, rewrite it to match the decisions

## Refuted by the judges

- FR-6.3 says ports are bound per machine. With by_id! in main.rs, the binding is in source code, so moving to another machine means editing and rebuilding. — FR-6.3 (SRS.md:166) only asks that ports be bound by serial id and resolved at bring-up. by_id!("FT8ISZ6H") does exactly that, and resolution happens at bring-up under /dev/serial/by-id (0002 R8). The FTDI serial belongs to the adapter that travels with the robot, so source edits are needed only when the adapter is swapped, not when the host changes. Issue 75 covers the remaining tension about serial ids in source.
- Section 8 marks decision 0001 (constants, HISTORY_DEPTH as capacity plus runtime check) accepted, but lists the HISTORY_DEPTH owner as still open. The decision file itself still says 'proposed'. — SRS.md:272 accepts 0001 and explicitly says 'mark the decision file accepted', so the 'proposed' status is a pending action, not an SRS error. Issue 77 repeats that part. Decision 0001 only says what happens 'when the builder gains history_depth(n)' (line 127), so whether the builder call returns and who sets it (SRS.md:307) really is still open.
- 'The reference example' is not named. Module paths are controllers/**, but the only full example is kmr_api/examples/complete_api_example.rs. 'Compile clean' is also undefined: zero warnings? are the section-5 unimplemented! markers allowed? — SRS line 274 says 'controllers/** (user crate, reference example)', so the reference example is controllers/src/main.rs, not the kmr_api example. 'Compile clean' is close enough to read: unimplemented! markers compile, and NFR-2 already says CI counts them. Only a small wording gap is left.
- The SRS says sample age in ticks and a per-actuator stale flag are exposed, but the State handle offers neither. A reference controller cannot tell a fresh sample from a stale one, so it cannot show safe stale handling. — FR-2.1 and FR-2.6 are clear. The getters missing from kmr_api/src/state.rs are unbuilt code, not an SRS fault. specs/004-kmr-api FR-020 (SRS FR-2.1, FR-2.6) requires both the age and the per-actuator stale flag. The per-sample vs per-actuator age question is a separate issue (19/134).
- 'fixed-size arrays of f32 always' read literally forces time and counters into f32. An f32 tick index loses precision after 2^24 ticks (about 4.6 h at 1 kHz), which is exactly the v2 bug FR-3.3 bans. Bitsets (stale, fault, written masks) are not f32 either. — FR-10.1 scopes the FFI to 'the state types'. FR-10.2 allows 'plain repr(C) structs' as well as f32 arrays, and a struct can carry a u64 tick or a bitset. Reading the rule as forcing time into f32 misreads it. At most this is a one-word wording fix.
- FR-10 does not say whether foreign code may raise shutdown!/emergency_stop!. The macros mint a per-call-site static SignalMeta, which has no C equivalent. Any direct path from C touches the human-only signal.rs invariant. — Inline C runs inside a Rust controller, so the Rust side can call shutdown!/emergency_stop! when the C function returns. kmr_ffi can also export a small extern "C" wrapper whose body expands the macro. That mints its own static SignalMeta in kmr_ffi and needs no change to signal.rs. FR-4.4 already allows any depth and any thread. The claim that any C path touches the human-only invariant does not hold. The open choice is ordinary spec design, not an SRS defect.
- 'Empty controllers' cannot mean zero controllers. An empty list is rejected at run() today, and at compile time once NFR-4's trait bound lands. It must mean no-op bodies, and the measuring probe itself is not empty. — The claim that an empty list is 'rejected at run() today' is false. NoControllers is defined in error.rs but never used, and AGENTS.md says run() does not check yet. Reading 'empty controllers' as controllers with empty bodies is the obvious reading, and the bench spec already uses it (FR-004). It needs no SRS change.
- Compile time is to be tracked, but section 8 says the bench hardware is 'always the RP4 setup'. Compile time on the Raspberry Pi is not useful, and the SRS names no host for it. — NFR-11 (SRS.md:234) asks to track compile time. Section 8 (SRS.md:256) fixes the bench hardware as the RP4, so together they mean compile time is measured on the RP4. Compiling natively on a Pi 4 works and stays consistent over time. Saying it is "not useful" is an opinion, not a conflict in the text. At most it is a preference about which host to use.
- Layout choices must be 'benchmarked, not guessed', but no module owns those benchmarks. bench covers NFR-1, NFR-8 and NFR-11 only, and kmr_core (human-only) covers NFR-9. — SRS.md:332 lists NFR-9 under the kmr_core module. kmr_core being human-only does not leave the requirement without an owner. It means the human writes the layout benchmarks there. Nothing requires the bench module to own them.
- 'Every registration yields a label' conflicts with the chained builder (add_controller returns the robot). If the label is the fn itself, it cannot tell apart duplicate registrations (FR-1.3) and cannot name closures. — SRS.md:119 says 'yields a controller label usable later as the C in .before(C)'. 'Yields' does not mean the builder must return a value. Each fn item is already its own zero-sized type, so it can serve as the label, as in Bevy. A label that covers every registration of the same fn is coherent and fits FR-1.3. Dynamic ordering is roadmap only (SRS.md:118). At most, closures need an explicit label later. That is a roadmap note, not a contradiction.
