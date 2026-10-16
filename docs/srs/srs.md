# kmr v3 SRS

Oct 16, 2026 · @David

## 1. Purpose

kmr is a Rust framework for real-time robot control at about 1 kHz. The engine owns the loop, state, timing, safety and hardware access; the user writes control math as plain functions registered on a schedule (inversion of control). It replaces ROS2 or any other robot frameworks, specifically for KM-RoBoTa robots.

It serves three user classes:

- Control engineers who write controllers
- Contributors who extend the engine and the HAL
- Integrators who connect kmr to an existing robotics environment (ROS2, other stacks, external compute)

This SRS is the system-level specification. Module-level SRS files hang below it under `## Module:` headings in the ov template.

## 2. Scope

**In scope**

- One kmr instance per host, driving the robots wired to that host through its own HAL
- Several wired robots as one `Robot`: N is the sum of their actuators, user group slices split them (FR-6.5)
- Messaging over Zenoh at the API boundary: other kmr instances, ROS2 or any stack, external compute
- Medium and heavy payloads (Boston Dynamics tiers, see Definitions): a medium payload brings its own hardware interface to the robot's ports; a heavy payload registers its own services into the robot system. Both are integrated by the user, not by kmr. For every tier, including light, kmr must know the payload's mass and mount point, because it shifts the robot's center of mass and the model must recompute it reliably
- Controllers in Rust, with quality, DX-oriented FFI for Python, C/C++ and Lua (FR-10); FFI types are trivial to expose so the Rust side is never a chore, the Rust API stays simple enough not to overwhelm users, and the whole must be simpler to use and develop in than ROS2
- A `no_std`, heapless core as an opt-in Cargo feature; the default build is std, targeted at Linux

* Two scheduler modes behind one registration API: static (no heap, declaration order) is the default and the only mode in this version; a heap-gated dynamic mode with Bevy-style ordering is roadmap, and this version's API must leave that feature gate possible without breaking changes

- Security split: communication security is kmr's responsibility; robot behavior on faults (bad controller, dead camera, collision) is the control engineer's; kmr must ship the tooling that makes the second one easy (FR-13)

**Out of scope**

- Driving a robot over the network at the tick level: the loop and the HAL run on the wired host. Network controllers are in scope with one extra step: fetch current state and send desired state over Zenoh (FR-7)
- Swarm orchestration; Zenoh topology (peer or router) is the user's choice
- A query-based ECS or runtime entity registry (the scheduler is a pseudo-ECS over static declarations; the roadmap dynamic mode orders controllers, it is still not an entity registry or query system)
- Light payloads beyond their mass and inertia

## 3. Definitions

| Term | Definition |
| --- | --- |
| Device | Any addressable unit on a bus: actuator or sensor |
| Actuator | A device that produces motion; the actuator-to-joint mapping is many-to-many and not necessarily colocated: several actuators may move one joint differently, and one actuator may drive a joint elsewhere (a prismatic rod pushing a passive revolute joint) |
| Brand / series / model | Vendor, product family, exact part (lifted from v2 terminology.md) |
| Bus | An electrical link carrying device traffic (serial, CAN, USB); declared per actuator |
| Joint | One controlled degree of freedom, revolute or prismatic (the two primary forms; non-1-DOF joints are roadmap, not a priority); model-side only, the engine has no joint index; engine arrays are indexed by actuator, `0..N` |
| Group | A user-declared const array of actuator indices (e.g. \[3, 4, 5\]), contiguous or not, duplicates allowed, never registered, never from the descriptor. A slice view over the same arrays, with the same accessor set as the whole robot. v2's descriptor-generated groups and accessor methods are gone |
| Robot | The N actuators wired to this host, regardless of how many physical robots they span |
| Tick | One iteration of the control loop, period `dt` |
| Phase | A slot in the tick where controllers run: `PreInit`, `Init`, `PostInit` once; `First`, `EachTick`, `Last` every tick |
| Schedule | The ordered list of (phase, controller) registrations; fixed at compile time in the default static mode, a heap-gated dynamic mode is roadmap |
| Controller | A plain function or closure (Fn) registered on a phase. Inline: fn(&Time, &mut State, &Sensors). Inline with context: fn(&mut T, &Time, &mut State, &Sensors). Threaded with context: fn(&mut T, &Time), TBD as a whole (FR-1.2) |
| Sample | One slot of History: all actuator values from one hardware read. One slot is pushed every tick; on a tick without a read it is a copy of the previous one with its age incremented |
| Current state | Not a type. Prose shorthand for History at depth 0, prev(0); there is no CurrentState in the architecture |
| Desired state | Write-only target applied to hardware at the end of a tick; shoot-and-forget, cleared after apply, decoupled from History |
| History depth | `HISTORY_DEPTH` past samples kept; `prev(0)` is the current sample |
| Operating mode | The actuator control mode (position, velocity, torque) set at bring-up (PostInit); runtime override is open (section 8) |
| Payload tiers | Taken from [Boston Dynamics' Spot payload docs](https://dev.bostondynamics.com/docs/payload/readme). Light: inert, attached without any port or network connection (mass only). Medium: connects to a payload port and uses some or all services the robot provides. Heavy: registers and provides standard services that integrate with other components of the robot system |
| HAL | Hardware abstraction layer: sealed contract the core calls, vendor adapters implement |
| Instance | One running kmr process or firmware image on one host |
| Overrun | A tick that finishes after its absolute deadline `origin + k*dt` |

Lifecycle moments and naming, used consistently from here on:

| Term | Definition |
| --- | --- |
| Compile time | `cargo build`: descriptor parsed, N and groups fixed, schedule resolved. Errors here are compile errors. |
| run() finalization | There is no build(): run() finalizes the builder, then runs the loop. Finalization happens inside run() before any hardware I/O: `dt` checked, contexts allocated, controllers assembled. Errors here are returned by run(), never panicked. |
| Bring-up | The engine's hardware work inside the once-phases, always run before that phase's user controllers. `PreInit`: software only (tracing, Time, History, Sensors built), no hardware touched. `Init`: actuators (including brand-specific zeroing), sensors, payloads, battery initialized and registered, not yet checked or powered. `PostInit`: ping and health-check everything, read positions and flag out-of-place ones, double-check zeros, register modes, apply power on/off from defaults or user values. See FR-5.2. |
| Engine | `kmr_core` and the sub-crates it depends on (section 4). "Core" and "loop" mean the same thing. |
| API | `kmr_api`, the only crate user code imports. |
| Context | A user-defined struct (any `T: Default`; e.g. a tracker holding `[Q; N]`, camera flags, counters), sized at compile time, allocated at run() finalization. Inline-with-context: exclusive `&mut T`, one instance per registration, never shared, never locked. Threaded-with-context: the thread owns its instance and exchanges with the loop only through the hand-off of section 8, never through a lock the loop must take; between threaded controllers `Arc<Mutex<T>>` is acceptable. |
| History | The read-only ring of `HISTORY_DEPTH` samples; `prev(0)` is the current sample. The earlier prose name StateHistory is retired. Indexed by tick: one slot per tick, a copy with age+1 when no hardware read happened, so prev(k) always means k ticks ago. Reached through the State handle; Zenoh publishes its depth 0. "Current state" in prose means `prev(0)`. |
| Desired | The write-only, shoot-and-forget command buffer of FR-2.5. One canonical type name; `Desired` is the only name; the old DesiredState is not used. |
| Index space | **Actuator space.** State arrays and the HAL are indexed by actuator: `N` is the actuator count, every index and group is an actuator index, and all arrays share that length (same-length arrays, no padded aggregate struct; which is what is inferred from the index). Joints are the physical articulations; they exist in the descriptor and the model only, never as an engine index. TO RESOLVE: several actuators driving one joint is not expressible in this structure; see section 8, unimplemented! on that path. Code and docs still say "joint" in places (q\_at(i) "joint i", "A joint index exceeded"): read it as actuator until the rename pass. |
| State | kmr\_core::State, re-exported by kmr\_api unchanged (the kmr\_api::State DTO that hid types from the LSP in the closed-source era is gone). The one handle a controller receives: it reads History and writes Desired; decoupled inside, one type outside. |
| Write mask | Per-actuator written-this-tick bitset, stored out-of-band (NFR-10). Set by every write into Desired, read by the engine to pre-sort the bulked write (FR-5.3), cleared after apply. |
| Fault flags | A bitset per History slot (FR-13.4), out-of-band like the write mask. Not state fields: never implemented as a new field type. |

## 4. Overall description

**Product perspective.** kmr sits between robot hardware and everything that speaks Zenoh (C4 context). Three main crates in the workspace, plus sub-crates such as kmr\_model for the robot model and kmr\_ffi for FR-10 (final list pending, section 8; v2's crate list is not a template): `kmr_core` (engine, control loop, scheduler, state; std by default, opt-in `no_std`), `kmr_api` (gateway: Zenoh, config loading, user-facing builder; `std`), and the user's controller crate (controllers/ in the workspace). kmr\_api re-exports kmr\_core types unchanged; the v2 DTO layer is gone.

**User classes.** Control and robotics engineers who know C/C++ or Python and are not software experts. Controller code must read as math on arrays. Engine contributors are Rust developers.

**Operating environment.**

- Linux on the robot's host: x86-64 dev machine, aarch64 and armv7 targets, non-RT or PREEMPT\_RT kernel
- no\_std CI target, distinct from the NFR-1 bench target (Raspberry Pi 4): one bare-metal MCU for `kmr_core` (reference target to name: nRF52840 or STM32?)
- Buses: serial, CAN, USB

**Constraints.**

- Rust 2024 edition (The LTS per-say)
- `kmr_core`: `no_std` (opt-in) or std (default); in both: no heap, no `dyn`, no locks on the control path; the platform seam is the private Host trait in runtime.rs (now, sleep, poll); std::thread, std::time, tracing and the Ctrl-C handler live behind it under the std feature; under no\_std the target implements Host, tracing is a no-op and the platform's equivalent of Ctrl-C, if any, maps to shutdown. Heap is off by default and stays off in this version; a heap feature for compatible environments is roadmap (FR-1.6) and the current design must keep that gate possible
- `kmr_api` may use `std` and heap outside the hot path; the hot paths of kmr\_api are the State handle methods and the Zenoh translation that run inside a tick, and those are heap-free like the core
- Hand-engineered core; AI assistance only under AGENTS.md and the AI-Policy repo

**Assumptions.**

- One robot descriptor per compile (URDF default, .ron recommended, JSON discouraged but supported); actuator count N is a compile-time constant; several physical robots on one host follow FR-6.5
- Bus wiring and device addresses are known at init
- Clock source is monotonic and available on every target

## 5. Functional requirements

Items marked **\[decision\]** are open. Rule for agents, valid anywhere in this document: one open question never blocks development. Mark the exact path with unimplemented!("SRS \<id>: \<question>") so it is explicit and that path alone fails, then continue; for a severe question, use compile\_error! so the path does not build, and raise it. Severity is judged by what a wrong guess would cost, as in section 8.

**FR-1 Controllers**

- FR-1.1 A controller is a plain function or a closure (any Fn, not FnMut) registered against a schedule phase; no trait to implement. Captured values are immutable config; mutable per-controller state goes through the context variants, never through captures.
- FR-1.2 Three forms: inline, inline with user context, threaded with context. Threaded controllers are TBD as a whole (section 8): the code labels them with a phase (the example passes EachTick) while this document said not phase-bound; what the label means, when they spawn (after PostInit is the working assumption), the hand-off and the &Time data race are all open. Fixed already: std-only, never a lock the loop must take, never the State handle.
- FR-1.3 The same function may be registered several times.
- FR-1.4 Static mode (default, this version): within a phase, controllers run in declaration order. Phases: `PreInit`, `Init`, `PostInit` once; `First`, `EachTick`, `Last` every tick.
- FR-1.5 Phases must not assume every controller runs every tick, so a future `every N ticks` option is not a breaking change. **Settled:** per-controller rate itself is roadmap.

* FR-1.6 Two scheduler modes behind one registration API. **Static** (default, the only mode in this version): registrations are resolved at compile time into a fixed, heap-free sequence; ordering is declaration order within a phase. **Dynamic** (roadmap, opt-in `heap` feature, compatible environments only): registrations live in a runtime graph and may be ordered Bevy-style with `.before(C)` / `.after(C)`, trading some performance for finer control.
* FR-1.7 Gate-readiness, this version: every registration yields a controller label usable later as the `C` in `.before(C)`; enabling the heap feature must not change the builder's surface, the phase contract (FR-1.4, FR-1.5) or the controller signature; the static path remains the default and is never slower because the gate exists.

**FR-2 State**

- FR-2.1 History is read-only; its depth 0 is refreshed whenever the adapter reads the hardware (FR-5.3). On a tick without a read, prev(0) keeps the last sample and its age in ticks is exposed. Reading History costs nothing and is the user's business; Desired is write-only and applied once per tick, after Last (FR-3.4).
- FR-2.2 The scalar is f32 throughout; f64 only with a written justification. Values are typed `Q`, `Qd`, `Tau`, Kp, Kd (MIT mode, e.g. CubeMars AK60) and further fields as hardware needs them: the set is non-exhaustive, extended by the engine, sealed against user extension (NFR-3); never stored or passed bare inside the Rust API, but From/Into are implemented both ways for seamless conversion (FFI, FR-10.2, is the one place bare scalars cross) with `f32`.
- FR-2.3 History of `HISTORY_DEPTH` past samples, readable by depth; `prev(0)` is the current sample. History is indexed by tick, not by hardware read: on a tick without a read the last slot is copied with its age incremented, so prev(k) always means k ticks ago.
- FR-2.4 Groups are slices: a const array of actuator indices, contiguous or not, duplicates allowed, never registered or generated, with the same accessor set as the whole robot. Bulking for sync-write (FR-5.3) comes from the write mask, not from groups, so groups cost nothing.
- FR-2.5 Desired state is shoot-and-forget: written during the tick, applied once at the end of the tick, then cleared. It is decoupled from History; what was applied is recorded there, not in Desired. An actuator not written this tick (write mask, Definitions) receives no new command. **Decided 2026-10-16.**
- FR-2.6 A failed read keeps the last good sample, never zeros, and sets a per-actuator stale flag.

* FR-2.7 Home state. The example calls `initial_state()` and `go_home(ramp)`; where home comes from (descriptor, first read, user) is open (section 8). Until decided those paths are `unimplemented!`.

**FR-3 Timing**

- FR-3.1 User sets `dt`; outside \[10 µs, 1 s\] run() returns an error before any hardware I/O (the 10 µs floor is a management requirement, feasibility unverified until the NFR-1 bench). Unset falls back to 1 ms with a warning.
- FR-3.2 Deadlines are absolute: `origin + k*dt`. After an overrun the grid is kept, missed slots are skipped, no catch-up burst.
- FR-3.3 Time surface, through &Time: elapsed() and dt() as Duration (never f32 seconds, v2 lost precision after an hour), tick() index, overrun\_count(), last\_overrun() and accumulated\_overrun() as Duration. The same values are exposed to the API.

* FR-3.4 Tick order: hardware read (when the adapter reads this tick) → `First` → `EachTick` → `Last` → bulked write → sleep to the next deadline. `Last` is the last chance to write; the write happens after it. `shutdown!` (FR-4.1) therefore means: finish `Last`, write the hold command, return.

**FR-4 Signals and safety**

- FR-4.1 `shutdown!` finishes the current tick, `Last` included, writes a final hold command and returns; what hold means per mode is defined by each actuator adapter (HAL hold operation), not by the engine.
- FR-4.2 `emergency_stop!` exits at the next phase boundary and cuts torque.
- FR-4.3 Escalation is monotonic; the first reason wins; Ctrl-C (or the platform equivalent) maps to shutdown and is never lost, unless a foreign Ctrl-C handler was installed first: then kmr warns at bring-up and the guarantee is the foreign handler's.
- FR-4.4 Both are callable from any depth and any thread, async-signal-safe: one atomic store.
- FR-4.5 Joint limits (declared per joint in the descriptor, enforced per actuator through the model's mapping): no general rule exists, so the policy is configurable per actuator, conveniently through a group slice, among clamp, warn and abort (abort = emergency\_stop! with reason JointLimit). The default is undecided: an agent reaching this path writes unimplemented!("SRS FR-4.5: joint-limit default policy") and moves on. **\[decision\]**

**FR-5 Hardware**

- FR-5.1 The core never imports a vendor SDK; adapters implement a sealed contract (adapter pattern: the core is hardware-agnostic, one adapter crate per vendor or bus). Sealed means closed to the controllers crate and open to adapter crates: the trait is public with a #\[doc(hidden)\] supertrait in kmr\_core, since v2 learned a true seal blocks cross-crate adapters.
- FR-5.2 Bring-up runs inside the once-phases, engine work first, then that phase's user controllers. PreInit: software only, no hardware touched. Init: open buses, initialize and register actuators (brand-specific zeroing where needed), sensors, payloads, battery; nothing checked or powered yet. PostInit: ping and health-check all devices, read positions and flag out-of-place ones, double-check zeros, register modes, apply power on/off from defaults or user values. No tick-0 seed: Desired is shoot-and-forget, so the first tick commands nothing unless a controller writes.
- FR-5.3 Per tick and per (bus, series): one read transaction and one bulked write transaction where the protocol allows it (Dynamixel Protocol 2 sync read/write); where it does not (Protocol 1: AX-12A, RX-28, one round trip per actuator) the adapter declares its cadence and cost. The engine pre-sorts the written actuators by (bus, series) from the write mask before handing them to the adapter; this extra CPU work is accepted because I/O is the expensive part, and the gain is measured on the NFR-1 bench. The hardware read is opt-in and its cadence is decided per adapter, because on some hardware it is slow: Dynamixel adapters do not read every tick, others may. No general rule in the engine; the adapter declares its read cadence and the engine exposes it
- FR-5.4 A bus is an electrical concept, declared per actuator, not tied to groups.
- FR-5.5 Presence is checked at bring-up, not at compile time or run() finalization; a binary cross-compiles without the robot plugged in.
- FR-5.6 Shutdown: torque off, flush, close; best effort, no retry.
- FR-5.7 Sensors are read into a `Sensors` handle each tick (surface still WIP).

* FR-5.8 Torque enable is per actuator and chosen by the user at bring-up (default on). Read-only actuators (torque off, state still read, as in a leader arm for tele-op) are a first-class case, not a patch.
* FR-5.9 The user may reach the HAL directly from user code when the API is not enough; nothing is obfuscated.
* FR-5.10 v2 bring-up lessons, binding on adapters: USB latency timer set to 1 ms (the 16 ms default cost 128 ms per tick); flush the serial bus before open and on close; register all Dynamixel ports before opening any; never sleep or retry on the hot path (retries only in bring-up and ping); arm the clock after bring-up; pre-flight check before entering any C SDK that can segfault.

**FR-6 Model and configuration**

- FR-6.1 Actuator count N and groups come from one robot descriptor at compile time: URDF by default, .ron recommended, JSON discouraged but supported. Actuator order is descriptor declaration order; fixed joints are not actuators and are excluded from N. Joints live in the descriptor and the model only. URDF cannot carry firmware id, bus, model, protocol or operating mode, so a URDF build needs an actuators side file; .ron is self-contained. The per-machine port binding (FR-6.3) is its own file. The user's controller crate's build.rs reads the descriptor.
- FR-6.2 The descriptor declares no groups and generates no accessor methods (v2's robot.arm\_1() codegen was a mistake); groups are user-side slices (FR-2.4).
- FR-6.3 Topology is portable and compile-time; physical ports are per machine, bound by serial id, and resolved at bring-up.
- FR-6.4 A bad descriptor is a compile error; a bad port binding is a bring-up error; a bad dt or registration is a run() error.

* FR-6.5 Two deployment patterns. (1) One host, several wired robots as a single `Robot`: N is the total actuator count, one state, one loop, one clock; each robot is a contiguous index range the user may name as a group slice, and may sit on its own bus (two snakes on two buses). (2) One instance per robot, instances exchange over Zenoh (FR-7). For (1), URDF is a single kinematic tree, so a multi-robot build needs either one URDF with a common root link or a `.ron` descriptor composing several URDFs; which one is open (section 8).

**FR-7 Messaging**

- FR-7.1 Zenoh lives in `kmr_api` only, as a translator: `History` out, `Desired` in.
- FR-7.2 The core has no knowledge of any middleware.
- FR-7.3 Users may bring any other middleware inside a controller as long as it yields a valid desired state.

**FR-8 Payloads**

- FR-8.1 Medium and heavy payloads register through the API; the model takes their mass and mount point and recomputes the center of mass. Tiers follow Boston Dynamics' Spot payload classification (see Definitions).
- FR-8.2 Their actuators and sensors are the user's integration unless a kmr HAL exists.

* FR-8.3 Model. No kinematics code exists yet; center-of-mass recomputation is `kmr_model`'s responsibility and stays roadmap until that crate exists (section 8).
* FR-8.4 Registration path: through the builder (`register_payload`), not inside an `Init` controller. The example's `Init`-controller path and its comment claiming `add_controller` spawns a thread are wrong and must be fixed.

**FR-9 Visualization (WIP, not developed in this version; kept here as intent)**

- FR-9.1 A separate process renders a wireframe from the model, fed over Zenoh.
- FR-9.2 It runs without hardware, replaying `History` or a recorded trace, so the core is testable headless.

**FR-10 Foreign languages**

- FR-10.1 A C ABI exposes the state types to user code in Python or C/C++, lua, called from inside a Rust controller. The builder (Robot::new().add\_controller(), run()) is always written in Rust, is kept simple and headache-free, and is never exposed over FFI, for performance. Rust stays the primary language.
- FR-10.2 Types crossing the FFI are plain `repr(C)` structs and fixed-size arrays of `f32` always, `f64` only with justification (FR-2.2), no lifetimes or generics, so exposing a new type is not a chore on the Rust side.
- FR-10.3 A Rust controller may delegate its math to C/C++ through the FFI from any form; delegation to Python or Lua is allowed only from a threaded controller, because the CPython GIL is a lock and a Lua VM allocates and NFR-1 forbids locks on the control path. Inline C/C++ sees the same History/Desired through the FFI types; threaded Python/Lua gets only what the hand-off gives it (section 8), never the State handle, while phase registration stays in Rust.
- FR-10.4 Onboarding target: a first moving controller in less time and fewer files than the equivalent ROS2 node.

**FR-11 Plugins (planned, not built in this version)**

- FR-11.1 kmr exposes a plugin interface, in the spirit of `herdr plugin add`, so users extend it without forking: a new actuator family (an adapter at the HAL boundary), a new sensor, extra parameters in the control loop, URDF options, or a new schedule.
- FR-11.2 Each extension point above is a documented interface layer with a written how-to; the no-fork path is the supported path.
- FR-11.3 This version does not ship the plugin system but must not close its doors: the HAL contract (FR-5.1), the descriptor parser (FR-6) and the schedule (FR-1.6/1.7) stay behind compile-time interfaces (traits and generics, never trait objects) a plugin could implement later.

**FR-12 CLI helper (TBD)**

- FR-12.1 A `kmr` command-line tool complements `cargo`; its scope is undecided (candidates: project scaffolding, plugin add, descriptor validation, bench runs).

**FR-13 Security and fault tooling**

- FR-13.1 Responsibility split. Robot behavior under faults is the end user's. Securing every communication path is kmr's. Providing the tools the user needs to write fallbacks and behavior control is kmr's.
- FR-13.2 Communication security (kmr). All Zenoh traffic is authenticated and encrypted (mutual TLS or QUIC with per-machine identities). Access control per key: which identities may publish `Desired`, which may read `History`. Default deny. A `Desired` message from an unknown or unauthorized identity is dropped and counted. Secrets never live in the robot descriptor or in source.
- FR-13.3 Hijack and replay protection (kmr). Every incoming `Desired` carries a sequence number and a sender timestamp; out-of-order or older than K ticks is dropped. Each remote source has a watchdog: silence beyond its timeout triggers the user's fallback (FR-13.5), default hold.
- FR-13.4 Fault signals (kmr). The engine exposes typed, per-tick fault flags to controllers and over Zenoh: stale sample per actuator (FR-2.6), overrun (FR-3.3), bus error, device offline, sensor missing, joint limit hit (FR-4.5), remote source timed out, unauthorized command received. No fault is silent.
- FR-13.5 Fallback tooling (kmr, behavior by user). A registration may attach a fallback controller or a hold policy, triggered by a fault predicate over FR-13.4 flags. The user may declare per-actuator envelopes (position, velocity, torque) the engine enforces before every write. `shutdown!` and `emergency_stop!` (FR-4) remain the escalation of last resort.
- FR-13.6 Not provided. Collision detection, perception-failure handling, recovery strategies and any behavior logic: the user writes them with FR-13.4 and FR-13.5. Local buses (serial, CAN, USB) are treated as physically trusted.

## 6. Non-functional requirements

**Design priorities.** Of the performance / velocity / adaptability triangle, velocity (speed of development) is explicitly not a priority: AI tooling absorbs it. The two key principles the NFRs below serve are:

- **Adaptability.** A non-AI developer must understand the code fast without comments and integrate a new feature quickly and without breaking anything: proper DX, ergonomic code, YAGNI. No speculative generality.
- **Performance.** Not perfect, but above the standard industry expectation, so the engine stays future-proof. Measured, not guessed (NFR-1, NFR-11).

* NFR-1 **Real time.** No heap allocation, no `dyn`, no lock on the control path; static dispatch of controllers (default static scheduler; the roadmap heap-gated scheduler may use dynamic dispatch but never locks); one atomic load per signal check. Target jitter is hardware-dependent and is specified for a reproducible reference bench: Raspberry Pi 4, CPU rev 1.4 or 1.5, Arch Linux ARM, kernel 6.14.x or later in the 6.x line (not 7.x), measured on both a non-RT and a PREEMPT\_RT kernel, with the supply voltage and ambient temperature recorded alongside each run. Exact figures are deferred until measured on that bench; v2 reached 50 to 100 µs on non-RT Linux, but on a faster machine, so that number is not the target.
* NFR-2 **Safety.** No `unwrap`, `panic` or unchecked indexing in the engine; errors are human-readable; release builds hide internal causes. One explicit exception: the unimplemented! and compile\_error! markers of section 5 are allowed, temporary and counted by CI; a marker left on the control path is a release blocker.
* NFR-3 **Encapsulation.** `pub(crate)` by default; state fields sealed against user extension, extended by the engine (FR-2.2); raw handles never forwarded.
* NFR-4 **Developer experience.** Controller code reads as math on arrays; compile errors may show internal type paths (frunk), nothing is obfuscated any more; ergonomics (aliases, wrapper methods, conveniences) live in kmr\_api, never in kmr\_core; the builder guides the user with type-state rather than runtime checks (the runtime LEN == 0 check in the code is a closed-source-era workaround, to be replaced by a trait bound); the public surface is small enough to learn in a day and is measured against ROS2 with the FR-10.4 metric (time and file count to a first moving controller).
* NFR-5 **Portability.** x86-64, aarch64, armv7 Linux; cross-build without hardware.
* NFR-6 **Maintainability.** Every design decision defended in `docs/decisions/`; core is hand-engineered.
* NFR-7 **`no_std`.** Opt-in via a Cargo feature on `kmr_core`; the default build is `std` and targets Linux. With the feature on, `kmr_core` compiles with `#![no_std]` and no `alloc`, and `std` is confined to `kmr_api`, through the Host seam (section 4, Constraints). CI builds the core with the feature for one bare-metal target on every change.
* NFR-8 **Replicability.** A black-box benchmark run on the NFR-1 reference bench, reproducible by anyone with that hardware and setup. Measurement set: (a) tick-rate ceiling: highest rate held without overrun with empty controllers; (b) at 1 kHz: jitter p50/p99/max and overrun count, non-RT vs PREEMPT\_RT; (c) same as (b) under CPU and I/O load (stress-ng); (d) engine overhead per tick, NFR-11; (e) footprint: binary size, peak RSS, stack high-water mark.
* NFR-9 **Data layout.** State is structure-of-arrays, cache-line aligned; layout choices are documented and benchmarked, not guessed.

- NFR-10 **Memory footprint.** `kmr_core` and its surrounding modules (the kmr\_\* crates kmr\_core depends on, such as kmr\_model; exact list TBD, see section 8) apply the strategies from Andrew Kelley, *A Practical Guide to Applying Data-Oriented Design*: indexes instead of pointers, booleans stored out-of-band, padding eliminated with struct-of-arrays, sparse data in hash maps, encodings instead of OOP/polymorphism. The user API is not shaped by this; only the engine side pays for it.
- NFR-11 **Throughput and build time.** The hot path must keep up with the fastest bus or sensor I/O it is wired to, so that the remaining budget of each tick goes to controller instructions, not engine overhead. Motto: the less you do, the better, except when extra CPU work buys cheaper I/O (FR-5.3 pre-sorting). Engine overhead per tick is measured on the NFR-1 bench and tracked. Compile time of the core and of a user controller crate is tracked too, since type-level builders tend to inflate it.

* NFR-12 **Security.** No network endpoint is reachable without authentication, including debug and visualization feeds. Identities and keys are per machine, provisioned outside the descriptor. The hot path never does crypto: authentication and decryption happen in `kmr_api` before a `Desired` reaches the engine. Fault flags (FR-13.4) are a bitset per slot of `History`, so every incident is recorded and replayable (NFR-8).

## 7. Interfaces

| Interface | Where | Direction | Contract |
| --- | --- | --- | --- |
| User API | `kmr_api` | user → engine | Builder: HAL adapters, per-machine port bindings, `dt`, controller registrations; `run()` finalizes (there is no `build()`) and runs the loop, returning `Result` |
| Controller | user's controller crate, through `kmr_api` (types defined in `kmr_core`, re-exported and wrapped with QoL alias and wrapper methods, so kmr\_core stays clean while kmr\_api does the heavy lifting of ergonomics; the user never touches `kmr_core`) | engine → user | `fn(&Time, &mut State, &Sensors)`; context variant fn(&mut T, &Time, &mut State, &Sensors); threaded fn(&mut T, &Time), TBD (FR-1.2); called per phase |
| HAL | `kmr_core` | engine → adapter | Trait set (sealed per FR-5.1): bus init, ping, health-check, mode config, read sample with declared cadence, bulked write from the write mask, hold, torque on/off per actuator, close |
| Zenoh | `kmr_api` | both | `History` latest sample published per tick, full ring on request; `Desired` subscribed and applied on the tick it arrives, then dropped (no hold); key layout per instance and group |
| Config | `kmr_api` | file → compile time (descriptor) / bring-up (port binding) | Robot descriptor validated at compile time; per-machine port binding by serial id resolved at bring-up |
| FFI | `kmr_ffi` | controller → foreign code | C ABI for state types, called from inside Rust controllers; Python and C++ on top; the builder is never exposed |

## 8. Decisions pending and open questions

**Decisions (proposed default in section 5/6, architect to confirm or overrule)**

- [x] FR-2.5 desired state persists across ticks: no, shoot-and-forget (decided)
- [ ] FR-4.5 joint-limit default policy: undecided, configurable per actuator, blocked with unimplemented! on that path
- [ ] FR-1.5 per-controller tick rate: roadmap, phase contract reserved (settled)
- [x] NFR-8 replicability: black-box bench, hardware always the RP4 setup of NFR-1, measurement set (a) to (e) (settled)
- [ ] Section 4 reference MCU target for `kmr_core` CI

* [x] Actuator space vs joint space: actuator space (decided 2026-10-16). The engine indexes actuators, same-length arrays; joints are model-side only. See Index space in Definitions.

- [x] Controller signature `fn(&Time, &mut State, &Sensors)`; `State` is the one handle over History and Desired (2026-10-16)
- [x] No `build()`: `run()` finalizes and runs, errors returned before any hardware I/O
- [x] Tick order read → First → EachTick → Last → bulked write → sleep (FR-3.4)
- [x] History indexed by tick; a copy with age+1 on ticks without a hardware read
- [x] Groups are user-side slices; no descriptor groups, no accessor codegen (v2 mistake)
- [x] `kmr_api::State` DTO removed; `kmr_core::{State, History, Desired}` re-exported unchanged
- [x] Type-state over runtime checks; internal type paths in compile errors accepted
- [x] `Time` exposes `Duration`, never f32 seconds (FR-3.3)
- [x] Torque enable per actuator, user's choice; direct HAL access allowed (FR-5.8, FR-5.9)
- [x] `Kp`, `Kd` join the state fields (MIT mode); the field set is non-exhaustive (FR-2.2)
- [x] Write pre-sorted by (bus, series) from the write mask for sync-write; CPU spent to buy I/O, measured (FR-5.3)
- [x] Decision 0001 (HISTORY\_DEPTH as capacity const plus runtime check) is accepted by this SRS; mark the decision file accepted

* [ ] List of `kmr_core`'s `kmr_*` dependencies (the "surrounding modules" bound by NFR-10 and NFR-11)

- [ ] FR-1.7 controller label: zero-sized marker type per registration, or a const id; and whether the static builder keeps the HList type-state from v3

**Delivery order for this version** (an agent works top to bottom; nothing below a line starts before the line above moves a motor)

1. HAL contract and one adapter (FR-5, Must)
2. Descriptor parsing: N, actuator order, limits, side files; no accessor codegen (FR-6, Must)
3. State transition: read cadence, write mask, bulked write, tick order (FR-2, FR-3, Must)
4. Signals, Time surface, fault flags (FR-4, FR-3.3, FR-13.4, Must)
5. Zenoh translation (FR-7, Should)
6. Fallback tooling and envelopes (FR-13.5, Should)
7. Communication security (FR-13.2, FR-13.3, Could)
8. FFI (FR-10, Could)

Roadmap items (FR-9, FR-11, FR-12, dynamic scheduler, per-controller rate) are not started in this version.

**Open questions**

- [ ] Threaded controllers as a whole (FR-1.2): what the phase label means, spawn point, result hand-off (channel, double buffer, atomic slot?), the &Time data race with the loop's begin\_tick/end\_tick, and the missing F: Send + 'static bound
- [ ] Return value of `run()` after an emergency stop
- [ ] `State<N>` (one sample) rename to Sample, now that State is the handle name
- [ ] Sensors handle surface (FR-5.7)
- [ ] Final license
- [ ] Final name of the product

* [ ] TO RESOLVE. Several actuators driving one joint (one lifts, one lowers the same link) cannot be expressed with the chosen data structure, two same-size actuator arrays (SoA or decoupled). No solution yet; do not redesign the arrays around it. An agent that hits it writes unimplemented!("SRS index space: multi-actuator joint") on that path and continues
* [ ] Zenoh `Desired` apply-once (FR-7.1): likely bug nest in velocity and torque modes; solution unclear, study when it bites
* [ ] What "hold" means per actuator mode (FR-4.1): depends on each actuator's HAL/SDK, decided per adapter, not here
* [ ] Multi-robot descriptor (FR-6.5): one URDF with a common root link, or a `.ron` composing several URDFs; verify what URDF allows
* [ ] Threaded context: the loop side must stay lock-free; what the threaded side gets (merged into the item above; `Arc<Mutex<T>>` between threads is fine)

- [ ] Home state (FR-2.7): source of `initial_state()` / `go_home(ramp)`: descriptor, first read, or user; the example's calls are `todo!()`
- [ ] `HISTORY_DEPTH` owner: const today, builder `history_depth()` commented out; who sets it and whether the builder call returns
- [ ] Model crate (FR-8.3): `kmr_model` does not exist; center-of-mass recomputation waits for it
- [ ] Payload registration (FR-8.4): builder `register_payload` is `todo!()`; example to fix
- [ ] Runtime operating-mode override (v2 `config()`), and verifying Dynamixel modes against firmware (dxl\_rs exposes no register access)
- [ ] Run-time cost of the FR-5.3 pre-sort: confirm on the NFR-1 bench that it pays for itself
- [ ] Module list and `Paths:` globs for the ov split, naming `state/field.rs`, `sealed.rs`, `signal.rs`, `schedule.rs`, `controllers.rs` as the no-go files of AGENTS.md

**Roadmap (not in this revision)**

- Per-controller tick rate
- Heap-gated dynamic scheduler with Bevy-style `.before` / `.after` ordering (FR-1.6)

* Visualization process (FR-9)

- Plugin system (FR-11)
- `kmr` CLI helper (FR-12)

* Non-1-DOF joints (spherical, planar, etc.)
