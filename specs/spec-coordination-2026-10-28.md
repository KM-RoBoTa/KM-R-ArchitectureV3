# Spec coordination, 2026-10-28

All 7 module specs rewritten against the SRS patched 2026-10-27. Two verifiers checked them
(interfaces; feasibility and delivery order). A fixer applied every fix that needed no human
decision. This page lists what needs you.

## 1. Decisions for the human

| # | Question | Options | Recommended | Specs |
| --- | --- | --- | --- | --- |
| D1 | Delivery gate (SRS §8 "nothing below a line starts before the line above moves a motor") is circular: step 1 cannot move a motor without step 3 (write mask, tick order) and step 4 (exit paths). | A: step 1 = HAL contract + adapter + minimal state transition + exit paths; B: keep text, waive per case | **Decided 2026-10-28: A** A | all |
| D2 | Joint-limit breach with no policy (SRS FR-4.5 default undecided). Model makes a limit mandatory per actuator. | A: SRS marker on the breach path only, unwind guard cuts torque; B: `run()` refuses unless every actuator has a policy; C: pick the default now | **Decided 2026-10-28: B, for now** verifiers split: interfaces A, feasibility B (no panic after open, NFR-2) | core FR-037, api FR-019/FR-052, controllers FR-022/023 |
| D3 | Envelope breach (SRS FR-13.5) | A: clamp + new envelope flag (FR-13.4 lacks one); B: reuse FR-4.5 policy; C: drop command + flag | A | api FR-052, core FR-051 |
| D4 | N cap. Write mask on stable Rust: one `u64` → N ≤ 64. SRS silent above 64. | A: cap 64; B: fixed larger word count (e.g. 4 → 256) | A unless a robot > 64 is planned | core FR-012, design-const-generic-n.md |
| D5 | SRS §5 `unimplemented!` marker rule vs NFR-2 "never panic". Markers after `open` run with torque on (api FR-036 `go_home`, FR-065). | A: amend §5: no marker after open; use typed error, `run()` error or feature-gated `compile_error!` | A | api, core |
| D6 | Bus binding: plugin names its bus label `.bus("label")`, optional for one-bus robots, mismatch = `run()` error | confirm / change | confirm | api Q2, adapters Q1 |
| D7 | Hardware without a preset | A: `robot!` hardware argument from kmr_adapters' table types; B: `hardware!` macro; C: builder calls | A | api Q1, adapters FR-025/026 |
| D8 | Zero position (0 rad) | A: model centre; B: raw 0 (v2); C: per-actuator offset, default centre | C | adapters Q3 |
| D9 | Default read cadence | A: adapter default per brand, bench-confirmed; B: table must set it | A | adapters Q2, bench |
| D10 | Reference chain for NFR-1 | XmBench (4× XM430-W210, 1 Mbaud) + a named USB adapter, added to NFR-1 | yes | bench FR-015 |
| D11 | Pre-sort gain measurement (FR-5.3) | A: report cost only; B: opt-in core switch, compare builds (HUMAN) | B | bench FR-016, core |
| D12 | How C/C++ gets N (FR-6.1 forbids user build.rs) | A: build.rs allowed only to compile foreign code, names the robot again + check; B: separate crate; C: pointer + length | A | ffi Q3 |
| D13 | Foreign shutdown/e-stop | A: return a status, Rust forwarder calls the macro | A | ffi Q2 |
| D14 | ROS2 baseline (FR-10.4) | minimal `ros2 pkg create` node, count user-edited files | yes | controllers Q2 |
| D15 | Shipped `controllers/src/main.rs` bus line | A: none, simulates as shipped | A | controllers Q1 |
| D16 | `try_lock` in an `EachTick` controller of the validated example: a lock in a tick (NFR-1) | keep / replace with a lock-free slot | replace | api FR-069 |
| D18 | Actuator limits (voltage, temperature) configurable through the API | a plugin capability on every actuator plugin whose hardware supports it (all Dynamixel models) | **Future, decided 2026-10-29.** Not implemented; adapters spec Roadmap | adapters T070 |
| D17 | Several vendors in one robot: any number of brands (10 actuators of 10 brands must work), not just two. Decision 0002 had made it one adapter per build; `run_on<A: Hal>` took the whole hardware as one object. | A: a set of drivers in kmr_core, built like the controller list; B: a `Chain<A, B>` type in kmr_adapters, zero core edits, lifted later | **Decided 2026-10-29: a set of drivers of any length (HList), each owning an index set; placement A (core) assumed, pending human.** Design: `specs/002-kmr-core/design-hal-set.md`. Not on the first-motor path (XmBench has one member) | core hal.rs, `run_on` (HUMAN, core T069); adapters `check` (T069); api `Spec`, `robot!` hardware (T069) |

## 2. SRS problems (text to amend)

| SRS | Problem |
| --- | --- |
| §8 decision 0001 | Still says N and DEPTH are constants; FR-6.1 says const generic. Needs a superseding record. |
| FR-6.1 | Calls `N` a type alias (it is a const). "`&mut State` everywhere" holds only in the crate calling `robot!`; shared controller crates write `State<'_, N>`. "Any preset default can be overridden": scope unclear. Hardware list omits cadence, torque, zero offset. |
| NFR-10 | "kmr_model is a dependency of kmr_core": false with a const generic N. "Sparse data in hash maps" clashes with the heap ban. |
| NFR-7 / §4 | `robot!` lives in std-only kmr_api, "the only crate user code imports": a `no_std` build has no user API. Recommended: out of this version, say so. |
| §4 heap rules | Name kmr_core and kmr_api only: silent on kmr_model (build-time heap), kmr_ffi (FR-10.3 allows heap), bench. Constitution II and root `clippy.toml` need the same. |
| FR-2.2 | `Q` holds rad or mm by kind; the kind is model-side, so `From<Deg>` into a prismatic actuator is a silent error and a controller cannot tell the unit. |
| FR-10.2 vs FR-3.3 | "f32 always" vs "never f32 seconds"; an f32 tick index breaks after 2^24 ticks (~4.7 h at 1 kHz). |
| FR-13.4 | No `rejected` flag (hal.rs has one), no envelope-breach flag. |
| FR-13.5 vs FR-13.3 | Hold policy needs the running-loop hold that FR-13.3 defers. |
| FR-13.2 | "Dropped and counted": kmr never sees transport drops. |
| FR-5.9 | "Reach the HAL directly": no path named. |
| FR-6.3 | "run() simulates": undefined. Recommended: no-hardware adapter seeds nothing; History reads return "no sample". |
| FR-8.3, FR-11.3 | Need run-time/public code; a proc-macro crate exports only macros. Later: split kmr_model into lib + macro crates. |
| FR-2.1 | "Its age": per sample or per actuator. |
| NFR-12 | "Every incident replayable" cannot hold in a `HISTORY_DEPTH` ring. |
| NFR-1 | Names the host only, no actuator chain or USB adapter; "CPU rev 1.4/1.5" is the board revision. |
| NFR-8 vs FR-5.3 | Black-box bench cannot isolate the pre-sort gain. |
| NFR-11 | "Fastest bus or sensor I/O": no number. |
| Definitions "Overrun" | Possible off-by-one in `origin + k*dt` (unverified). |
| Definitions "Context" | Allows `Arc<Mutex<T>>`; root clippy bans `Arc` in controllers. |
| §3 | "Contexts allocated at finalization" vs "moved in at registration". |
| Module kmr_adapters | Covers line omits adapter duties set elsewhere (FR-2.2, FR-4.1, FR-5.2, FR-5.6, FR-2.6, FR-13.4, FR-6.3 resolution). Preset hardware half and the SDK exemption are not in the SRS. |
| FR-5.3 | Protocol 1 has a sync write; "hardware read is opt-in" unclear. |
| §7 | FFI row says "Python and C++" (FR-10 names C and Lua too); "full ring on request" vs depth-0 slot. |
| Safety | No actuator-side bus watchdog: the only guard against abort or an FFI crash. |
| FR-4.1 | Does not say what the hold is built from. Built from Desired after the clear, Dynamixel `hold` sends 0 rad to every motor. Design: latest sample with this tick's writes laid over it. |
| FR-4.3 / NFR-2 | FR-4.3 does not name SIGTERM or SIGHUP; NFR-2 should list SIGQUIT and SIGKILL with abort. |
| FR-4.2 | Silent on an emergency stop raised during the PostInit bring-up steps. |
| FR-3.3 | `last_overrun()` and the origin of `elapsed()` undefined. |
| FR-2.3 | Does not cover the starting History slot seeded by `read_all`. |
| FR-13.4 | "No fault is silent" cannot hold before core plan #13, which is off the critical path. |
| FR-5.1 | "One adapter per vendor or bus": silent on a robot mixing vendors; decision 0002 had made it one adapter per build (D17). Patch prepared 2026-10-29. |

## 3. Critical path to the first motor (task ids)

XmBench, position mode, N = 4 unchanged. `kmr_model` is **not** on it. Ids are each module's
`specs/00n-*/tasks.md` (rewritten 2026-10-28, cross-module analysis).

1. adapters T002 HUMAN (dxl_rs `force_torque_off`; T003 pushes the branch CI needs).
2. adapters T001, T004–T017 agent (strict fake, torque off everywhere; T017 after step 1).
3. adapters T018–T020, T023, T024, T026, T027–T033 agent (table and binding; startable now).
4. core T001–T014 HUMAN (T003 gates M1; T006 touches `state/field.rs`).
5. adapters T041 agent (engine tests through `run_on`).
6. controllers T001–T003 agent (`robot!(XmBench)`, `hardware` feature).
7. kmr_api T001–T005, T007–T013 agent (`run()` hands the adapter to `run_on`; Time, `Deg`).
8. controllers T006–T012 agent (bounded swing, `shutdown!` after 12 s).
9. Bench session, HUMAN: controllers T014, T015 = kmr_api T017 = adapters T045 = core T018. One
   blocker list everywhere: core T004–T013, adapters T017, T041, kmr_api T011, controllers T012.

Startable today (agent, no human decision or code): adapters T001, T004–T016, T018–T020, T023,
T024, T026, T027–T033; kmr_api T001–T003 then T005, T007–T009; controllers T001–T003, T030.

Old numbers in the tasks files ("CP n", "critical path step n"): 1 → 1, 2 → 2, 3 to 8 → 4,
9 → 7, 10 → 6 and 8, 11 → 9.

Not needed first: core plan #1, #5, #6, #7, #13, #15, #18+, all of kmr_model.

Design for step 4: `specs/002-kmr-core/design-first-motor.md`, proved on a scratch copy (219
tests pass, 0 fail; one invariant file, `state/field.rs`).

## 4. Artefacts

- Specs: `specs/00{1..7}-*/spec.md`.
- Design for core const-generic N: `specs/002-kmr-core/design-const-generic-n.md` (prototype proved; N = 3, 5, 10, 64; no_std; zero runtime cost).
- kmr_model plan: `specs/003-kmr-model/` (off the critical path) and code review `review-2026-10-28.md`.
- Stale: `specs/001-kmr-adapters/{plan,tasks,research,data-model,quickstart,fix-plan}.md`, contracts.

## 5. Changes the human design notes need (after the cross-plan fixes)

`design-first-motor.md`:

- `push_slot` fills empty slots with zeros (`unwrap_or_default`). Spec FR-042 says History reads
  return "no sample" on a no-hardware run. Needs an `ever_read` gate and `Option<u32>` age. (Pick
  one: zeros or "no sample".)
- "`Overrides::default()` until #5 adds the cadence knob" is stale: plan K3.8 keeps `Overrides`
  empty; cadence comes from the hardware table.
- The per-tick `Faults` local becomes per-slot `Faults<N>`; the envelope flag is per actuator.
- §5 `run_on(kmr_adapters::Hardware::default())` becomes the emitted `__Spec` /
  `Dynamixel::bind`.
- State the K1.6 warning text: "no bus plugin: simulating", printed at `open`.
- 2026-10-29 (D17, `design-hal-set.md`): `run_on(hal)` takes the list of drivers, not one
  adapter; `check` returns each member's claim and the core merges the `Layout`. The `Powered`
  guard covers every member: `torque_off` on all, then `close` on all, reverse order.

`design-const-generic-n.md` (§2.2, step 8):

- The alias reads `Robot<N, __Spec>` (later with `E`); it comes from kmr_api's front door, not
  the proc macro.
- controllers moves to `robot!` earlier (plan step C0).
- Kind bit sets are `[u64; MASK_WORDS]` too.

Safety fix already in the core plan (K5.2): `Limit { lower: Option<Q>, upper: Option<Q>, .. }`;
`None` = continuous joint, never clamped to 0.
