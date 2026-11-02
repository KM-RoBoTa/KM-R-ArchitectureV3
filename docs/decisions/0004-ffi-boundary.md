# 0004 — The FFI boundary: copied views, generated headers, a returned status

Status: **proposed**, waiting for the maintainer's decision. Several points rest
on pending decisions (D1, D3, D4, D12, D13, Q1, see [Pending](#pending)).

The research behind each point is in
[`specs/006-kmr-ffi/research.md`](../../specs/006-kmr-ffi/research.md)
(R1 to R5, R11, R13, R14). Requirements: SRS FR-10, delivery step 8, Could.

## The question

A Rust controller hands one tick's math to C, C++, Python or Lua (SRS FR-10.1).
Six choices shape the boundary:

- whether foreign code sees a copy of the state or engine memory;
- how C and C++ learn N, which `robot!` fixes in `main.rs`;
- how time, ages and flags cross;
- how foreign code asks for a shutdown or an emergency stop;
- where a script bridge keeps its interpreter;
- how many actuators a view can hold.

## Current state

| Piece | State |
|---|---|
| `kmr_ffi` crate, `for_each_field!`, `for_each_flag!` | written |
| `ReadView`, `Slot`, `WriteBuf`, `Written`, `ActuatorFlags`, `TimeView`, layout tests | written |
| Header and ABI-file generator, `Build::compile` over `cc` | written; N and depth are passed in |
| `robot(..)`, `.model(..)`: N from a preset or a descriptor | not written: waits on the kmr_model library entry (kmr_model T059, D12) |
| `call`, `Status`, `ForeignError`, `foreign!` | not written: `unsafe`, human (T007, T012) |
| Glue in `kmr_api::ffi` | not written: kmr_api G1, after the core readers (core K1, K3.5, K4.1, K7.1) |
| Python and Lua bridges | not written: wait on `ForeignError` (T007) |

## Decision proposed

1. **Copy, not share.** kmr_api's glue copies every History slot, the flags and
   the per-actuator ages into a `#[repr(C)]` `ReadView`, and the clock into a
   `TimeView`. Foreign code fills a zeroed `WriteBuf`; the glue applies only the
   flagged entries, through the normal `State` setters.
2. **Headers generated per build, from the user's `build.rs`** (D12 A).
   `kmr_api::ffi::build::robot("XmBench").compile(&["src/swing.c"])` writes
   `kmr_ffi.h`, `kmr_ffi.hpp` and `kmr_ffi_abi.rs` into `OUT_DIR` and compiles
   the sources against them. `foreign! { fn swing; }` in `main.rs` const-asserts
   that the ABI file's N and depth equal `robot!`'s.
3. **Time, ages and flags as integers** (Q1). Tick and overrun count are `u64`,
   durations `u64` nanoseconds, ages `u32` ticks with `u32::MAX` for "never
   read", flags `u64` masks with bit i for actuator i. State values stay `f32`.
4. **A returned status** (D13 A). The foreign function returns 0 (continue),
   1 (shutdown), 2 (emergency stop) or a negative error. The Rust forwarder
   applies the writes, then raises `shutdown!` or `emergency_stop!`. An error
   applies nothing.
5. **The bridge is the controller's context** (SRS FR-1.1). `Bridge::load(..)`
   runs in `main.rs` and returns its error there, before `run()` (SRS FR-6.4);
   the bridge then goes into `add_controller_with`.
6. **N ≤ 64** (D4 A). One `u64` per mask; `type Mask = u64` and the const
   assert in `ReadView::default` are the one place.

## Defence

### Real-time performance

- The C/C++ path is one fill, one indirect call, one apply loop over N. No
  heap, no lock, no `dyn` in kmr's work (SRS NFR-1). The view is on the stack:
  440 bytes at N = 4, 3560 at N = 64 (R1, measured layout).
- The copy's time is not guessed: the bench measures it against an empty Rust
  controller, NFR-8 (d) (SC-003, a human run). A context-held buffer is the
  fallback if the zeroing shows.
- A slow, allocating or blocking language is allowed: overruns are counted and
  the cost is the user's (SRS FR-10.3). Python's GIL and Lua's VM allocation
  are stated in each bridge's docs.

### Quality

- **The seal holds.** History's sample type stays `pub(crate)`. Foreign code
  never writes engine memory, so the write mask and envelope checks apply to a
  foreign write exactly as to a Rust one (SRS FR-2.5, FR-13.5). A NaN from C
  gets the core's treatment, no policy of its own (SRS FR-2.2).
- **The layout is proved, not assumed.** Size, alignment and every offset of the
  six types are equal in Rust, in C and in a hand count by the C11 rules, for
  N = 1, 4, 64 (R1, rerun on `view.rs` by its tests). Odd N pads 4 bytes in
  the same three places on both sides.
- **A wrong N fails the build.** C compiles before `robot!` expands, so the
  robot is named twice; the const assert turns a mismatch into `error[E0080]`
  under `cargo check` (R1, proved in a scratch crate).
- **Exact time.** An `f32` tick is wrong after 2^24 ticks, 4.6 h at 1 kHz (R1,
  proved); SRS FR-3.3 forbids `f32` seconds. `u64` nanoseconds are exact for
  584 years.
- **No exported signal function.** The status keeps `signal.rs` (invariant)
  untouched, keeps first-reason-wins (SRS FR-4.3) through the existing macros,
  and hands no Rust callback to C, so no unwind can cross.
- **One field list.** `for_each_field!` drives the Rust types, both headers and
  both bridges. With the core's `FieldId` (core K7.1), a field the list misses
  is `E0004` in kmr_api's apply `match` (SC-004).

### Developer experience

- A C example is four files: `Cargo.toml`, `build.rs`, `main.rs`, `swing.c`
  (SRS FR-10.4). The controller crate keeps one kmr dependency, `kmr_api`, with
  the `ffi` and `ffi-build` features (SRS FR-10.1).
- C reads named arrays, `in->slot[0].q[i]`, and writes through
  `kmr_set_q(out, i, v)`, which ignores `i >= KMR_N`. C++ gets typed `kmr::Q`,
  `Qd`, `Tau` at no cost over the C types, and `KMR_CPP_CONTROLLER` turns any
  exception into `-100`.
- A script that does not load is an error in `main.rs`, before the robot
  moves, never a panic mid-run.

### Alternatives rejected

- **A pointer to History** breaks the seal and couples C to the ring layout.
  **Copying depth 0 only** breaks parity with `prev_q(k)` (SRS FR-2.3).
- **A separate header crate** (D12 B) moves the helper, not the problem.
  **Pointer plus length** (D12 C) loses the compile-time N check and puts a
  length test in every C function.
- **`f64` seconds** need a written justification and lose nanoseconds after
  2^53 ns. **`f32`** is forbidden by SRS FR-3.3.
- **Exported `kmr_shutdown()` / `kmr_emergency_stop()`** (D13 B) need a
  `static SignalMeta` minted from C, an edit to `signal.rs`.
- **A captured bridge** works in an inline controller but hides mutable state
  outside a context (SRS FR-1.1). Allowed, not advised.
- **More mask words** (D4 B) turn each mask into `[u64; W]` in the view and the
  headers, for robots the core does not target.

## SRS deviations to accept

| SRS | Deviation (R14) |
|---|---|
| FR-6.1 | "No `build.rs` in the user's crate": D12 A needs one, for foreign sources only. "Names it once in `main.rs`": the robot is named again in `build.rs`, checked at compile time |
| FR-10.2 vs FR-3.3 | "`f32` always" vs "never `f32` seconds": state values `f32`; time, ages and flags integers |
| FR-10.2 | "No lifetimes or generics": true on the foreign side; the Rust types are generic over N |
| FR-10.1 | "A C ABI exposes the state types to Python ... Lua": a C ABI cannot run a script; the bridges embed the interpreters |
| FR-10.3, §4 | Heap "like kmr_api": §4 and constitution II name kmr_api only. kmr_ffi's build helper allows heap types in its own module; the crate-wide override and the constitution amendment are the human's (T002) |
| NFR-2 | A crash or abort in foreign code leaves torque as it was; no software guard prevents it. The headers and crate docs say so, next to SRS FR-5.10 |

## Pending

- **D1** (delivery gate, A): planning before step 7, building after it. Task
  order only.
- **D3** (`envelope` per actuator, A): one line in `for_each_flag!`.
- **D4** (N ≤ 64, A): `type Mask` and the const assert in `ReadView::default`.
- **D12** (build helper in the user's `build.rs`, A): `kmr_ffi::build` and
  `foreign!`.
- **D13** (returned status, A): `Status` in `call.rs` and its `match` in
  `kmr_api/src/ffi.rs`.
- **Q1** (integer time and flags, no D id; coordination "FR-10.2 vs FR-3.3"):
  `TimeView` and `ActuatorFlags` in `view.rs`, and the header generator.
