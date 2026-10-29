# Research: kmr_ffi

Phase 0 of [plan.md](plan.md). Each item: decision, rationale, alternatives. "Assumed, pending
human: Dn" marks a choice the coordination page leaves open; the "one place" line says what a
different answer changes.

## R1. The riskiest claim: `repr(C)` layout and N-sized arrays across the ABI (proved)

**Decision**: the types that cross are `#[repr(C)]` structs generic over `const N` (and
`const D` for History). They hold `[f32; N]`, a `[u32; N]` age and `u64` masks. The C side gets the
same layout from a header generated for one N.

**Proof**: scratch workspace
`/tmp/claude-1000/-home-ovsiankina-Projects-km-robota-projects-Architecture-kmr-v3/cc4ee719-f1ad-43cc-b1ab-5c5018f7d9a6/scratchpad/kmr_ffi-plan/`
(rustc 1.97.1, gcc 16.2.1, cc 1.10.0, offline). Three crates: `mffi` (types, `call`, header
generator, `abi!`), `muser` (build script, C, C++, glue stand-in, tests), `mlint` (today's `Q`).

| Check | Result |
|---|---|
| `cargo build` | clean; `extern "C"` declarations over `*const ReadView<N, HISTORY_DEPTH>` raise no `improper_ctypes` warning |
| `cargo test -p muser`, debug and release | 6 passed, 0 failed |
| `layout_matches_c_for_every_n` | Old shape (`u64` age per slot), superseded by the rerun below. Was: size, alignment and 9 offsets equal for N = 1, 3, 4, 10, 64 (D = 4); `ReadView` 296 / 456 / 520 / 1000 / 5320 bytes |
| `transparent_q_arrays_are_f32_arrays` | with `#[repr(transparent)] Q`, `[Q; 64]` has the size and alignment of `[f32; 64]`; C sums a `[Q; 4]` passed as `*const f32` |
| `mlint`: today's `pub struct Q(pub f32)` (no repr) inside a `repr(C)` view | `warning: extern block uses type Q, which is not FFI-safe ... this struct has unspecified layout` |
| `c_write_matches_rust_twin_bit_for_bit_over_1000_ticks` | C writes `q[0]`, `q[2]` = prev + 0.1; values equal bit for bit, mask `0b0101`, context counter = 1000 |
| `an_error_applies_nothing` | C writes then returns -7: `Err(ForeignError(-7))`, mask 0, buffer cleared |
| `a_cpp_exception_is_an_error_and_applies_nothing` | a C++ body that throws returns -100 through `KMR_CPP_CONTROLLER` (`noexcept` + `catch (...)`); nothing applied. The non-throwing twin applies `q[0]` |
| `status_asks_for_estop_with_exact_tick_past_2_pow_24` | tick 2^24 + 1 arrives exact as `u64`; as `f32` it reads 2^24. Status 2 maps to `EmergencyStop`, the `tau` write is applied |
| `KMR_PROTO_N=5 cargo build` and `KMR_PROTO_N=10 cargo check` | `error[E0080]: evaluation panicked: kmr_ffi: C sources were built for another N` at the `abi!` line |
| `cargo clippy --all-targets` | first run: `not_unsafe_ptr_arg_deref` on a safe `call` taking `*mut c_void`. Fixed by taking `&mut T`; then clean |

**Rerun on the current shape (2026-10-28)**: `uint32_t age[KMR_N]` in `kmr_read_view` (latest
slot only, `KMR_AGE_NONE` = `UINT32_MAX`), no age in `kmr_slot`, six per-actuator flags in
`kmr_actuator_flags` (`envelope` included). Scratch crate
`/tmp/claude-1000/-home-ovsiankina-Projects-km-robota-projects-Architecture-kmr-v3/cc4ee719-f1ad-43cc-b1ab-5c5018f7d9a6/scratchpad/kmr_ffi-plan2/`
(rustc 1.97.1, gcc 16.2.1, cc 1.10.0 from the offline cache). `build.rs` compiles the header of
[contracts/c-abi.md](contracts/c-abi.md) three times, `-DKMR_N=1`, `4`, `64`, D = 4, with
`-std=c11 -Wall -Wextra -Werror`. The Rust side is `#[repr(C)]` twins generic over N, with
`improper_ctypes` denied.

| Check | Result |
|---|---|
| `cargo test`, debug and release | 6 passed, 0 failed |
| `layout_matches_c_n1`, `_n4`, `_n64` | 41 values per N (size, alignment and every field offset of the six types) equal three ways: Rust `size_of` / `align_of` / `offset_of!`, C `sizeof` / `_Alignof` / `offsetof`, and a hand count from the C11 rules. `KMR_AGE_NONE` = `u32::MAX` |
| `c_reads_and_writes_the_rust_values_n1`, `_n4`, `_n64` | Rust fills a view; C reads by field name `samples`, `age[0]`, `age[N-1]` (equal to `KMR_AGE_NONE` in C), `slot[3].q[N-1]` and `slot[3].tau[0]` bit for bit, `flags.stale`, `flags.envelope`, `tick_flags`, a `u64` time value above 2^32. C's `kmr_set_q(w, N-1, ..)` lands in `q[N-1]` with bit N-1 alone in `written.q`; `kmr_set_q(w, N, ..)` is ignored |
| `cargo clippy --all-targets -- -D warnings` | clean |

Sizes in bytes, the same in Rust and C; every type aligns to 8:

| N | `kmr_actuator_flags` | `kmr_slot` (`flags` at) | `kmr_read_view` (`age` at, `slot` at) | `kmr_write_buf` (`written` at) | `kmr_time_view` |
|---|---|---|---|---|---|
| 1 | 48 | 72 (16) | 304 (8, 16) | 40 (16) | 48 |
| 4 | 48 | 104 (48) | 440 (8, 24) | 72 (48) | 48 |
| 64 | 48 | 824 (768) | 3560 (8, 264) | 792 (768) | 48 |

Odd N pads 4 bytes in three places, the same on both sides: before `flags` in `kmr_slot`, before
`slot` in `kmr_read_view`, before `written` in `kmr_write_buf`. Even N has no padding.

**What it means for the plan**:

- Copying into `f32` arrays works with today's core. No core change is needed for the layout.
- Zero copy of `[Q; N]` needs `#[repr(transparent)]` (core FR-016). The plan does not rely on it.
- The safe `call` takes the context as `&mut T`, never a raw pointer (clippy finding).

**Not proved**: Python and Lua bridges (`pyo3`, `mlua` are not in the offline cache). Their risk
is lower: both are mature embedding crates, and they sit off the C ABI.

## R2. Copy, not shared memory

**Decision**: the glue copies History into `ReadView` before the call, and applies `WriteBuf`
through the setters after it.

**Rationale**: History's sample type is `pub(crate)`. Pointing C at engine memory would expose
its layout and bypass the write mask and envelopes (FR-011 (SRS FR-2.5, NFR-3, FR-13.5)). The
copy is 440 bytes at N = 4 and 3560 at N = 64 (R1, rerun 2026-10-28). Its time is not guessed: SC-003 measures it.

**Alternatives**: a pointer to History (breaks the seal, couples C to the ring layout); copying
depth 0 only (breaks parity with `prev_q(k)`, FR-010 (SRS FR-2.3)).

## R3. Time and flags as integers (assumed, pending human: Q1)

**Decision**: `TimeView` holds `u64` tick and overrun count, and `u64` nanoseconds for `dt`,
elapsed, last and accumulated overrun. `last_overrun_ns == 0` means "none" (an overrun of 0 ns is
not an overrun). Ages are `u32` ticks per actuator, latest slot only, `u32::MAX` = never read
(2026-10-28 coordination; was `u64` per slot). Stale and fault flags are `u64` masks, bit i =
actuator i; per-tick flags are one `u32` mask per slot.

**Rationale**: FR-009 (SRS FR-10.2, FR-3.3, FR-13.4). An `f32` tick is wrong after 2^24 ticks
(proved, R1). SRS FR-3.3 forbids `f32` seconds. Q1 has no D id on the coordination page; the
conflict is listed there as "FR-10.2 vs FR-3.3".

**One place**: `TimeView` and `ActuatorFlags` in `kmr_ffi/src/view.rs`, plus the header
generator, which reads the same macro.

**Alternatives**: `f64` seconds (needs a written justification, loses ns past 2^53 ns, 104 days);
`f32` (rejected by SRS FR-3.3).

## R4. How C/C++ gets N (assumed, pending human: D12, option A)

**Decision**: the user's `build.rs` calls `kmr_api::ffi::build::robot("XmBench")` (or
`.model("models/arm.urdf")`) then `.compile(&["src/swing.c"])`. The helper:

1. gets N from kmr_model's library entry, and `HISTORY_DEPTH` from the core constant;
2. writes `kmr_ffi.h` and `kmr_ffi.hpp` into `OUT_DIR` and compiles the sources with `cc`;
3. writes `OUT_DIR/kmr_ffi_abi.rs` with `FFI_N` and `FFI_DEPTH`.

In `main.rs`, `kmr_api::ffi::foreign! { fn swing; }` includes that file and const-asserts
`FFI_N == N` and `FFI_DEPTH == HISTORY_DEPTH`. A mismatch fails `cargo check` (R1).

**Rationale**: C compiles before `robot!` expands, so N must reach C another way. Naming the
robot twice is caught at compile time. The C side is always compiled against the header of the
same build, so drift between C and Rust needs C compiled outside `build.rs`, which is not a
supported path.

**One place**: `kmr_ffi::build` and the `foreign!` macro. D12 B (separate crate) moves the helper
into that crate. D12 C (pointer + length) drops both and adds a runtime length to `CFn`.

**Needs**: kmr_model must expose N to a build script (it is a proc-macro crate today, so it
cannot). The coordination page already lists "split kmr_model into lib + macro crates".
**SRS conflict**: FR-6.1 says "no build.rs in the user's crate" (R14).

**Build dependency**: the user's `[build-dependencies]` names `kmr_api` with the `ffi-build`
feature, so the controller crate keeps a single kmr dependency (FR-001 (SRS FR-10.1)). Its
compile time is tracked (SRS NFR-11). If it is too slow, a direct `kmr_ffi` build dependency is
the fallback; that changes the quickstart's `Cargo.toml` only.

## R5. Foreign shutdown and emergency stop (assumed, pending human: D13, option A)

**Decision**: the foreign function returns `int32_t`: 0 continue, 1 shutdown, 2 emergency stop,
negative = error. On 0, 1 or 2 the writes are applied, then the glue calls `shutdown!` or
`emergency_stop!`. On an error nothing is applied and `call` returns `Err`.

**Rationale**: FR-023 (SRS FR-4.1 to FR-4.4). The macros already give first-reason-wins
(SRS FR-4.3). No exported Rust function, so `signal.rs` (invariant) is untouched and no Rust
panic can unwind into C. Writes before a stop match Rust, where a controller may write and then
raise.

**One place**: `Status` in `call.rs` and its `match` in `kmr_api/src/ffi.rs`.

**Alternative**: exported `kmr_shutdown()` / `kmr_emergency_stop()` that raise the bus (D13 B).
Needs a per-call-site `static SignalMeta` minted from C, which touches `signal.rs`.

## R6. Calling foreign code safely

**Decision**: `foreign!` declares the `unsafe extern "C"` block with the fixed signature, so the
user never writes it. `kmr_ffi::call` holds the one `unsafe` block. The context is `&mut T`, or
null when the controller has none. C memory safety is the user's; every array length is N,
documented.

**Rationale**: FR-022 (SRS NFR-2). No Rust callback is handed to C, so no unwinding crosses
(a panic in a Rust `extern "C" fn` aborts since Rust 1.81 anyway). C++ exceptions stop at
`KMR_CPP_CONTROLLER` (R1).

**Alternative**: `call` as an `unsafe fn` (pushes `unsafe` into every controller, against NFR-4).

## R7. One declaration per field, and a build error when missing

**Decision**: `for_each_field!` in `kmr_ffi/src/fields.rs` is the one FFI line per field. It
generates `Slot`, `WriteBuf`, `Written`, `FIELDS`, the C and C++ header lines, and the bridge
names. The build fails when the core gains a field without that line: the kmr_api glue does an
exhaustive `match` on a core `FieldId`, so a new variant is `E0004` at that `match`.

**Needs (HUMAN)**: `kmr_core` exposes a public `FieldId` enum (`Q`, `Qd`, `Tau`), or an
associated `const ID: FieldId` on `Slot`. This touches `state/field.rs` (invariant).

**One place**: the `match` in `kmr_api/src/ffi.rs`. If the human prefers a count, a
`const FIELD_COUNT` in the core and `assert!(FIELDS.len() == FIELD_COUNT)` replace it, but a
count can be forgotten too.

**Alternative**: a proc macro over the field set (more code than the field set itself).

## R8. Python and Lua bridges

**Decision**: `kmr_ffi::python::Bridge` over `pyo3` (embedded CPython) and `kmr_ffi::lua::Bridge`
over `mlua` (vendored Lua 5.4). Each is an opt-in feature. `Bridge::load(path, function)` runs in
`main.rs` and returns its error there (FR-020 (SRS FR-6.4)). The bridge is the controller's
context (`add_controller_with`).

Per call, the bridge refills persistent sequences (`q`, `qd`, `tau` per depth, `age`, `stale`,
`time`) and reads back a write object with `set_q(i, v)` etc. Python and Lua errors become
`ForeignError` with the message; nothing is applied.

**Rationale**: FR-017, FR-018, FR-019 (SRS FR-10.1, FR-10.3). A C ABI alone cannot run a script
(spec Assumptions). Vendored Lua avoids a system library; the dev machine has Lua 5.5 and LuaJIT,
which differ.

**Risks**: `pyo3` must support the host CPython (3.14 on the dev machine), checked at
implementation. `mlua::Lua` is not `Send` without its `send` feature; the threaded form needs it
(FR-004 (SRS FR-1.2)). Interpreter allocation per call is the user's cost (FR-005).

**Alternatives**: a Python C extension calling the C ABI (Python as the entry point, against SRS
FR-10.1); cffi/ctypes (same).

## R9. Crate placement and dependencies

**Decision**: `kmr_ffi` depends on no kmr crate. It holds plain types, `call`, the build helper and
the bridges. `kmr_api` depends on it under the `ffi` feature and owns the glue (kmr_api FR-072).

**Rationale**: kmr_api re-exports kmr_ffi, so kmr_ffi cannot depend on kmr_api (cycle). The glue
needs `State` internals, which only kmr_api reaches. Plain types make the FFI testable with no
engine. Examples and glue tests use kmr_api as a dev-dependency (Cargo allows that cycle).

**Alternative**: kmr_ffi on kmr_core (the core's readers are `pub(crate)`; the glue would split
over two crates).

## R10. Non-finite values

**Decision**: kmr_ffi adds no value policy. A NaN from C goes through the same setter as a NaN
from Rust (FR-013 (SRS FR-2.2, NFR-2)). The SRS is silent; the core decides.

## R11. Stack, not heap, on the C/C++ path

**Decision**: `call` builds `ReadView` and `WriteBuf` on the stack (5.4 KB at N = 64). No
allocation, no lock, one indirect call (FR-021 (SRS NFR-1)). A context-held buffer is the
fallback if the bench shows the zeroing matters.

## R12. Delivery gate (decided 2026-10-28: D1 A, option A)

Planning now; building after delivery step 7 moves a motor. Only the task order depends on it.

## R13. N cap (assumed, pending human: D4, option A)

`u64` masks give N ≤ 64, the core's cap. One place: `type Mask = u64` and the const assert in
`ReadView::default`. D4 B (more words) turns each mask into `[u64; W]` in the view and the headers.

## R14. SRS problems found

| SRS | Problem |
|---|---|
| FR-10.2 vs FR-3.3 | "f32 always" vs "never f32 seconds" (already on the coordination page). Ages and flag masks are integers too: FR-10.2 should say "state values `f32`; time, ages, flags integers" |
| FR-10.2 | "no lifetimes or generics": the Rust types are generic over N; only the C side has none. Wording should say "on the foreign side" |
| FR-6.1 vs FR-10 | "No build.rs in the user's crate" leaves no way to compile C/C++ against N. D12 A needs FR-6.1 to allow a build script for foreign sources only |
| FR-6.1 | "Names it once in main.rs": D12 A names the robot a second time in `build.rs` (checked at compile time) |
| FR-10.1 | "A C ABI exposes the state types to user code in Python ... lua": a C ABI cannot run a script; the bridges embed the interpreters. Should say so |
| FR-10.3 / §4 / Constitution II | Heap "like kmr_api": §4 and Constitution II name kmr_api only (on the coordination page under §4 heap rules) |
| §7 FFI row | Lists Python and C++ only; FR-10.1 also names C and Lua (on the coordination page) |
| FR-13.4 | The fault flag set is not final, so the read view grows with it. Safe because the header is regenerated each build, but the SRS should list the flags that cross |
| NFR-2 / Safety | A crash in foreign code leaves torque on; no actuator-side bus watchdog exists (on the coordination page) |

## R15. Constitution conflicts (recorded, not fixed)

- Principle 0 requires agent trailers; the human decided no AI or co-author trailer (AGENTS.md
  rule (b)). This plan makes no commit.
- Principle I delivery order: D1 (R12).
- Principle II heap: kmr_api only, against SRS FR-10.3 (plan Complexity Tracking).

## Coordination log

- 2026-10-28 coordination: `age` moved from every `kmr_slot` to `kmr_read_view`, latest slot
  only, as `uint32_t age[KMR_N]` with `KMR_AGE_NONE` = `UINT32_MAX` (the core gives
  `Option<u32>` per actuator, no age per slot). The layout table of R1 (sizes, 9 offsets) was
  proved on the old shape and must be rerun (plan A3). R3's "ages are `u64`" now reads `u32`.
- 2026-10-28 coordination: `envelope` added to `kmr_actuator_flags` and `for_each_flag!`, per
  actuator, matching core K4.1 and kmr_api's `Faults` view (D3 A, assumed, pending human).
- 2026-10-28 coordination: `FieldId` points at core plan K7.1; the kmr_model build-script
  entry points at kmr_model research R12 (lib + macro split, pending D12); kmr_api plans
  `ffi-python` and `ffi-lua` (its G2).
- 2026-10-28 coordination: R1 layout rerun on the current shape (`age[KMR_N]` as `uint32_t` in
  `kmr_read_view`, `KMR_AGE_NONE` = `UINT32_MAX`, six flags in `kmr_actuator_flags`), C side
  through `cc`: 6 tests pass; 41 values per N equal in Rust, C and a hand count for N = 1, 4, 64.
  `kmr_read_view` is 304 / 440 / 3560 bytes (was 296 / 520 / 5320). R2 and plan Performance
  Goals carry the new sizes. Plan A3 still reruns the probes on the real `view.rs`.
