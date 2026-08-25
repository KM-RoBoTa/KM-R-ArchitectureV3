# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`kmr` is a Rust framework for writing real-time (target ~1kHz) robot controllers.
Users register plain functions against schedules; the engine owns the control
loop, state, timing, and hardware. The mental model is borrowed from Bevy's ECS
(functions on schedules) but **deliberately stops short of a query system** — a
robot has one fixed-shape state, not an open world of entities. Read
`README.md` (design principles) and `docs/ROADMAP.md` (architecture note) before
large changes.

## Repo layout — two separate workspaces

The git root is the **parent** of this directory. It holds two sibling crates,
each its own Cargo workspace:

- `kmr_v3/` (this dir) — the workspace whose only member is `src/kmr_core`, the
  **private, closed-source engine**. All the real IP lives here.
- `kmr_api/` — a standalone workspace: the **public, minimal vendor API**. It
  depends on `kmr_core` by path *for dev only*; production builds swap to an
  rlib-only manifest so core source never ships. Do not add core internals to a
  public signature.

`kmr_core` must never appear in a public signature. `kmr_api` re-exposes
capabilities as newtypes (e.g. its own `Q`/`Qd`/`Tau` converted at the boundary
via a sealed `FieldConv`), it never forwards raw core handles.

## Commands

```sh
# from kmr_v3/ (this dir)
cargo build
cargo test                       # runs kmr_core unit tests
cargo test -p kmr_core bus       # single test by name substring (e.g. bus_semantics)
cargo clippy                     # clippy.toml bans heap types — see below

# from the git root (../), proves users cannot import kmr_core:
./scripts/test_kmr_core_isolation.sh
```

Rust edition 2024.

## Hard constraints

- **No heap allocation on the control path.** `clippy.toml` sets
  `disallowed-types` = `Box`, `Vec`, `String`, `Rc`, `Arc`, `HashMap`. Use fixed
  arrays and `'static`s instead. `kmr_core/src/lib.rs` also
  `#![forbid(clippy::disallowed_types)]` and `#![forbid(clippy::unwrap_used)]`.
- **Everything is `pub(crate)` by default.** The crate is highly private; only
  what `kmr_api` must name is `pub`. When adding a type, default to private and
  widen only if `kmr_api` needs it.
- **State fields are sealed and never publicly writable.** See below.

## Architecture

### Static dispatch via HList (frunk), no `dyn`

Controllers are stored in a heterogeneous list (`frunk`'s `HCons`/`HNil`),
monomorphized — zero `dyn`, zero fat pointers. `Robot<L>` (`robot.rs`) *has* the
list as a field; each `.add_controller*` prepends one node, growing the type by
one `HCons` layer. The node types are `Inline` / `InlineWith` / `Threaded`
(`controllers.rs`) — opaque containers holding `(schedule, [ctx,] fn)`. Fields
are `pub(crate)`, constructors `#[doc(hidden)]`: a user can hold an `HCons` but
can neither forge a node nor read one out. This is the core's protected IP.

`Robot::run()` uses a **runtime** `L::LEN == 0` check (→ `ApiError::NoControllers`)
rather than a compile-time bound, because a trait-bound gate leaks internal type
paths (`frunk_core::...`, core file layout) into `E0277` diagnostics at the user
call site.

### Schedules — a pre-sorted fixed pipeline (`schedule.rs`)

The `schedules!` macro generates, from one ordered `field => Label` list: the
zero-sized label structs (`PreInit, Init, PostInit, First, EachTick, Last`),
their sealed `Schedule` impls, a `Schedules<..>` struct with one bucket HList per
phase, and one `Insert<Label, Node>` impl per phase. Routing happens **once**,
when a controller is added — the label is the compile-time key that picks the one
bucket to grow. Per tick the runtime walks each bucket once in field-declaration
order: zero phase compares, zero filtering. `PreInit/Init/PostInit` are startup
(walked once); `First/EachTick/Last` run every tick. `Phase` is introspection
only, never touched for dispatch.

**These phase names are engineer-chosen, not Bevy's.** Do not reintroduce Bevy
names (`Update`, `Startup`, `RunFixedMainLoop`, …). The schedule is the source of
truth.

### Runtime (`runtime.rs`) — largely TODO

`runtime()` runs `pre_init → init → post_init`, then loops
`first → each_tick → state_transition → last`. Most bodies are stubs. `pre_init`
already installs the ctrl-c handler that fires `shutdown!`. The open work
(`ROADMAP.md`): wire `config` + the controller HList through the runtime by
walking the list via a `Tick`-style impl. The walk touches api types, so it is
driven from the `kmr_api` side or through a core-generic environment.

### Signal bus (`signal.rs`) — the only channel from controllers back to the loop

A process-global, **lock-free** bus (`AtomicU8` level + `AtomicPtr<SignalMeta>`).
Raised with the exported `shutdown!` / `emergency_stop!` macros, callable from
any depth (nested helper, closure, other thread, raw signal handler). Macros —
not free fns — because each expansion mints a per-call-site `static SignalMeta`
(`file!`/`line!` folded in only under `debug_assertions`), so nothing allocates
and the path is async-signal-safe. `compare_exchange` is monotonic: estop never
downgrades to shutdown, first winner keeps its meta. Runtime calls
`Signal::reset()` once at entry and `Signal::drain()` after each tick (happy path
= one atomic load).

### State (`state/`) — private, sealed, typed

`RobotState<DEPTH>` holds a write-only `Desired` staging area + a read-only
`History` ring buffer (`circular-buffer`). `JOINTS`/`HISTORY_DEPTH` are fixed
consts (`state.rs`) — a build-time-from-model value is a TODO. Users never touch
raw fields (`robot.desired.q[0] = ..` is a compile error). Access is
setters/getters only (`set_at`, `set_all`, `current`, `current_at`, `prev`,
`prev_at`), all speaking typed newtypes `Q`/`Qd`/`Tau` — never bare `f32`.

**The field set is sealed** (`state/field.rs`): a private `sealed::Slot` trait
routes each field type to its storage slot; the public `StateField: sealed::Slot`
can be named as a bound by `kmr_api` but no downstream crate can implement it, so
the writable field set (`Q`, `Qd`, `Tau`) is fixed. When adding a field type you
must add both a `Slot` impl and a `StateField` impl here.

### Time (`clock.rs`)

`Time` is internal state; all methods `pub(crate)`. The only user knob is
`Robot::set_dt_ms/us`, which forwards to `set_delta_ms/us`. `last`-phase (runtime)
is meant to compute elapsed/overrun/accumulated-overrun and sleep.

## Style

Comment density: the code carries dense *why* comments on the tricky invariants
(sealing, lock-free ordering, macro rationale). Match that where the reasoning is
non-obvious; do **not** add headers that restate the code. Keep module/doc
comments minimal and accurate — trust code over headers.
