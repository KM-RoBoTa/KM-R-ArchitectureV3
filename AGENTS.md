# AI Agent Instructions

These are the common instructions for any AI agent or assistant working in this
repository — including **Claude** (Claude Code, claude.ai), **ChatGPT / Codex**,
**Cursor** and **Copilot**. Read them before making any change.

This file is the single source of truth for agent rules. `CLAUDE.md` is a
symlink to it, and `.cursor/rules/` only points here — do not duplicate rules,
extend them here. For the fuller contribution policy, see
[`CONTRIBUTING.md`](CONTRIBUTING.md): follow it wherever it goes deeper, but the
rules below always apply.

## Spec-driven development

`kmr` is built spec-first with the cycle of
[ov-agentic-workflow](https://github.com/Ovsiankina/ov-agentic-workflow): the
SRS says what, the cycle says who builds it and how, agents write most of the
code, the maintainer decides and reviews.

Read `workflows/cycle/ROADMAP.md` first: which module is at which step, its
owner and its mode. Then `workflows/cycle/PROCEDURE.md` for the step you are
on, and `workflows/cycle/QA.md` before any PR. A module's requirements are its
`## Module:` section in [`docs/srs/SRS.md`](docs/srs/SRS.md) and the ids its
`Covers:` line names; its tasks are in `docs/plans/<module>.md`.

The **oracle** is the set of `[oracle]` checks in `QA.md`: tests, clippy, pub
diff. "Green" means all of them pass.

## Agent decision policy

**Allowed, never ask:** any internal choice that keeps the oracle green.
Algorithm, private helper, error text, local naming, test data, iteration
order, private struct. Mark each one where it happens:

    // DECISION: <one line: what was chosen and why>

This is the one exception to [`CONTRIBUTING.md` rule 8](CONTRIBUTING.md#engineering-rules):
the reviewer reads the what and the why in the code.

Bookkeeping is never a question: status rows, plan ticks, doc moves, Cargo
scaffold, copying templates. Do it and mention it in one line.

**Decisions you don't take alone:**

- anything `pub`: signature, visibility, new item, re-export, trait impl on a
  pub type. In `kmr_core`, widening `pub(crate)` counts. A `pub` item that a
  plan task or `specs/<module>/contracts/` spells out is already decided:
  build it as written.
- data layout: `#[repr]`, field order, field types, enum discriminants,
  anything a `size_of` test pins
- dependencies: adding, removing, bumping, feature flags
- `unsafe`, anywhere
- the safety behaviour of the loop: torque off, shutdown, emergency stop
- any module whose ROADMAP row says owner `mine`

When a task hits one of these:

1. Write it to `docs/deferred.md`: module, step, what was needed, why it is
   not yours to decide, the smallest change that would unblock.
2. Mark the blocked test `#[ignore = "deferred"]`.
3. Continue with the rest of the task.

After 3 open deferrals on one module, stop that module, set its ROADMAP step
to `needs contract rework`, and move to the next module.

**The invariants** — the sealing in `state/field.rs` and `sealed.rs`, the
lock-free ordering in `signal.rs`, the HList/`Insert` machinery in
`schedule.rs` / `controllers.rs` — may be edited. A change there keeps the
invariant, has a test that would catch breaking it, and is named in the PR.

**Modules owned `mine`:** do not edit their files unless the task cannot be
done otherwise. If you do, list those files first in your summary, with one
line on why.

## Git & commit rules

**a) Never push to `main`.** `main` is protected and only moves through a
   reviewed and merged PR. Work happens on `dev` and on branches cut from it.

**b) Commit as the maintainer, never credit an AI.** Commits are made under
   the maintainer's own git identity. No `Co-Authored-By:` trailers and no
   "Generated with" footers naming an AI.

**c) An agent never approves or merges** a pull request. The maintainer does
   both.

## Commits, branches & pull requests

- **Commits** follow [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/).
  A commit that does a plan task puts its id first in the description, so the
  step end can tick it: `feat(core): T037 add the Kp and Kd slots`.
- **Branches** follow [Conventional Branch](https://conventional-branch.github.io/):
  one `feature/<module>` per module in the cycle (`feature/kmr-core`),
  `fix/…` for a change.
- **Merge** (step 7): a PR from the module branch into `dev`, squash-merged by
  the maintainer. `dev` reaches `main` through its own PR.
- **Explain the change** in the commit body, the PR description, or both.
- **Pull requests** use [`.github/pull_request_template.md`](.github/pull_request_template.md).

## General guidance

- Keep changes scoped to the task. Ask before anything destructive: history
  rewrites, force pushes, deleting branches or data.
- Ambiguity inside the oracle is a `DECISION`; ambiguity outside it is a
  deferral. Never guess silently.
- Match the existing style (see [Style](#style)).
- Defend the data structures and the shape of your decisions: real-time
  performance, quality and developer experience, in that order. See
  [`CONTRIBUTING.md` § Defend your design](CONTRIBUTING.md#defend-your-design).

---

# Project context

## What this is

`kmr` is a Rust framework for writing real-time (target ~1kHz) robot
controllers. Users register plain functions against schedules; the engine owns
the control loop, state, timing, and hardware. The mental model is borrowed
from Bevy's ECS (functions on schedules) but **deliberately stops short of a
query system** — a robot has one fixed-shape state, not an open world of
entities. Read [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) and
[`docs/ROADMAP.md`](docs/ROADMAP.md) before large changes.

## Repo layout — one Cargo workspace

The git root is a single Cargo workspace with three crates:

- `kmr_core/` — the **engine**. May use advanced Rust; exposes only what is
  really needed.
- `kmr_api/` — the **public user API**: thin wrappers and the convenience
  layer. Must stay easy to read for users.
- `controllers/` — where controllers are written. Depends on `kmr_api` only,
  so Cargo itself refuses `use kmr_core::…` there.

`kmr_api` is a **convenience layer**, not a DTO layer. The value types
`Q`/`Qd`/`Tau` are defined once, in `kmr_core`, and `kmr_api` re-exports them
(`pub use`); the sealed `StateField` bound is re-exported as `Field`. Do not
reintroduce api-side copies or a conversion trait — the duplication was a
closed-source leftover. Because of the orphan rule, anything users need *on*
those types (arithmetic, `Default`, `From<f32>`) lives in the core; keep that
surface minimal. Core *handles* are still never forwarded raw: `State`,
`Time`, `Sensors` are borrowed wrappers with a private inner field, so the
user-facing method set stays a deliberate choice.

## Commands

```sh
# from the git root
cargo build --workspace
cargo test --workspace --no-fail-fast
cargo test -p kmr_core bus       # single test by name substring
cargo clippy --workspace         # clippy.toml bans heap types — see below
cargo clippy --workspace --all-targets   # also lints tests and examples
cargo fmt --all --check
cargo run -p kmr_api --example complete_api_example
```

Rust edition 2024.

## Hard constraints

- **No heap allocation, except in the API.** The root `clippy.toml` sets
  `disallowed-types` = `Box`, `Vec`, `String`, `Rc`, `Arc`, `HashMap` for the
  whole workspace. `kmr_api/clippy.toml` overrides it with an empty list: the
  API is the one crate where heap types are allowed. Crates outside the
  loop override it the same way: `kmr_model` and `kmr_model_core` (build
  time only) and `kmr_ffi`
  (build helper and script bridges; never in kmr's own per-call work, SRS
  FR-10.3). Elsewhere use fixed arrays and `'static`s. `kmr_core/src/lib.rs` also
  `#![forbid(clippy::disallowed_types)]` and `#![forbid(clippy::unwrap_used)]`.
- **The core builds `no_std` without `alloc`.** `std` is a default feature of
  `kmr_core` (Ctrl-C, logging, system clock, threads). CI builds
  `cargo build -p kmr_core --no-default-features --target thumbv7em-none-eabihf`:
  import from `core::`, log through `crate::logging`, gate anything hosted
  behind `#[cfg(feature = "std")]`.
- **Everything is `pub(crate)` by default.** Only what `kmr_api` must name is
  `pub`. When adding a type, default to private and widen only if `kmr_api`
  needs it.
- **State fields are sealed and never publicly writable.** See below.

## Architecture

### Static dispatch via HList (frunk_core), no `dyn`

Controllers are stored in a heterogeneous list (`frunk_core`'s `HCons`/`HNil`),
monomorphized — zero `dyn`, zero fat pointers. `Robot<L>` (`robot.rs`) *has*
the list as a field; each `.add_controller*` prepends one node, growing the
type by one `HCons` layer. The node types are `Inline` / `InlineWith` /
`Threaded` (`controllers.rs`) — opaque containers holding
`(schedule, [ctx,] fn)`. Fields are `pub(crate)`, constructors
`#[doc(hidden)]`: a user can hold an `HCons` but can neither forge a node nor
read one out.

An empty robot is a **compile error** by type-state (decision 0008):
`Robot<N, CS, E = Empty>`, every `add_*` returns `E = Filled`, and
`run()`/`run_on()` need `E: HasController` (sealed in `robot.rs`). Its
`#[diagnostic::on_unimplemented]` puts "kmr: this robot has no controller"
first, so the internal type paths stay in the notes below it. `kmr_api`
threads the same `E` through its wrapper.

### Schedules — a pre-sorted fixed pipeline (`schedule.rs`)

The `schedules!` macro generates, from one ordered `field => Label` list: the
zero-sized label structs (`PreInit, Init, PostInit, First, EachTick, Last`),
their sealed `Schedule` impls, a `Schedules<..>` struct with one bucket HList
per phase, and one `Insert<Label, Node>` impl per phase. Routing happens
**once**, when a controller is added — the label is the compile-time key that
picks the bucket to grow. Per tick the runtime walks each bucket once in
field-declaration order: zero phase compares, zero filtering.
`PreInit/Init/PostInit` are startup (walked once); `First/EachTick/Last` run
every tick. `Phase` is introspection only, never touched for dispatch.

**These phase names are engineer-chosen, not Bevy's.** Do not reintroduce Bevy
names (`Update`, `Startup`, `RunFixedMainLoop`, …). The schedule is the source
of truth.

### Runtime (`runtime.rs`)

`runtime()` installs tracing, resets the signal bus, installs the ctrl-c
handler that fires `shutdown!` (after the reset, so a Ctrl-C is never erased;
if the process already has a handler of its own, a warning says that Ctrl-C
will not stop the loop), then runs
`pre_init → init → post_init` and loops `first → each_tick → last` until a
signal is raised. It returns the `Signal` that ended the run; `Robot::run()`
returns `Ok(())` for a shutdown and for an emergency stop alike.

- **Shutdown is graceful**: the tick that saw it is completed (`Last` bucket
  and clock bookkeeping included), then the loop exits without sleeping.
- **Emergency stop is immediate**: the loop exits at the first bus check after
  it was raised. The bus is checked between the phases, startup phases
  included, so the granularity is one phase bucket.
- **One clock**: `robot.config.time`. Each phase builds its own short-lived
  `Env`, so the engine can mutate the clock between the controller walks
  while controllers only ever get `&Time`. Do not hold an `Env` across phases.
- **`Host` seam**: the loop takes the instant, the sleep and the bus through
  the `Host` trait, public and sealed like `hal::seal::Adapter`
  (`kmr_core::host_seal`), on the core's own `Instant` (`RealHost` in
  production under `std`, monomorphized, no `dyn`; its sleep is an absolute `clock_nanosleep` on the monotonic clock,
  an optional spin tail off by default). Loop tests use a scripted host: they never sleep and never touch the
  global bus. Tests on the real bus live in `kmr_core/tests/run_signals.rs`
  (own process) and take a file-local lock.

The hardware contract is `hal.rs`: `run_on` takes a list of drivers
(`hal/set.rs`), and the `Powered` guard ends every exit, unwind included,
with `torque_off` then `close` on every member. `Threaded` controllers are
stored but never spawned (hand-off open in the ROADMAP).

### Signal bus (`signal.rs`) — the only channel from controllers back to the loop

A process-global, **lock-free** bus (`AtomicU8` level +
`AtomicPtr<SignalMeta>`). Raised with the exported `shutdown!` /
`emergency_stop!` macros, callable from any depth (nested helper, closure,
other thread, raw signal handler). Macros — not free fns — because each
expansion mints a per-call-site `static SignalMeta` (`file!`/`line!` folded in
only under `debug_assertions`), so nothing allocates and the path is
async-signal-safe. `compare_exchange` is monotonic: estop never downgrades to
shutdown, first winner keeps its meta. Runtime calls `Signal::reset()` once at
entry and `Signal::drain()` five times per tick (happy path = one atomic load
each). `drain()` loads and never clears: the bus stays raised until the next
`reset()`.

### State (`state/`) — private, sealed, typed

`RobotState<N>` holds a write-only `Desired` staging area + a read-only
`History` ring buffer (`circular-buffer`). `N`, the actuator count, is a const
generic set by `robot!` from the descriptor (decision 0007): every type that
holds a per-actuator array carries it, nothing else does, and it has no
default. `HISTORY_DEPTH` stays a fixed const, public as
`kmr_core::state::HISTORY_DEPTH`. The write mask is `Mask<N>([u64;
MASK_WORDS])`, `MASK_WORDS = 1`: N above 64 fails the build. Users never touch raw fields
(`robot.desired.q[0] = ..` is a compile error). Access is setters/getters
only, all speaking typed newtypes `Q`/`Qd`/`Tau` — never bare `f32`.

**The field set is sealed** (`state/field.rs`): a private `sealed::Slot` trait
routes each field type to its storage slot; the public `StateField:
sealed::Slot` can be named as a bound by `kmr_api` but no downstream crate can
implement it, so the writable field set (`Q`, `Qd`, `Tau`) is fixed. `kmr_api`
re-exports it under the name `Field`. Adding a
field type means adding both a `Slot` impl and a `StateField` impl there.

### Time (`clock.rs`)

`Time` is internal state; all methods `pub(crate)`. The only user knob is
`Robot::set_dt(Duration)`, kept as an `Option` in the config. At
finalization, before any I/O, a `dt` outside [10 µs, 1 s] (zero included) is
`RunError::Config(DtOutOfRange)`; never set, it falls back to 1 ms with a
warning. The runtime then hands it to `Time::set_delta`.

The clock never reads the system time: the runtime injects every instant
(`arm`, `begin_tick`, `end_tick`), which keeps the arithmetic testable without
sleeping. Deadlines are **absolute**, every one of them is
`origin + k * delta`, so the schedule does not drift. Overrun is the lateness
against the deadline. After an overrun the grid is kept: missed slots are
skipped and the loop sleeps to the first grid point still ahead, so the next
tick starts on the grid with a whole `delta` as its budget. No catch-up burst,
no re-anchoring, no tick squeezed in what is left of a slot.

## Style

Comments follow [`CONTRIBUTING.md` rule 8](CONTRIBUTING.md#engineering-rules):
short doc comments on every module, type, function and method; inside a body,
one short line only where the code is really ambiguous. Never restate the code
in English.
