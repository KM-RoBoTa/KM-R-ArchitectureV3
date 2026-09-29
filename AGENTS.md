# AI Agent Instructions

These are the common instructions for any AI agent or assistant working in this
repository — including **Claude** (Claude Code, claude.ai), **ChatGPT / Codex**,
**Cursor** and **Copilot**. Read them before making any change.

This file is the single source of truth for agent rules. `CLAUDE.md` is a
symlink to it, and `.cursor/rules/` only points here — do not duplicate rules,
extend them here. For the fuller contribution policy, see
[`CONTRIBUTING.md`](CONTRIBUTING.md): follow it wherever it goes deeper, but the
rules below always apply.

## This project is hand-engineered

`kmr` is written by humans. Hand engineering is the norm; AI assistance is
**tolerated** under the rules in this file and in
[`CONTRIBUTING.md` § AI assistance](CONTRIBUTING.md#ai-assistance). It is not
the primary author of anything.

In practice, as an agent you should default to *helping the human write the
code*, not writing it for them:

- Prefer explaining, reviewing, pointing at the relevant invariant, or
  sketching an approach over producing a finished diff.
- When you do write code, keep it small and scoped to what was asked. Large
  generated changes (new modules, new subsystems, rewrites) are out of bounds
  unless the human explicitly asks for that specific change.
- Never touch the tricky invariants on your own initiative: the sealing in
  `state/field.rs` and `sealed.rs`, the lock-free ordering in `signal.rs`, the
  HList/`Insert` machinery in `schedule.rs` / `controllers.rs`. Explain, don't
  edit, unless asked.

## Git & commit rules

If you are an AI agent and you want to create a commit, you are **disallowed**
to:

**a) Push to the `main` branch without creating a Pull Request first.**
   Always work on a dedicated branch and open a PR. `main` is protected and
   must only ever be updated through a reviewed and merged PR.

**b) Credit yourself.**
   Do not add `Co-Authored-By:` trailers, `Co-authored-by` lines, "Generated
   with" footers, or any other attribution naming the AI agent (Claude,
   ChatGPT, Cursor, Copilot, …) in commit messages or PR descriptions. Commits
   are authored by the human running the tool, who is accountable for them.
   CI rejects PRs that break this rule.

## Agentic work

Hand engineering is the default. A maintainer may explicitly lift that for a
session and let an agent write and commit on its own. That work is kept
visibly apart from human-controlled branches:

- **Dedicated branch.** Agentic work happens only on an `agent/<topic>`
  branch, created from the human branch it targets. This prefix is a
  deliberate exception to Conventional Branch. Never commit agentic work
  directly on a human-controlled branch.
- **PR into the human branch.** Open a pull request from `agent/<topic>` into
  the branch it was created from — never into `main`. A human reviews and
  merges it.
- **Do not block yourself.** When working unattended, do not stop on an open
  question: pick the most sensible option, prefer the reversible one, and keep
  going.
- **Report every decision.** The PR description carries a "Decisions" section
  listing each choice made without a human, the alternative, and why. It also
  lists what was deliberately left undone.
- **Say that it is agentic.** The PR description states that the work was
  produced by an agent and who authorized it. Rule (b) still applies: no
  co-author trailer, no "generated with" footer.

## Commits, branches & pull requests

- **Commits** follow [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/),
  e.g. `feat(core): add overrun accounting` or `fix(api): bound group indices`.
- **Branches** follow [Conventional Branch](https://conventional-branch.github.io/),
  e.g. `feature/overrun-accounting` or `fix/group-bounds`.
- **Explain the change** in the commit body, the PR description, or both.
- **Pull requests** use [`.github/pull_request_template.md`](.github/pull_request_template.md).

## General guidance

- Keep changes scoped to what was requested; ask before large or destructive
  edits.
- No silent guessing: if a requirement is ambiguous, ask. State assumptions.
  When working unattended, see [Agentic work](#agentic-work): decide, and
  report the decision.
- Match the existing style (see [Style](#style)).
- AI-assisted code gets the same review as any other; the human must be able to
  explain every submitted line.

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
cargo fmt --all --check
cargo run -p kmr_api --example complete_api_example
```

Rust edition 2024.

## Hard constraints

- **No heap allocation, except in the API.** The root `clippy.toml` sets
  `disallowed-types` = `Box`, `Vec`, `String`, `Rc`, `Arc`, `HashMap` for the
  whole workspace. `kmr_api/clippy.toml` overrides it with an empty list: the
  API is the one crate where heap types are allowed. Elsewhere use fixed
  arrays and `'static`s. `kmr_core/src/lib.rs` also
  `#![forbid(clippy::disallowed_types)]` and `#![forbid(clippy::unwrap_used)]`.
- **Everything is `pub(crate)` by default.** Only what `kmr_api` must name is
  `pub`. When adding a type, default to private and widen only if `kmr_api`
  needs it.
- **State fields are sealed and never publicly writable.** See below.

## Architecture

### Static dispatch via HList (frunk), no `dyn`

Controllers are stored in a heterogeneous list (`frunk`'s `HCons`/`HNil`),
monomorphized — zero `dyn`, zero fat pointers. `Robot<L>` (`robot.rs`) *has*
the list as a field; each `.add_controller*` prepends one node, growing the
type by one `HCons` layer. The node types are `Inline` / `InlineWith` /
`Threaded` (`controllers.rs`) — opaque containers holding
`(schedule, [ctx,] fn)`. Fields are `pub(crate)`, constructors
`#[doc(hidden)]`: a user can hold an `HCons` but can neither forge a node nor
read one out.

`Robot::run()` uses a **runtime** `L::LEN == 0` check
(→ `ApiError::NoControllers`) rather than a compile-time bound, because a
trait-bound gate leaks internal type paths (`frunk_core::...`, core file
layout) into `E0277` diagnostics at the user call site.

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

`runtime()` resets the signal bus, installs the ctrl-c handler that fires
`shutdown!` (after the reset, so a Ctrl-C is never erased), then runs
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
  the private `Host` trait (`RealHost` in production, monomorphized, no
  `dyn`). Loop tests use a scripted host: they never sleep and never touch the
  global bus. Tests on the real bus live in `kmr_core/tests/run_signals.rs`
  (own process) and take a file-local lock.

The hardware layer does not exist yet: the init bodies, the
`state_transition` step, the final write of a shutdown and the torque cut of
an emergency stop are TODOs. `Threaded` controllers are stored but never
spawned (hand-off open in the ROADMAP). `Robot::run()` does not check for an
empty controller list yet, whatever the section above says.

### Signal bus (`signal.rs`) — the only channel from controllers back to the loop

A process-global, **lock-free** bus (`AtomicU8` level +
`AtomicPtr<SignalMeta>`). Raised with the exported `shutdown!` /
`emergency_stop!` macros, callable from any depth (nested helper, closure,
other thread, raw signal handler). Macros — not free fns — because each
expansion mints a per-call-site `static SignalMeta` (`file!`/`line!` folded in
only under `debug_assertions`), so nothing allocates and the path is
async-signal-safe. `compare_exchange` is monotonic: estop never downgrades to
shutdown, first winner keeps its meta. Runtime calls `Signal::reset()` once at
entry and `Signal::drain()` four times per tick (happy path = one atomic load
each). `drain()` loads and never clears: the bus stays raised until the next
`reset()`.

### State (`state/`) — private, sealed, typed

`RobotState<DEPTH>` holds a write-only `Desired` staging area + a read-only
`History` ring buffer (`circular-buffer`). `DEPTH` defaults to `HISTORY_DEPTH`:
write a bare `RobotState`, never `RobotState<JOINTS>`. `JOINTS`/`HISTORY_DEPTH`
are fixed consts, defined once (`state.rs`) and public under one path,
`kmr_core::state` — a build-time-from-model value is a TODO. Users never touch raw fields
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
`Robot::set_dt(Duration)`, which forwards to `Time::set_delta`. A `dt` of zero
(or never set) falls back to 1 ms with a warning.

The clock never reads the system time: the runtime injects every instant
(`arm`, `begin_tick`, `end_tick`), which keeps the arithmetic testable without
sleeping. Deadlines are **absolute**, every one of them is
`origin + k * delta`, so the schedule does not drift. Overrun is the lateness
against the deadline. After an overrun the grid is kept: missed slots are
skipped, the next tick starts at once with what is left of its slot. No
catch-up burst, no re-anchoring.

## Style

Comment density: the code carries dense *why* comments on the tricky
invariants (sealing, lock-free ordering, macro rationale). Match that where the
reasoning is non-obvious; do **not** add headers that restate the code. Keep
module/doc comments minimal and accurate — trust code over headers.
