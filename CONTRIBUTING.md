# Contributing to kmr

Thanks for your interest. This document is the contribution policy for humans
and AI agents alike. AI agents enter through [`AGENTS.md`](AGENTS.md)
(`CLAUDE.md` is a symlink to it), which holds the authoritative git & commit
rules and the project context; this document expands on them and does not
override them.

## How kmr is built

`kmr` is a real-time robotics engine. Its value is in invariants that are easy
to break and hard to see break: no allocation on the control path, a sealed
state field set, lock-free signalling, zero-`dyn` static dispatch.

It is built spec-first: [`docs/srs/SRS.md`](docs/srs/SRS.md) says what, the
cycle in `workflows/cycle/` says who builds each module and how, and agents
write most of the code under [`AGENTS.md`](AGENTS.md). Whoever wrote a line,
human or agent, it gets the same review, and the maintainer must be able to
explain it before it merges.

## License of contributions

The project is under the [MIT License](LICENSE) as a **temporary** license while
the final one is decided. By contributing you agree that your contribution is
licensed under the repository's license at the time, and you acknowledge that
the maintainers intend to relicense the project. Contributors may be asked to
confirm their agreement when that happens.

## Getting started

Prerequisites: a stable Rust toolchain supporting edition 2024.

The git root is one Cargo workspace:

| Path           | Role                                                          |
|----------------|---------------------------------------------------------------|
| `kmr_core/`    | The engine: state, schedules, runtime, signal bus, clock.     |
| `kmr_api/`     | The public user API: thin wrappers and the convenience layer. |
| `controllers/` | Where controllers are written. Depends on `kmr_api` only.     |

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

Read [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) before touching the
engine, and [`docs/ROADMAP.md`](docs/ROADMAP.md) to see what is
open.

## Engineering rules

These are enforced by the compiler and clippy where possible, and by review
where not.

1. **No heap, except in the API.** `Box`, `Vec`, `String`, `Rc`, `Arc`,
   `HashMap` are banned workspace-wide by the root `clippy.toml`, and
   forbidden in `kmr_core`. Only `kmr_api` opts out (`kmr_api/clippy.toml`).
   Use fixed arrays, const generics and `'static`s.
2. **No `unwrap()` in the engine.** Propagate or handle errors.
3. **No `dyn`.** Controllers are dispatched statically through the HList. If
   your change needs dynamic dispatch, open an issue first.
4. **Private by default.** Everything in `kmr_core` is `pub(crate)` unless
   `kmr_api` must name it.
5. **`kmr_core` never appears in a public `kmr_api` signature.** Add a
   capability by writing a re-exposure (newtype + mirrored methods), never by
   forwarding a core type.
6. **The state field set is sealed.** New fields need both a `Slot` and a
   `StateField` impl in `state/field.rs`, and a design discussion first.
7. **Schedule names are fixed.** `PreInit, Init, PostInit, First, EachTick,
   Last`. Don't reintroduce Bevy names.
8. **Rust is the authority; comment only real ambiguity.** Every module,
   type, function and method has a short doc comment. Inside a body, comment
   only what the code cannot say (a hidden invariant, a trap), in one short
   line. Never restate the code in English: `if self.delta.is_zero()` needs
   no comment. A reviewer should not read everything twice.

Architecture changes (new phases, new state fields, threading model, anything
touching the ROADMAP's open questions) start as an **issue**, not a PR.

## Defend your design

Every data structure and every decision that gives the code its shape must be
defended, in the pull request and, where it is not obvious, in a comment next
to the code. "It works" is not a defence. This applies to humans and to AI
agents alike, and to small changes as much as to large ones.

A defence answers three questions, in this order of priority when they
conflict:

1. **Real-time performance.** What does this cost on the control path at
   1 kHz? State the memory layout and size, whether anything allocates, locks,
   blocks or makes a system call, how many copies and branches a tick pays,
   and what the worst case is, not the average. If you claim it is free,
   say why: inlined, monomorphized, fixed-size, known at compile time.
   Measure when you can; a number beats an argument.
2. **Quality.** Which invariant does this shape make impossible to break, and
   which misuse becomes a compile error and no longer a runtime surprise? What
   does it cost to change later?
3. **Developer experience.** What does the user write, read and see in a
   compiler error? Would someone who knows only basic Rust understand it?

Then name **the alternative you rejected** and why it lost. A choice with no
alternative considered has not been defended.

Typical things that need a defence: a new struct, enum or trait; a field
added to the state; an array versus a ring buffer; a generic versus a
constant; a newtype versus a bare value; a macro versus a function; owned
versus borrowed returns; anything that widens a `pub`.

A reviewer may ask for the defence of any line. A contribution whose shape
cannot be defended is reworked, however correct it is.

## Commits and branches

1. Commit messages follow [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/):
   `feat(core): …`, `fix(api): …`, `docs: …`, `chore: …`.
2. Branch names follow [Conventional Branch](https://conventional-branch.github.io/):
   `feature/…`, `fix/…`, `docs/…`, `chore/…`.
3. Every contribution briefly explains what it does and why — in the commit
   body, the PR description, or both.
4. Open pull requests with
   [`.github/pull_request_template.md`](.github/pull_request_template.md).

## Pull requests and `main`

`main` is protected:

- No direct pushes, no force pushes, no deletion — everything lands through a
  pull request.
- At least one approving review from someone other than the author; new pushes
  dismiss stale approvals; all review threads must be resolved.
- CI must be green (build, tests, clippy, rustfmt, and the human-authorship
  check).
- History stays linear (squash or rebase merge).

Before requesting review:

- [ ] `cargo fmt --check`, `cargo clippy` and `cargo test` pass for the
      whole workspace.
- [ ] New behaviour has tests; changed behaviour has updated tests.
- [ ] Public API changes are reflected in `kmr_api` docs and the example.
- [ ] Every data structure and design decision is defended in the PR
      (see [Defend your design](#defend-your-design)).
- [ ] You re-read the whole diff and can justify every line.

## AI agents

Agents follow [`AGENTS.md`](AGENTS.md): the cycle, the decision policy, and
the git rules. In short:

1. **Commits name a human.** They are made under the maintainer's identity,
   with no `Co-authored-by:` trailer and no "generated with" footer for an AI
   tool. CI rejects an AI as author or committer.
2. **Choices are visible.** An agent marks each internal choice with a
   `// DECISION:` line and writes every decision it may not take alone to
   `docs/deferred.md`.
3. **The oracle is the gate.** Nothing reaches review unless tests, clippy and
   the pub diff in `workflows/cycle/QA.md` are green, or the report says which
   one is red and why.
4. **A human merges.** An agent never approves or merges a pull request.
5. **No secrets.** Do not paste credentials, private keys or non-public
   hardware/customer data into an AI tool.
