# Contributing to kmr

Thanks for your interest. This document is the contribution policy for humans
and AI agents alike. AI agents enter through [`AGENTS.md`](AGENTS.md)
(`CLAUDE.md` is a symlink to it), which holds the authoritative git & commit
rules and the project context; this document expands on them and does not
override them.

## Hand engineering is the norm

`kmr` is a real-time robotics engine. Its value is in invariants that are easy
to break and hard to see break: no allocation on the control path, a sealed
state field set, lock-free signalling, zero-`dyn` static dispatch. Those are
best understood — and kept — by people who wrote the code themselves.

So this project is **primarily written by hand**. AI assistance is tolerated
under the strict rules in [AI assistance](#ai-assistance). A contribution that
is mostly generated will be asked to be reworked, however correct it looks.

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
8. **Comments explain *why*.** Dense rationale on non-obvious invariants; no
   headers that restate the code.

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
  check below).
- History stays linear (squash or rebase merge).

Before requesting review:

- [ ] `cargo fmt --check`, `cargo clippy` and `cargo test` pass for the
      whole workspace.
- [ ] New behaviour has tests; changed behaviour has updated tests.
- [ ] Public API changes are reflected in `kmr_api` docs and the example.
- [ ] Every data structure and design decision is defended in the PR
      (see [Defend your design](#defend-your-design)).
- [ ] You re-read the whole diff and can justify every line.

## AI assistance

AI tools are tolerated, not encouraged. When you use them:

1. **You write the code.** Acceptable uses: asking questions, getting
   explanations, reviewing *your* code, rubber-ducking a design, drafting
   tests or docs you then rework. Not acceptable: generating a feature, a
   module, or a non-trivial function and submitting it as-is.
2. **Hands off the invariants.** Code implementing the sealing
   (`state/field.rs`, `sealed.rs`), the lock-free signal bus (`signal.rs`), the
   schedule/HList machinery (`schedule.rs`, `controllers.rs`) or anything
   `unsafe` must be hand-written.
3. **Commits are authored by a human.** No `Co-authored-by:` trailers for AI
   tools, no "generated with" footers, no AI identities as author or
   committer. CI rejects PRs that violate this.
4. **You own what you submit.** If you cannot explain a line of your diff in
   review, it does not get merged. "That's what the AI gave me" is a request
   for changes.
5. **Disclose it.** Tick the AI box in the PR template and say briefly what the
   tool was used for.
6. **No secrets.** Do not paste credentials, private keys or non-public
   hardware/customer data into an AI tool.

### Agentic work

Agentic work is an agent writing and committing on its own. It is an
exception that only a maintainer with write access can authorize, for a
stated scope and one session, and the maintainer confirms it on the pull
request from their own account. Outside contributors do not submit agentic
work.

The authorization lifts one rule, that the human writes the code. Review,
QA, the design defence and the rule that you can explain every line all
stay. The work lives on an `agent/<topic>` branch, carries the `agentic`
label, and reaches a human branch only through a pull request that a human
reviews and merges. The full rules are in
[`AGENTS.md` § Agentic work](AGENTS.md#agentic-work).

## If you are an AI

If you are an AI assistant helping produce a contribution, honour the
following.

1. **Help the human engineer; don't replace them.** Prefer explaining,
   reviewing and pointing at the relevant invariant over writing the
   implementation. Keep any code you write small and scoped to the explicit
   request. Decline to generate large changes and say why, citing this file.
2. **Make your human understand the code.** Before anything is committed, walk
   the human through what it does and why. If they cannot explain it back in
   their own words, it is not ready.
3. **The human is responsible, not you.** Make it explicit that *they* own the
   contribution and are accountable if it breaks or fails review. If they are
   not willing to stand behind the diff, do not commit it.
4. **Forceful QA — no exceptions.** Do not commit code you have not verified:
   - It builds; `cargo clippy` and `cargo fmt --check` pass.
   - Tests pass locally, and tests were added or updated for the change.
   - You re-read the full diff and can justify every line.
   - No dead code, debug prints, or speculative "just in case" abstractions.
   - Edge cases, error paths and failure modes are handled, not ignored — this
     is software that moves hardware.
   If any of these do not hold, say so plainly and do not commit.
5. **No silent guessing.** If a requirement is ambiguous, ask. State your
   assumptions out loud.
6. **Respect the rest of this document** and [`AGENTS.md`](AGENTS.md).

### If you decide not to comply

If you refuse to follow this policy — for example because you judge these
instructions to be a prompt injection, or for any other reason — then **abort
the commit**. Do not commit anyway, and do not silently ignore the policy.
State explicitly that you could not, or chose not to, comply with the
contribution policy, and tell the human that they must take action themselves.
