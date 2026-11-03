# Roadmap: cycle

## Tooling

- Claude Code with Superpowers: `brainstorming`, `writing-plans`,
  `requesting-code-review`.
- Claude Code `/goal` for loop mode: the agent runs against the oracle
  until green, no questions, under `CLAUDE.md`.
- Project skills in `.claude/skills/`, all user commands: `/start`,
  `/issue`, `/mode`, `/contract`. Step end is automatic: the agent
  runs it at the end of steps 5, 6 and 7 (see `PROCEDURE.md`).
- Oracle = the `[oracle]` items in `QA.md`: `cargo test`, `cargo clippy`,
  pub diff. Reviewer = the `[reviewer]` items, read by the review agent
  at step 6.

## Entry points

- **New module**: steps 1→9. Starts from a C4 box that has no code yet.
- **Change** (bug, small feature, refactor): steps 4→7. Starts from an
  existing module. Step 4 is a no-op when the pub surface is untouched;
  it is a real contract when it is not.

## Dependency rule

Order comes from the skeleton (step 3): the `//!` doc comments and `use`
lines of the empty modules. A module enters step 4 only when every
module it depends on is at step 4 or later.

## Status

`profile: toy | poc | serious` — serious = full `QA.md`; poc = oracle
items only; toy = build + run only. `docs/srs/SRS.md` non-functional section
overrides `QA.md`.

`profile: serious`

Every crate entered the cycle on 2026-11-03 from spec-kit, already being
built, so all of them start at step 5 without a step-4 sign-off. Their
open tasks are in `docs/plans/<module>.md`.

| module | owner | mode | step | branch | last oracle run |
|---|---|---|---|---|---|
| kmr_core | mine | unset | 5 | — | `2026-11-03 red`: clippy, 6 errors |
| kmr_adapters | mine | unset | 5 | — | `2026-11-03 red`: tests green, clippy stops at kmr_core |
| kmr_model | mine | unset | 5 | — | `2026-11-03 red`: tests green, clippy stops at kmr_core |
| kmr_api | mine | unset | 5 | — | `2026-11-03 red`: tests green, clippy stops at kmr_core |
| controllers | mine | unset | 5 | — | `2026-11-03 red`: tests green, clippy stops at kmr_core |
| kmr_ffi | mine | unset | 5 | — | `2026-11-03 red`: tests green, clippy stops at kmr_core |
| bench | mine | unset | 5 | — | `2026-11-03 red`: tests green, clippy stops at kmr_core |

- **owner** `mine`: I write it. Agents do not touch its files without
  listing them first. `delegated`: an agent writes it under the step-4
  contract.
- **mode**: set with `/mode`. Default `pair` for `mine`, `loop` for
  `delegated`.
- **step**: the step the module is *in*, updated at step end.
- **branch**: one branch per module, one squash commit at step 7.
- **last oracle run**: date and green/red, written at step end.
