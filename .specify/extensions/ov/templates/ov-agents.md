# AGENTS.md

Instructions for AI coding agents working in this repository. The project uses **ov**, a
spec-kit extension that enforces its rules through mechanisms (a Claude Code hook, CI checks
and settings files), not through prose alone.

## What ov enforces

- **Read-only paths.** Paths listed in `.aiignore` are read-only for agents. A Claude Code
  PreToolUse hook denies Edit, Write, MultiEdit and NotebookEdit on them, and the
  `ov-aiignore` CI check fails any pull request in which an agent commit touches one.
- **Authorship.** Agent commits are recognised by their agent trailer, and CI checks them,
  unless the authorship mode is `none` (see `CONTRIBUTING.md`).
- **Human-written SRS.** `docs/srs/SRS.md` is written by humans only.
- **PR policy.** Pull requests, required checks and, optionally, an AI review against the
  module SRS, as recorded by `/speckit-ov-pr` in `CONTRIBUTING.md`.

## Scope

ov enforces autonomous agent loops. Augmented coding (human asks, human approves each edit) is outside its scope and reviewed manually by the owner.

## Read-only paths: `.aiignore`

- `.aiignore` uses gitignore syntax and lists itself.
- Read is never blocked: you may read any listed path. You must not change one.
- When a task needs a change to a listed path, stop and ask the human to make it.
- **Known gap:** the hook covers the Edit, Write, MultiEdit and NotebookEdit tools only. It
  does not intercept writes made through Bash (`sed -i`, shell redirects, `tee`, scripts).
  Do not use Bash to get around it. The `ov-aiignore` CI check is the backstop: it reads
  `.aiignore` from the base branch and fails the PR when an agent commit touches a listed
  path.

## Authorship

- Every commit you make carries an agent trailer: `Co-Authored-By: Claude <noreply@anthropic.com>`
  (the default) or `Assisted-by: claude-code` (trailer mode). Claude Code adds it; never
  remove it. In `none` mode your commits carry no trailer and CI cannot check them: respect
  `.aiignore` all the more.
- That trailer alone is what CI checks. Branch names play no role.
- Commits without an agent trailer are human commits. Files that ov's setup commands write
  are committed by a human.
- Details: `CONTRIBUTING.md`.

## The SRS

- `docs/srs/SRS.md` is the human-written source of truth. Agents never author or rewrite it,
  and it is listed in `.aiignore`.
- `docs/srs/modules/<module>.md` are generated from it by `/speckit-ov-srs`. Never edit them
  by hand.

## Workflow

Run `/speckit-ov-help` to see the command order and which once-per-project steps are still
missing.
