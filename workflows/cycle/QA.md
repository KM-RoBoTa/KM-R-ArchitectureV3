# QA checklist

The review agent reads this before any PR, at step 6. `[oracle]` items
are mechanical: run the command, pass or fail. `[reviewer]` items need
judgment: read, compare, state the finding in one line. The profile tag
(`[serious]`, `[poc, serious]`) says which ROADMAP profiles run the item;
`toy` runs none.

Any failed item blocks the PR. The report lists every item with
pass/fail and, for `[reviewer]` items, one line of evidence.

## Code

- [ ] `[oracle]` `[poc, serious]` Tests green. `cargo test --workspace`, zero failures.
      Tests marked `#[ignore = "deferred"]` are listed in the report,
      not hidden.
- [ ] `[oracle]` `[serious]` Clippy clean.
      `cargo clippy --workspace --all-targets -- -D warnings`, zero
      warnings. This also catches leftover `todo!()`, `unimplemented!()`
      and `dbg!` (see `Cargo.toml.snippet`).
- [ ] `[oracle]` `[poc, serious]` Pub diff empty against the step-4 contract, unless the
      task text says otherwise. Compare `pub` items, signatures,
      visibility, re-exports, trait impls on pub types.
- [ ] `[oracle]` `[serious]` No `mod.rs` anywhere. `clippy::mod_module_files` is
      deny; a module is `foo.rs` with children in `foo/`.
- [ ] `[reviewer]` `[serious]` Hot structs are `#[repr(C)]` and each has a
      `size_of` test pinning its size. Hot = named as hot path in the
      SRS or HLD, or crossing an FFI or serialization boundary.

## Docs

- [ ] `[reviewer]` `[poc, serious]` The module's SRS section matches the code. Every
      requirement id the module covers has a test that names it (`[serious]` only). Every pub item is
      implied by a requirement. Nothing in the code contradicts a
      requirement.
- [ ] `[reviewer]` `[poc, serious]` A C4 box for the module exists in
      `docs/c4/<module>.dsl`, and the dependencies drawn there match the
      module's `use` lines.
- [ ] `[reviewer]` `[poc, serious]` Every `// DECISION:` comment added in this task is
      listed in the report. Every deferral raised in this task is in
      `docs/deferred.md` with its smallest unblocking change.
- [ ] `[reviewer]` `[poc, serious]` Plan file `docs/plans/<module>.md`
      exists and every task is ticked or in `docs/deferred.md`.

## Boundaries

- [ ] `[reviewer]` `[poc, serious]` Modules whose ROADMAP owner is `mine` were not
      touched by a loop. If one was, its files are listed first in the
      summary with the reason, and the owner has seen that list.
- [ ] `[reviewer]` `[poc, serious]` No new dependency, no `unsafe`, no `#[repr]` or
      field-order change, unless the task text asked for it or
      `docs/deferred.md` records the owner's decision.
