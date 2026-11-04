## What
<!-- One or two sentences. -->

## Why
Closes #

## Design defence
<!-- Required when the change adds or reshapes a data structure, a type, a trait,
     a macro or a public item. See CONTRIBUTING.md § Defend your design.
     For each decision:
     - Real-time cost on the control path: layout, allocation, locks, copies, worst case.
     - Quality: which invariant it protects, which misuse becomes a compile error.
     - Developer experience: what the user writes and what errors they see.
     - The alternative you rejected, and why. -->

## Cycle
<!-- Module and step (workflows/cycle/ROADMAP.md), plan task ids, and the SRS ids covered. -->
- [ ] Every `// DECISION:` added here is listed below
- [ ] Every deferral raised here is in `docs/deferred.md`

## Checklist
- [ ] `cargo fmt --check`, `cargo clippy` and `cargo test` pass for the whole workspace
- [ ] Tests added / updated for the change
- [ ] Public API changes reflected in `kmr_api` docs and example
- [ ] Every data structure and design decision is defended above
- [ ] The maintainer can explain every line of this diff
