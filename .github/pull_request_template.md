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

## AI assistance
- [ ] No AI tool was used
- [ ] An AI tool was used, within CONTRIBUTING.md § AI assistance, for: <!-- e.g. review, explaining X, drafting tests -->

## Checklist
- [ ] `cargo fmt --check`, `cargo clippy` and `cargo test` pass for the whole workspace
- [ ] Tests added / updated for the change
- [ ] Public API changes reflected in `kmr_api` docs and example
- [ ] Every data structure and design decision is defended above
- [ ] I can explain every line of this diff without assistance
