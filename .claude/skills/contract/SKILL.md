---
name: contract
description: Step 4 of the cycle. From one module SRS (docs/srs/modules/<module>.md), draft the module's pub stubs and test names with todo!() bodies. Nothing is implemented. Use on /contract <module>.
---

# contract

Input: a module name. Its SRS is
`docs/srs/modules/<module>.md`. Ask for the name if not given.

1. Read the SRS section, the module's `//!` doc comment in the skeleton,
   and its C4 box in `docs/c4/<module>.dsl`. If any is missing, stop and
   name it. Step 4 needs all three.
2. Check the dependency rule in `workflows/cycle/ROADMAP.md`: every
   dependency at step 4 or later. If not, stop and name the blocker.
3. Draft, in the module file:
   - every `pub` item the Interfaces subsection implies: full signature,
     `///` doc comment naming the requirement ids it serves, body `todo!()`
   - hot structs from the Data subsection: `#[repr(C)]`, typed fields,
     and a `size_of` test pinning the size
   - `#[cfg(test)] mod tests` with one `#[test] fn <mod>_<n>_<slug>()`
     per requirement `<MOD>-<n>`, body `todo!()`
4. No private items, no implementation, no new dependencies. Anything
   the SRS leaves open: a `TODO(srs)` comment in place and a row in
   `docs/deferred.md`.
5. Run `cargo build`: must be green. `cargo test` must compile.
6. Print the pub surface, one line per item, and ask for sign-off. On
   sign-off, set the module's ROADMAP row to step 5 and say so.

Rules:

- If the SRS section is thin, invoke `superpowers:brainstorming` to fill
  it before drafting. Never draft stubs from guesses.
- Never touch another module's file.
