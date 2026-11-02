//! Compile-fail cases: what a user crate must not be able to write (SC-008 (SRS NFR-3)).
//!
//! Each case in `tests/ui/` builds as its own crate; its pinned `.stderr` is the
//! diagnostic the user reads. `TRYBUILD=overwrite` rewrites them after a reviewed change.

#[test]
fn ui() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/*.rs");
}
