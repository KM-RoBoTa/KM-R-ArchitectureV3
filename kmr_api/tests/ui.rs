//! Compile-fail cases: what a user crate must not be able to write (SC-008 (SRS NFR-3)).
//!
//! Each case in `tests/ui/` builds as its own crate; its pinned `.stderr` is the
//! diagnostic the user reads. `TRYBUILD=overwrite` rewrites them after a reviewed change.
//!
//! A case names its model fixture relative to that crate, which trybuild writes under
//! `<target>/tests/trybuild/kmr_api/`: the path climbs four levels to the workspace root, so
//! the cases need the workspace's own `target/` directory.

use std::path::Path;

#[test]
fn ui() {
    let target = Path::new(env!("CARGO_MANIFEST_DIR")).join("../target");
    let in_target = match (std::env::current_exe(), target.canonicalize()) {
        (Ok(exe), Ok(target)) => exe.starts_with(target),
        _ => false,
    };
    assert!(
        in_target,
        "the ui cases reach their fixtures through <workspace>/target/tests/trybuild/kmr_api/; \
         run them without CARGO_TARGET_DIR, --target-dir or build.target-dir"
    );
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/*.rs");
    t.pass("tests/ui/pass/*.rs");
    #[cfg(feature = "ffi")]
    t.compile_fail("tests/ui/ffi/*.rs");
}
