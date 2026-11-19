//! Two robots' C and C++ in one crate: the fixture crate tests/fixtures/two_robots compiles one
//! group per robot, and `kmr_api::c!` checks each function against the N of its `robot!`.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};
use std::sync::Mutex;

/// One cargo run on the fixture at a time: all share its target dir and build script.
static FIXTURE: Mutex<()> = Mutex::new(());

/// The fixture crate.
fn fixture() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/two_robots")
}

/// `cargo <cmd> --bin <bin>` on the fixture; `duplicate` makes its build.rs compile a function
/// in a second group.
fn cargo(cmd: &str, bin: &str, duplicate: bool) -> Output {
    let _one = FIXTURE.lock().unwrap_or_else(|e| e.into_inner());
    let dir = fixture();
    let lock = dir.join("Cargo.lock");
    if !lock.exists() {
        // The workspace's versions, all in the local cache: the run needs no network.
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../Cargo.lock");
        std::fs::copy(root, &lock).expect("copy the workspace Cargo.lock");
    }
    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let mut command = Command::new(cargo);
    command
        .arg(cmd)
        .args(["--offline", "--quiet", "--bin", bin, "--manifest-path"])
        .arg(dir.join("Cargo.toml"))
        // The target dir of tests/wrong_n.rs: the fixtures share kmr_api's build.
        .arg("--target-dir")
        .arg(Path::new(env!("CARGO_TARGET_TMPDIR")).join("fixtures"))
        .env_remove("RUSTFLAGS")
        .env_remove("KMR_FFI_FIXTURE_DUPLICATE");
    if duplicate {
        command.env("KMR_FFI_FIXTURE_DUPLICATE", "1");
    }
    command.output().expect("cargo runs")
}

#[test]
fn each_function_with_its_own_robot_builds_and_links() {
    let out = cargo("build", "right", false);
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
}

#[test]
fn a_function_of_the_other_robot_fails_at_the_c_macro() {
    let out = cargo("check", "crossed", false);
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(!out.status.success(), "{err}");
    assert!(err.contains("error[E0080]"), "{err}");
    assert!(
        err.contains("kmr: `two_cpp` was built for another robot, another N"),
        "{err}"
    );
    assert!(err.contains("src/bin/crossed.rs:6:26"), "{err}");
    assert_eq!(err.matches("error[E0080]").count(), 1, "{err}");
}

#[test]
fn a_function_no_group_defines_fails_at_the_c_macro() {
    let out = cargo("check", "unknown", false);
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(!out.status.success(), "{err}");
    assert!(
        err.contains("kmr: build.rs compiled no C or C++ function named `nobody`"),
        "{err}"
    );
    assert!(err.contains("src/bin/unknown.rs:6:26"), "{err}");
    assert_eq!(err.matches("error[E0080]").count(), 1, "{err}");
}

#[test]
fn a_function_two_groups_define_fails_the_build_script() {
    let out = cargo("check", "right", true);
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(!out.status.success(), "{err}");
    assert!(
        err.contains(
            "DuplicateSymbol { symbol: \"one_c\", first: [\"src/one.c\", \"src/one.cpp\"], \
             second: [\"src/one.c\"] }"
        ),
        "{err}"
    );
}
