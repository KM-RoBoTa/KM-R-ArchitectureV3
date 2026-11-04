//! `foreign!` against the N of `robot!`: the fixture crate tests/fixtures/wrong_n builds C for
//! the robot `KMR_FFI_FIXTURE_MODEL` names and declares one actuator in `robot!` (T012, T017).

use std::path::{Path, PathBuf};
use std::process::{Command, Output};
use std::sync::Mutex;

/// One cargo run on the fixture at a time: both share its target dir and build script.
static FIXTURE: Mutex<()> = Mutex::new(());

/// The fixture crate.
fn fixture() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/wrong_n")
}

/// `cargo <cmd>` on the fixture, its C built for `model`.
fn cargo(cmd: &str, model: &str) -> Output {
    let _one = FIXTURE.lock().unwrap_or_else(|e| e.into_inner());
    let dir = fixture();
    let lock = dir.join("Cargo.lock");
    if !lock.exists() {
        // The workspace's versions, all in the local cache: the run needs no network.
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../Cargo.lock");
        std::fs::copy(root, &lock).expect("copy the workspace Cargo.lock");
    }
    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    Command::new(cargo)
        .arg(cmd)
        .args(["--offline", "--quiet", "--manifest-path"])
        .arg(dir.join("Cargo.toml"))
        .arg("--target-dir")
        .arg(Path::new(env!("CARGO_TARGET_TMPDIR")).join("wrong_n"))
        .env("KMR_FFI_FIXTURE_MODEL", model)
        .env_remove("RUSTFLAGS")
        .output()
        .expect("cargo runs")
}

#[test]
fn c_built_for_the_robot_of_robot_macro_links() {
    let out = cargo("build", "one.urdf");
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
}

#[test]
fn c_built_for_another_n_fails_at_foreign() {
    let out = cargo("check", "two.urdf");
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(!out.status.success(), "{err}");
    assert!(err.contains("error[E0080]"), "{err}");
    assert!(
        err.contains("kmr_ffi: C sources were built for another N"),
        "{err}"
    );
    assert!(err.contains("src/main.rs:5:1"), "{err}");
}
