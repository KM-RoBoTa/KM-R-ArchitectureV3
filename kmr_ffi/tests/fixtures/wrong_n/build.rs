//! Builds `src/body.c` for the robot `KMR_FFI_FIXTURE_MODEL` names, `two.urdf` by default.

fn main() -> Result<(), kmr_ffi::build::Error> {
    println!("cargo:rerun-if-env-changed=KMR_FFI_FIXTURE_MODEL");
    let model = std::env::var("KMR_FFI_FIXTURE_MODEL").unwrap_or_else(|_| "two.urdf".into());
    kmr_ffi::build::model(model, kmr_api::HISTORY_DEPTH).compile(&["src/body.c"])
}
