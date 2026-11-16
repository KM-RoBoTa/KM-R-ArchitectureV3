//! Builds `src/body.c` for the robot `KMR_FFI_FIXTURE_MODEL` names, `two.urdf` by default.

fn main() -> Result<(), kmr_api::ffi::build::Error> {
    println!("cargo:rerun-if-env-changed=KMR_FFI_FIXTURE_MODEL");
    let model = std::env::var("KMR_FFI_FIXTURE_MODEL").unwrap_or_else(|_| "two.urdf".into());
    kmr_api::ffi::build::model(model).compile(&["src/body.c"])
}
