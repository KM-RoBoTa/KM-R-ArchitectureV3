//! Compiles the C and C++ controllers (feature `c`) for the robot of the examples; without the
//! feature it does nothing.

#[cfg(feature = "c")]
fn main() -> Result<(), kmr_api::ffi::build::Error> {
    kmr_api::ffi::build::model("models/ax_12_12act_an_rx28_10act.urdf")
        .compile(&["c/clap.c", "cpp/clap.cpp"])
}

#[cfg(not(feature = "c"))]
fn main() {
    println!("cargo:rerun-if-changed=build.rs");
}
