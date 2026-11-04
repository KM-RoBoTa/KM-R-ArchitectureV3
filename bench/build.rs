//! Compiles the foreign body of `tick_ffi` (feature `ffi`) for the robot it names, and nothing
//! else (assumed, pending human: D12 A). Without the feature it does nothing.

#[cfg(feature = "ffi")]
fn main() -> Result<(), kmr_api::ffi::build::Error> {
    println!("cargo::rerun-if-changed=ffi/empty.c");
    kmr_api::ffi::build::robot("XmBench").compile(&["ffi/empty.c"])
}

#[cfg(not(feature = "ffi"))]
fn main() {}
