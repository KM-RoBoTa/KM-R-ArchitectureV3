//! Compiles the C bodies for the robot `robot!` names in src/main.rs.

fn main() -> Result<(), kmr_api::ffi::build::Error> {
    // The tests/*.c bodies are the tests' own; the binary links only `swing`.
    kmr_api::ffi::build::robot("XmBench").compile(&["src/swing.c", "tests/bodies.c"])
}
