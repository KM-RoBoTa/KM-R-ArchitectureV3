//! Compiles the C controllers for the robot `robot!` names in src/main.rs.

fn main() -> Result<(), kmr_api::ffi::c::build::Error> {
    // tests/*.c are the controllers of the tests; the binary links only `swing`.
    kmr_api::ffi::c::build::robot("XmBench").compile(&[
        "src/swing.c",
        "tests/mirror.c",
        "tests/twin.c",
        "tests/time.c",
        "tests/signal.c",
        "tests/sensors.c",
    ])
}
