//! Compiles the C bodies for the robot `robot!` names in src/main.rs.

fn main() -> Result<(), kmr_api::ffi::build::Error> {
    // tests/*.c are the C bodies of the tests; the binary links only `swing`.
    kmr_api::ffi::build::robot("XmBench").compile(&[
        "src/swing.c",
        "tests/abi.c",
        "tests/twin.c",
        "tests/status.c",
        "tests/history.c",
    ])
}
