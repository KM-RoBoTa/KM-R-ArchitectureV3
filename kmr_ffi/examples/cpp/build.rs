//! Compiles the C++ controllers for the robot `robot!` names in src/main.rs.

fn main() -> Result<(), kmr_api::ffi::c::build::Error> {
    // tests/*.cpp are the controllers of the tests; the binary links only `swing_cpp`.
    kmr_api::ffi::c::build::robot("XmBench").compile(&["src/swing.cpp", "tests/cpp.cpp"])
}
