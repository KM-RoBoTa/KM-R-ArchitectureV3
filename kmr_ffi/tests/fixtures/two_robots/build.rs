//! Two groups: `src/one.c` and `src/one.cpp` for the robot of one actuator, `src/two.cpp` for
//! the robot of two. With `KMR_FFI_FIXTURE_DUPLICATE` set, a third group compiles `src/one.c`
//! again.

use kmr_api::ffi::c::build::{Error, model};

/// The descriptors of tests/fixtures/wrong_n.
const ONE: &str = "../wrong_n/one.urdf";
const TWO: &str = "../wrong_n/two.urdf";

fn main() -> Result<(), Error> {
    println!("cargo:rerun-if-env-changed=KMR_FFI_FIXTURE_DUPLICATE");
    model(ONE).compile(&["src/one.c", "src/one.cpp"])?;
    model(TWO).compile(&["src/two.cpp"])?;
    if std::env::var_os("KMR_FFI_FIXTURE_DUPLICATE").is_some() {
        model(TWO).compile(&["src/one.c"])?;
    }
    Ok(())
}
