//! One actuator in `robot!`, C sources built by build.rs.

kmr_api::robot!(model = "one.urdf");

kmr_ffi::foreign! { depth = kmr_api::HISTORY_DEPTH; fn body; }

fn main() {
    let _: kmr_ffi::CFn<N, { kmr_api::HISTORY_DEPTH }> = body;
}
