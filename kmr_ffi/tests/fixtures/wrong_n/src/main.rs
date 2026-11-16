//! One actuator in `robot!`, C sources built by build.rs.

kmr_api::robot!(model = "one.urdf");

fn main() {
    let _ = kmr_api::c!(body);
}
