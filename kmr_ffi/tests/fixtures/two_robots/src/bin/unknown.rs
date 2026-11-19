//! A function no compile call of build.rs defines.

kmr_api::robot!(model = "../wrong_n/one.urdf");

fn main() {
    std::hint::black_box(kmr_api::c!(nobody));
}
