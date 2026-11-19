//! The robot of one actuator with a function built for the robot of two.

kmr_api::robot!(model = "../wrong_n/one.urdf");

fn main() {
    std::hint::black_box(kmr_api::c!(two_cpp));
}
