//! Each function with the robot it was built for: builds and links.

mod one {
    kmr_api::robot!(model = "../wrong_n/one.urdf");

    pub fn controllers() {
        std::hint::black_box(kmr_api::c!(one_c));
        std::hint::black_box(kmr_api::c!(one_cpp));
    }
}

mod two {
    kmr_api::robot!(model = "../wrong_n/two.urdf");

    pub fn controllers() {
        std::hint::black_box(kmr_api::c!(two_cpp));
    }
}

fn main() {
    one::controllers();
    two::controllers();
}
