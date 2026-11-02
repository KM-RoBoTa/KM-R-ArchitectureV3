//! The home state is open (SRS FR-2.7): `go_home` and `initial_state` do not build.

use kmr_api::{N, Q, State};

fn home(s: &mut State) {
    s.go_home(90);
    let _: [Q; N] = s.initial_state();
    let _: [f32; 4] = s.initial_state();
}

fn main() {}
