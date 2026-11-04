//! The home state is open (SRS FR-2.7): `go_home` and `initial_state` do not build.

use kmr_api::{Q, State};

fn home<const N: usize>(s: &mut State<'_, N>) {
    s.go_home(90);
    let _: [Q; N] = s.initial_state();
    let _: [f32; N] = s.initial_state();
}

fn main() {}
