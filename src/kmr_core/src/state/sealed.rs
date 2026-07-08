use crate::state::State;

use super::{Desired, JOINTS};

pub trait Slot: Sized + Copy + From<f32> {
    fn slot(desired: &mut Desired) -> &mut [Self; JOINTS];
    fn field(state: &State<JOINTS>) -> &[Self; JOINTS];
}
