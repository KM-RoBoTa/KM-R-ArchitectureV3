//! `State::group` is still `todo!()`, so the views are built here from the
//! private fields. Nothing in the public core API records a sample or reads
//! the desired buffer back, which bounds what can be observed from this crate:
//! the index logic, the gather helpers, and the empty-history behaviour.

use super::{GroupView, gather, try_gather};
use crate::{N, Q, Qd, State, StateError, Tau};

// A macro, not a fn: the view borrows `State` for the state's own lifetime
// (`&'a mut State<'a>`), so both owners have to live in the caller's frame.
macro_rules! view {
    ($name:ident, $indices:expr) => {
        let mut core = kmr_core::RobotState::<N>::default();
        let mut state = State::new(&mut core);
        #[allow(unused_mut)]
        let mut $name = GroupView {
            robot: &mut state,
            indices: $indices,
        };
    };
}

#[test]
fn gather_preserves_group_order() {
    let joints = [Q(0.0), Q(10.0), Q(20.0), Q(30.0)];

    assert_eq!(
        gather([3, 0, 2], |i| joints.get(i).copied()),
        Some([Q(30.0), Q(0.0), Q(20.0)])
    );
}

#[test]
fn gather_keeps_duplicated_indices() {
    let joints = [Q(0.0), Q(10.0), Q(20.0), Q(30.0)];

    assert_eq!(
        gather([1, 3, 3], |i| joints.get(i).copied()),
        Some([Q(10.0), Q(30.0), Q(30.0)])
    );
}

#[test]
fn gather_stops_at_the_first_miss() {
    let mut reads = 0;

    let gathered: Option<[Q; 3]> = gather([0, 1, 2], |i| {
        reads += 1;
        (i != 1).then_some(Q(0.0))
    });

    assert_eq!(gathered, None);
    assert_eq!(reads, 2);
}

#[test]
fn gather_of_an_empty_group_is_an_empty_array() {
    assert_eq!(gather([], |_| None::<Q>), Some([]));
}

#[test]
fn try_gather_preserves_group_order() {
    let joints = [Tau(0.0), Tau(1.0), Tau(2.0), Tau(3.0)];

    assert_eq!(
        try_gather([2, 0], |i| Ok(joints[i])),
        Ok([Tau(2.0), Tau(0.0)])
    );
}

#[test]
fn try_gather_returns_the_first_error_and_stops() {
    let mut reads = 0;

    let gathered: Result<[Qd; 3], StateError> = try_gather([0, 7, 9], |index| {
        reads += 1;
        match index {
            0 => Ok(Qd(0.0)),
            _ => Err(StateError::OutOfRange { index, len: N }),
        }
    });

    assert_eq!(gathered, Err(StateError::OutOfRange { index: 7, len: N }));
    assert_eq!(reads, 2);
}

#[test]
fn check_indices_accepts_any_order_within_range() {
    view!(group, [N - 1, 0, 0]);

    assert_eq!(group.check_indices(), Ok(()));
}

#[test]
fn check_indices_reports_the_first_out_of_range_index() {
    view!(group, [0, N + 2, N]);

    assert_eq!(
        group.check_indices(),
        Err(StateError::OutOfRange {
            index: N + 2,
            len: N
        })
    );
}

#[test]
fn current_reads_are_none_before_the_first_sample() {
    view!(group, [0, 2]);

    assert_eq!(group.q(), None);
    assert_eq!(group.position(), None);
    assert_eq!(group.qd(), None);
    assert_eq!(group.velocity(), None);
    assert_eq!(group.tau(), None);
    assert_eq!(group.torque(), None);
    assert_eq!(group.effort(), None);
}

#[test]
fn history_reads_propagate_the_cause() {
    view!(group, [0, 2]);

    let empty_history = StateError::HistoryValueOutOfRange {
        provided_depth: 1,
        max_depth: 0,
    };

    assert_eq!(group.prev_q(1), Err(empty_history));
    assert!(matches!(
        group.prev_qd(1),
        Err(StateError::HistoryValueOutOfRange { .. })
    ));
    assert!(matches!(
        group.prev_tau(1),
        Err(StateError::HistoryValueOutOfRange { .. })
    ));
}

#[test]
fn writes_within_range_are_accepted() {
    view!(group, [3, 1]);

    assert_eq!(group.set_all_q([Q(1.0), Q(2.0)]), Ok(()));
    assert_eq!(group.set_all_qd([Qd(1.0), Qd(2.0)]), Ok(()));
    assert_eq!(group.set_all_tau([Tau(1.0), Tau(2.0)]), Ok(()));
}

#[test]
fn writes_reject_an_out_of_range_index() {
    view!(group, [1, N]);

    let out_of_range = || StateError::OutOfRange { index: N, len: N };

    assert_eq!(group.set_all_q([Q(1.0), Q(2.0)]), Err(out_of_range()));
    assert_eq!(group.set_all_qd([Qd(1.0), Qd(2.0)]), Err(out_of_range()));
    assert_eq!(group.set_all_tau([Tau(1.0), Tau(2.0)]), Err(out_of_range()));
}
