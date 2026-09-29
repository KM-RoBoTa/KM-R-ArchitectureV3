//! `State::group` is still `todo!()`, so the views are built here from the
//! private fields. The normal core surface neither records a sample nor reads
//! the desired buffer back: the populated cases go through the core's
//! `test-util` hooks, enabled for this crate's tests only.

use super::{GroupView, gather, try_gather};
use crate::{N, Q, Qd, State, StateError, Tau};

// A macro, not a fn: the view borrows `State` for the state's own lifetime
// (`&'a mut State<'a>`), so both owners have to live in the caller's frame.
//
// The `on $core` form takes a core state owned by the test, which can then
// fill it before the view exists and inspect it once the view is dead.
macro_rules! view {
    ($name:ident, $indices:expr) => {
        let mut core = <kmr_core::RobotState>::default();
        view!($name, $indices, on core);
    };
    ($name:ident, $indices:expr, on $core:ident) => {
        let mut state = State::new(&mut $core);
        #[allow(unused_mut)]
        let mut $name = GroupView {
            robot: &mut state,
            indices: $indices,
        };
    };
}

// Distinct per joint and per field, so a value landing on the wrong joint or
// in the wrong buffer cannot go unnoticed.
fn ramp<T: From<f32>>(start: f32) -> [T; N] {
    std::array::from_fn(|joint| T::from(start + joint as f32))
}

fn staged_core() -> kmr_core::RobotState {
    let mut core = <kmr_core::RobotState>::default();
    core.set_all(ramp::<Q>(10.0));
    core.set_all(ramp::<Qd>(20.0));
    core.set_all(ramp::<Tau>(30.0));
    core
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

#[test]
fn writes_scatter_to_the_group_joints_only() {
    let mut core = staged_core();
    {
        view!(group, [N - 1, 1], on core);

        assert_eq!(group.set_all_q([Q(1.0), Q(2.0)]), Ok(()));
        assert_eq!(group.set_all_qd([Qd(3.0), Qd(4.0)]), Ok(()));
        assert_eq!(group.set_all_tau([Tau(5.0), Tau(6.0)]), Ok(()));
    }

    let mut q = ramp::<Q>(10.0);
    (q[N - 1], q[1]) = (Q(1.0), Q(2.0));
    let mut qd = ramp::<Qd>(20.0);
    (qd[N - 1], qd[1]) = (Qd(3.0), Qd(4.0));
    let mut tau = ramp::<Tau>(30.0);
    (tau[N - 1], tau[1]) = (Tau(5.0), Tau(6.0));

    assert_eq!(core.staged::<Q>(), q);
    assert_eq!(core.staged::<Qd>(), qd);
    assert_eq!(core.staged::<Tau>(), tau);
}

#[test]
fn a_duplicated_index_keeps_the_last_written_value() {
    let mut core = staged_core();
    {
        view!(group, [2, 2], on core);

        assert_eq!(group.set_all_q([Q(1.0), Q(2.0)]), Ok(()));
    }

    let mut q = ramp::<Q>(10.0);
    q[2] = Q(2.0);

    assert_eq!(core.staged::<Q>(), q);
}

#[test]
fn a_rejected_write_stages_nothing() {
    let mut core = staged_core();
    {
        // The valid index comes first: a non-atomic scatter would have staged
        // joint 1 before hitting the bad one.
        view!(group, [1, N], on core);

        assert!(group.set_all_q([Q(1.0), Q(2.0)]).is_err());
        assert!(group.set_all_qd([Qd(1.0), Qd(2.0)]).is_err());
        assert!(group.set_all_tau([Tau(1.0), Tau(2.0)]).is_err());
    }

    assert_eq!(core.staged::<Q>(), ramp::<Q>(10.0));
    assert_eq!(core.staged::<Qd>(), ramp::<Qd>(20.0));
    assert_eq!(core.staged::<Tau>(), ramp::<Tau>(30.0));
}

#[test]
fn current_reads_gather_the_newest_sample_in_group_order() {
    let mut core = <kmr_core::RobotState>::default();
    core.record_sample(ramp(10.0), ramp(20.0), ramp(30.0));
    core.record_sample(ramp(40.0), ramp(50.0), ramp(60.0));
    view!(group, [N - 1, 0, N - 1], on core);

    let last = (N - 1) as f32;

    assert_eq!(group.q(), Some([Q(40.0 + last), Q(40.0), Q(40.0 + last)]));
    assert_eq!(group.position(), group.q());
    assert_eq!(
        group.qd(),
        Some([Qd(50.0 + last), Qd(50.0), Qd(50.0 + last)])
    );
    assert_eq!(group.velocity(), group.qd());
    assert_eq!(
        group.tau(),
        Some([Tau(60.0 + last), Tau(60.0), Tau(60.0 + last)])
    );
    assert_eq!(group.torque(), group.tau());
    assert_eq!(group.effort(), group.tau());
}

#[test]
fn history_reads_gather_the_requested_depth_in_group_order() {
    let mut core = <kmr_core::RobotState>::default();
    core.record_sample(ramp(10.0), ramp(20.0), ramp(30.0));
    core.record_sample(ramp(40.0), ramp(50.0), ramp(60.0));
    view!(group, [2, 0], on core);

    assert_eq!(group.prev_q(0), Ok([Q(42.0), Q(40.0)]));
    assert_eq!(group.prev_q(1), Ok([Q(12.0), Q(10.0)]));
    assert_eq!(group.prev_qd(1), Ok([Qd(22.0), Qd(20.0)]));
    assert_eq!(group.prev_tau(1), Ok([Tau(32.0), Tau(30.0)]));
    assert_eq!(
        group.prev_q(2),
        Err(StateError::HistoryValueOutOfRange {
            provided_depth: 2,
            max_depth: 1,
        })
    );
}

#[test]
fn reads_on_a_populated_history_still_reject_an_out_of_range_index() {
    let mut core = <kmr_core::RobotState>::default();
    core.record_sample(ramp(10.0), ramp(20.0), ramp(30.0));
    view!(group, [0, N], on core);

    assert_eq!(group.q(), None);
    assert_eq!(
        group.prev_q(0),
        Err(StateError::OutOfRange { index: N, len: N })
    );
}
