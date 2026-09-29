use std::fmt::Debug;

use crate::{
    RobotState,
    error::StateError,
    state::{Desired, HISTORY_DEPTH, History, JOINTS, Q, Qd, State, StateField, Tau, sealed::Slot},
};
use rstest::rstest;

// `RobotState::default()` routes through `home_state()` which is `todo!()`, so we
// build the test fixtures directly from zeroed fields instead. The plumbing under
// test (slot routing, history reads, desired writes) is independent of the home
// state, so a zeroed start is a valid, panic-free baseline.
fn create_test_robot() -> RobotState {
    RobotState {
        desired: zeroed_desired(),
        history: History::default(),
    }
}

fn zeroed_desired() -> Desired {
    Desired {
        q: [Q(0.0); JOINTS],
        qd: [Qd(0.0); JOINTS],
        tau: [Tau(0.0); JOINTS],
    }
}

fn zeroed_state() -> State<JOINTS> {
    State {
        q: [Q(0.0); JOINTS],
        qd: [Qd(0.0); JOINTS],
        tau: [Tau(0.0); JOINTS],
    }
}

#[rstest]
#[case(Q(2.0))]
#[case(Qd(2.0))]
#[case(Tau(2.0))]
fn write_single_desired_state<T: StateField + PartialEq + core::fmt::Debug>(#[case] value: T) {
    let mut robot = create_test_robot();

    let q_i: usize = 1;
    let result = robot.set_at(value, q_i);

    assert_eq!(result, Ok(()));
    assert_eq!(<T as Slot>::slot(&mut robot.desired)[q_i], value);
}

#[rstest]
#[case(Q(2.0))]
#[case(Qd(2.0))]
#[case(Tau(2.0))]
fn out_of_range_write_single_desired(#[case] value: impl StateField) {
    let mut robot = create_test_robot();
    let index: usize = JOINTS + 1; // guarantee OOR

    assert_eq!(
        robot.set_at(value, index),
        Err(StateError::OutOfRange {
            index,
            len: crate::state::JOINTS
        }),
    );
}

#[test]
fn read_single_current_q() {
    let mut robot = create_test_robot();
    let mut state = zeroed_state();
    state.q = [Q(10.0), Q(11.0), Q(12.0), Q(13.0)];
    robot.history.buf.push_back(state);

    let read_value: &Q = robot.current_at(1).unwrap();

    assert_eq!(*read_value, Q(11.0));
}

#[test]
fn read_single_current_qd() {
    let mut robot = create_test_robot();
    let mut state = zeroed_state();
    state.qd = [Qd(10.0), Qd(11.0), Qd(12.0), Qd(13.0)];
    robot.history.buf.push_back(state);

    let read_value: &Qd = robot.current_at(1).unwrap();

    assert_eq!(*read_value, Qd(11.0));
}

#[test]
fn read_single_current_tau() {
    let mut robot = create_test_robot();
    let mut state = zeroed_state();
    state.tau = [Tau(10.0), Tau(11.0), Tau(12.0), Tau(13.0)];
    robot.history.buf.push_back(state);

    let read_value: &Tau = robot.current_at(1).unwrap();

    assert_eq!(*read_value, Tau(11.0));
}

#[test]
fn read_single_previous_q() {
    let mut robot = create_test_robot();

    // depth 1 = one step back
    let mut prev = zeroed_state();
    prev.q = [Q(10.0), Q(11.0), Q(12.0), Q(13.0)];
    robot.history.buf.push_back(prev);
    // depth 0 = newest
    let mut newest = zeroed_state();
    newest.q = [Q(20.0), Q(21.0), Q(22.0), Q(23.0)];
    robot.history.buf.push_back(newest);

    let read_value: &Q = robot.prev_at(1, 1).unwrap();

    assert_eq!(*read_value, Q(11.0));
}

#[test]
fn read_single_previous_qd() {
    let mut robot = create_test_robot();

    let mut prev = zeroed_state();
    prev.qd = [Qd(10.0), Qd(11.0), Qd(12.0), Qd(13.0)];
    robot.history.buf.push_back(prev);
    let mut newest = zeroed_state();
    newest.qd = [Qd(20.0), Qd(21.0), Qd(22.0), Qd(23.0)];
    robot.history.buf.push_back(newest);

    let read_value: &Qd = robot.prev_at(1, 1).unwrap();

    assert_eq!(*read_value, Qd(11.0));
}

#[test]
fn read_single_previous_tau() {
    let mut robot = create_test_robot();

    let mut prev = zeroed_state();
    prev.tau = [Tau(10.0), Tau(11.0), Tau(12.0), Tau(13.0)];
    robot.history.buf.push_back(prev);
    let mut newest = zeroed_state();
    newest.tau = [Tau(20.0), Tau(21.0), Tau(22.0), Tau(23.0)];
    robot.history.buf.push_back(newest);

    let read_value: &Tau = robot.prev_at(1, 1).unwrap();

    assert_eq!(*read_value, Tau(11.0));
}

#[test]
fn out_of_range_read_single_current_q() {
    let mut robot = create_test_robot();
    robot.history.buf.push_back(zeroed_state());

    let index: usize = usize::MAX;

    assert_eq!(
        robot.current_at::<Q>(index),
        Err(StateError::OutOfRange {
            index,
            len: crate::state::JOINTS
        }),
    );
}

#[test]
fn out_of_range_read_single_current_qd() {
    let mut robot = create_test_robot();
    robot.history.buf.push_back(zeroed_state());

    let index: usize = usize::MAX;

    assert_eq!(
        robot.current_at::<Qd>(index),
        Err(StateError::OutOfRange {
            index,
            len: crate::state::JOINTS
        }),
    );
}

#[test]
fn out_of_range_read_single_current_tau() {
    let mut robot = create_test_robot();
    robot.history.buf.push_back(zeroed_state());

    let index: usize = usize::MAX;

    assert_eq!(
        robot.current_at::<Tau>(index),
        Err(StateError::OutOfRange {
            index,
            len: crate::state::JOINTS
        }),
    );
}

#[rstest]
#[case([Q(1_f32), Q(2_f32), Q(3_f32), Q(4_f32)])]
#[case([Qd(1_f32), Qd(2_f32), Qd(3_f32), Qd(4_f32)])]
#[case([Tau(1_f32), Tau(2_f32), Tau(3_f32), Tau(4_f32)])]
fn write_all_desired<T: StateField + PartialEq + Debug>(#[case] desired: [T; JOINTS]) {
    let mut robot = create_test_robot();

    robot.set_all(desired);

    assert_eq!(*<T as Slot>::slot(&mut robot.desired), desired);
}

#[test]
fn write_all_desired_at_the_same_time() {
    let mut robot = create_test_robot();

    let desired_q = [Q(1.0), Q(2.0), Q(3.0), Q(4.0)];
    let desired_qd = [Qd(1.0), Qd(2.0), Qd(3.0), Qd(4.0)];
    let desired_tau = [Tau(1.0), Tau(2.0), Tau(3.0), Tau(4.0)];

    robot.set_all(desired_q);
    robot.set_all(desired_qd);
    robot.set_all(desired_tau);

    assert_eq!(robot.desired.q, desired_q);
    assert_eq!(robot.desired.qd, desired_qd);
    assert_eq!(robot.desired.tau, desired_tau);
}

// Copy the current value at `index` into a desired array, write it, and confirm
// the desired slot matches what was written at that index.
#[test]
fn write_some_desired_q() {
    let mut robot = create_test_robot();
    let mut state = zeroed_state();
    state.q = [Q(5.0), Q(6.0), Q(7.0), Q(8.0)];
    robot.history.buf.push_back(state);

    let index = 1;
    let current = *robot.current_at::<Q>(index).unwrap();
    let desired_q = [Q(1.0), current, Q(3.0), Q(4.0)];

    robot.set_all(desired_q);

    assert_eq!(robot.desired.q, desired_q);
    assert_eq!(robot.desired.q[index], current);
}

#[test]
fn write_some_desired_qd() {
    let mut robot = create_test_robot();
    let mut state = zeroed_state();
    state.qd = [Qd(5.0), Qd(6.0), Qd(7.0), Qd(8.0)];
    robot.history.buf.push_back(state);

    let index = 1;
    let current = *robot.current_at::<Qd>(index).unwrap();
    let desired_qd = [Qd(1.0), current, Qd(3.0), Qd(4.0)];

    robot.set_all(desired_qd);

    assert_eq!(robot.desired.qd, desired_qd);
    assert_eq!(robot.desired.qd[index], current);
}

#[test]
fn write_some_desired_tau() {
    let mut robot = create_test_robot();
    let mut state = zeroed_state();
    state.tau = [Tau(5.0), Tau(6.0), Tau(7.0), Tau(8.0)];
    robot.history.buf.push_back(state);

    let index = 1;
    let current = *robot.current_at::<Tau>(index).unwrap();
    let desired_tau = [Tau(1.0), current, Tau(3.0), Tau(4.0)];

    robot.set_all(desired_tau);

    assert_eq!(robot.desired.tau, desired_tau);
    assert_eq!(robot.desired.tau[index], current);
}

#[test]
fn read_all_current_q() {
    let mut robot = create_test_robot();
    let q = [Q(1.0), Q(2.0), Q(3.0), Q(4.0)];
    let mut state = zeroed_state();
    state.q = q;
    robot.history.buf.push_back(state);

    let read_value = robot.current::<Q>().unwrap();

    assert_eq!(*read_value, q);
}

#[test]
fn read_all_current_qd() {
    let mut robot = create_test_robot();
    let qd = [Qd(1.0), Qd(2.0), Qd(3.0), Qd(4.0)];
    let mut state = zeroed_state();
    state.qd = qd;
    robot.history.buf.push_back(state);

    let read_value = robot.current::<Qd>().unwrap();

    assert_eq!(*read_value, qd);
}

#[test]
fn read_all_current_tau() {
    let mut robot = create_test_robot();
    let tau = [Tau(1.0), Tau(2.0), Tau(3.0), Tau(4.0)];
    let mut state = zeroed_state();
    state.tau = tau;
    robot.history.buf.push_back(state);

    let read_value = robot.current::<Tau>().unwrap();

    assert_eq!(*read_value, tau);
}

#[rstest]
#[case([Q(1_f32), Q(2_f32), Q(3_f32), Q(4_f32)])]
#[case([Qd(1_f32), Qd(2_f32), Qd(3_f32), Qd(4_f32)])]
#[case([Tau(1_f32), Tau(2_f32), Tau(3_f32), Tau(4_f32)])]
fn write_all_desired_with_closure<T: StateField + PartialEq + Debug>(#[case] desired: [T; JOINTS]) {
    let mut robot = create_test_robot();

    robot.set_all(core::array::from_fn(|i| T::from(i as f32 + 1_f32)));

    assert_eq!(*<T as Slot>::slot(&mut robot.desired), desired);
}

// #[rstest]
// fn write_desired_q_to_group() {
//     let mut robot = create_test_robot();
//
//     let value = 8_f32;
//     let q_i: usize = 1;
//     let result = robot.group_example.set_state_at(Q(value), q_i);
//
//     assert_eq!(result, Ok(()));
//     assert_eq!(robot.desired.q[q_i], Q(value));
// }

// --- slot() routing ---

#[rstest]
#[case(Q(1_f32))]
#[case(Qd(1_f32))]
#[case(Tau(1_f32))]
fn generic_slot_routes_to_corresponding_generic_array<T: StateField + PartialEq + Debug>(
    #[case] desired: T,
) {
    let mut desired_default = zeroed_desired();

    let slot = <T as Slot>::slot(&mut desired_default);
    slot[0] = desired;

    assert_eq!(<T as Slot>::slot(&mut desired_default)[0], desired);
}

#[test]
fn slot_q_routes_to_qd_array() {
    let mut desired = zeroed_desired();

    let slot: &mut [Q; JOINTS] = <Q as Slot>::slot(&mut desired);
    slot[2] = Q(7.0);

    assert_eq!(desired.q[2], Q(7.0));
    assert_eq!(desired.qd, [Qd(0.0); JOINTS]);
    assert_eq!(desired.tau, [Tau(0.0); JOINTS]);
}

#[test]
fn slot_qd_routes_to_qd_array() {
    let mut desired = zeroed_desired();

    let slot: &mut [Qd; JOINTS] = <Qd as Slot>::slot(&mut desired);
    slot[2] = Qd(7.0);

    assert_eq!(desired.qd[2], Qd(7.0));
    assert_eq!(desired.q, [Q(0.0); JOINTS]);
    assert_eq!(desired.tau, [Tau(0.0); JOINTS]);
}

#[test]
fn slot_tau_routes_to_tau_array() {
    let mut desired = zeroed_desired();

    let slot: &mut [Tau; JOINTS] = <Tau as Slot>::slot(&mut desired);
    slot[JOINTS - 1] = Tau(7.0);

    assert_eq!(desired.tau[JOINTS - 1], Tau(7.0));
    assert_eq!(desired.q, [Q(0.0); JOINTS]);
    assert_eq!(desired.qd, [Qd(0.0); JOINTS]);
}

#[test]
fn slot_returns_full_length_array() {
    let mut desired = zeroed_desired();

    assert_eq!(Q::slot(&mut desired).len(), JOINTS);
    assert_eq!(Qd::slot(&mut desired).len(), JOINTS);
    assert_eq!(Tau::slot(&mut desired).len(), JOINTS);
}

#[test]
fn slot_mutation_persists_after_borrow_ends() {
    let mut desired = zeroed_desired();

    for i in 0..JOINTS {
        <Q as Slot>::slot(&mut desired)[i] = Q(i as f32);
    }

    assert_eq!(desired.q, [Q(0.0), Q(1.0), Q(2.0), Q(3.0)]);
}

// The two tests below guard the history depth against the joint count: the
// depth used to be fed `JOINTS` and only worked because both consts were 4.

#[test]
fn default_history_keeps_history_depth_ticks() {
    let mut robot = create_test_robot();

    for tick in 0..HISTORY_DEPTH + 3 {
        let mut state = zeroed_state();
        state.q[0] = Q(tick as f32);
        robot.history.buf.push_back(state);
    }

    assert_eq!(robot.history.buf.len(), HISTORY_DEPTH);
    assert_eq!(
        robot.prev_at::<Q>(HISTORY_DEPTH - 1, 0),
        Ok(&Q(3.0)),
        "the oldest retained tick is the 4th one pushed"
    );
    assert_eq!(
        robot.prev::<Q>(HISTORY_DEPTH),
        Err(StateError::HistoryValueOutOfRange {
            provided_depth: HISTORY_DEPTH,
            max_depth: HISTORY_DEPTH - 1,
        }),
    );
}

#[test]
fn history_depth_is_independent_of_the_joint_count() {
    const DEPTH: usize = JOINTS + 5;
    let mut robot = RobotState::<DEPTH> {
        desired: zeroed_desired(),
        history: History::default(),
    };

    for _ in 0..DEPTH + 1 {
        robot.history.buf.push_back(zeroed_state());
    }

    assert_eq!(robot.history.buf.len(), DEPTH);
    assert_eq!(robot.set_at(Q(1.0), JOINTS - 1), Ok(()));
    assert_eq!(robot.prev::<Q>(DEPTH - 1), Ok(&[Q(0.0); JOINTS]));
}
