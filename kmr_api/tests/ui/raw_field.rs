//! No path leads from a `State` to a core field: state is written through setters only.

fn write(robot: &mut kmr_api::State<'_, 4>) {
    robot.desired.q[0] = kmr_api::Q(0.0);
}

fn main() {}
