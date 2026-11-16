// Moves actuators 0 and 2 by 0.1 rad a tick, from the newest sample.
#include "kmr.hpp"

static void swing(const kmr::Time &, kmr::State &state, const kmr::Sensors &) {
    auto q0 = state.q_at(0);
    auto q2 = state.q_at(2);
    if (!q0 || !q2) {
        state.emergency_stop("no sample to swing from");
        return;
    }
    state.set_q_at(*q0 + 0.1f, 0);
    state.set_q_at(*q2 + 0.1f, 2);
}

KMR_CONTROLLER(swing_cpp, swing)
