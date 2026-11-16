/* Moves actuators 0 and 2 by 0.1 rad a tick, from the newest sample. */
#include "kmr.h"

void swing(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
    float q0, q2;
    if (kmr_q_at(state, 0, &q0) != KMR_OK || kmr_q_at(state, 2, &q2) != KMR_OK) {
        kmr_emergency_stop(state, "no sample to swing from");
        return;
    }
    kmr_set_q_at(state, q0 + 0.1f, 0);
    kmr_set_q_at(state, q2 + 0.1f, 2);
}
