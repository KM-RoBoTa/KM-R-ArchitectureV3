#include "kmr.h"

void body(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
    float q;
    if (kmr_q_at(state, 0, &q) == KMR_OK && kmr_elapsed(time) > 1.0 && kmr_mic(sensors, 0) != NULL &&
        ctx != NULL) {
        kmr_shutdown(state, "never");
    }
}
