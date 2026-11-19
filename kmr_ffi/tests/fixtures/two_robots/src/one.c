#include "kmr.h"

void one_c(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
    float q;
    if (kmr_q_at(state, KMR_N - 1, &q) == KMR_OK && kmr_elapsed(time) > 1.0 &&
        kmr_mic(sensors, 0) != NULL && ctx != NULL) {
        kmr_shutdown(state, "never");
    }
}
