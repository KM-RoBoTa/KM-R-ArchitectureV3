/* Controller of tests/signal.rs. */
#include "kmr.h"

/* At tick 3, asks for what its context says: 1 a shutdown, 2 an emergency stop, 0 nothing. */
void at_tick_3(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
    uint32_t level = *(const uint32_t *)ctx;
    if (kmr_tick(time) != 3) {
        return;
    }
    if (level == 1) {
        kmr_shutdown(state, "tick 3");
    } else if (level == 2) {
        kmr_emergency_stop(state, "tick 3");
    }
}
