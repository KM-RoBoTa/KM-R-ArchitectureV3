/* The C controller of tick_ffi: reads nothing, writes nothing (kmr_ffi SC-003). */
#include "kmr.h"

void empty(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
}
