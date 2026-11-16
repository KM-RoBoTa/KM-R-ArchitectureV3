/* Controller of tests/time.rs. */
#include "kmr.h"

/* The clock as the kmr_* time functions give it; mirrored as `TimeOut` in tests/time.rs. */
typedef struct {
    uint64_t tick;
    uint64_t overrun_count;
    double elapsed;
    double dt;
    double last_overrun;
    double accumulated_overrun;
    uint32_t late;
    uint32_t reserved;
} time_out;

/* Copies the clock through the API; ctx points at a reference to a time_out. */
void copy_time(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
    time_out *out = *(time_out **)ctx;
    out->tick = kmr_tick(time);
    out->overrun_count = kmr_overrun_count(time);
    out->elapsed = kmr_elapsed(time);
    out->dt = kmr_dt(time);
    out->last_overrun = 0.0;
    out->late = kmr_last_overrun(time, &out->last_overrun);
    out->accumulated_overrun = kmr_accumulated_overrun(time);
}
