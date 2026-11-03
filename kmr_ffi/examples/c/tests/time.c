/* C body of tests/time.rs. */
#include "kmr_ffi.h"

/* Copies the clock into ctx, a kmr_time_view, field by field. */
int32_t copy_time(const kmr_read_view *read, kmr_write_buf *write, const kmr_time_view *time,
                  void *ctx) {
    (void)read;
    (void)write;
    kmr_time_view *out = ctx;
    out->tick = time->tick;
    out->dt_ns = time->dt_ns;
    out->elapsed_ns = time->elapsed_ns;
    out->overrun_count = time->overrun_count;
    out->last_overrun_ns = time->last_overrun_ns;
    out->accumulated_overrun_ns = time->accumulated_overrun_ns;
    return KMR_CONTINUE;
}
