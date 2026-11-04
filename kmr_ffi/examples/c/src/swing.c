/* Moves actuators 0 and 2 by 0.1 rad a tick, from the newest sample. */
#include "kmr_ffi.h"

int32_t swing(const kmr_read_view *read, kmr_write_buf *write, const kmr_time_view *time,
              void *ctx) {
    (void)time;
    (void)ctx;
    if (read->samples == 0) {
        return -1;
    }
    kmr_set_q(write, 0, read->slot[0].q[0] + 0.1f);
    kmr_set_q(write, 2, read->slot[0].q[2] + 0.1f);
    return KMR_CONTINUE;
}
