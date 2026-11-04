/* The foreign body of tick_ffi: reads nothing, writes nothing, continues (kmr_ffi SC-003). */
#include "kmr_ffi.h"

int32_t empty(const kmr_read_view *read, kmr_write_buf *write, const kmr_time_view *time,
              void *ctx) {
    (void)read;
    (void)write;
    (void)time;
    (void)ctx;
    return KMR_CONTINUE;
}
