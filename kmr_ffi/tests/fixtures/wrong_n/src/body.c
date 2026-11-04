#include "kmr_ffi.h"

int32_t body(const kmr_read_view *read, kmr_write_buf *write, const kmr_time_view *time,
             void *ctx) {
    (void)read;
    (void)write;
    (void)time;
    (void)ctx;
    return 0;
}
