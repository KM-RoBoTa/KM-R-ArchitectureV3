/* C bodies of tests/status.rs. */
#include "kmr_ffi.h"

/* Returns the int32_t ctx points to at tick 3, KMR_CONTINUE before and after. */
int32_t at_tick_3(const kmr_read_view *read, kmr_write_buf *write, const kmr_time_view *time,
                  void *ctx) {
    (void)read;
    (void)write;
    return time->tick == 3 ? *(const int32_t *)ctx : KMR_CONTINUE;
}
