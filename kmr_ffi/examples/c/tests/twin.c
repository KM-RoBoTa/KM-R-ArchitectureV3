/* C bodies of tests/twin.rs; `twin` has a Rust twin there, step for step. */
#include <math.h>
#include <stddef.h>

#include "kmr_ffi.h"

/* The step added to q, by tick % 7. */
static const float STEP[7] = {0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};

/* Counts its calls in ctx when given; moves some actuators, leaves the others unwritten. */
int32_t twin(const kmr_read_view *read, kmr_write_buf *write, const kmr_time_view *time,
             void *ctx) {
    if (ctx != NULL) {
        *(uint32_t *)ctx += 1;
    }
    if (read->samples == 0) {
        return -1;
    }
    float step = STEP[time->tick % 7];
    for (unsigned i = 0; i < KMR_N; i++) {
        if ((time->tick + i) % 3 != 0) {
            kmr_set_q(write, i, read->slot[0].q[i] + step);
        }
    }
    if (time->tick % 5 == 0) {
        kmr_set_tau(write, 1, 0.25f);
    }
    if (read->samples > 1) {
        kmr_set_qd(write, 3, read->slot[0].q[3] - read->slot[1].q[3]);
    }
    return KMR_CONTINUE;
}

/* Writes nothing. */
int32_t silent(const kmr_read_view *read, kmr_write_buf *write, const kmr_time_view *time,
               void *ctx) {
    (void)read;
    (void)write;
    (void)time;
    (void)ctx;
    return KMR_CONTINUE;
}

/* Writes q[0] and q[2], leaves q[1] unwritten. */
int32_t skip_one(const kmr_read_view *read, kmr_write_buf *write, const kmr_time_view *time,
                 void *ctx) {
    (void)read;
    (void)time;
    (void)ctx;
    kmr_set_q(write, 0, 0.5f);
    kmr_set_q(write, 2, 0.5f);
    return KMR_CONTINUE;
}

/* Writes q[0] = the uint32_t ctx points to. */
int32_t ctx_value(const kmr_read_view *read, kmr_write_buf *write, const kmr_time_view *time,
                  void *ctx) {
    (void)read;
    (void)time;
    kmr_set_q(write, 0, (float)*(const uint32_t *)ctx);
    return KMR_CONTINUE;
}

/* Writes a NaN to q[0]. */
int32_t nan_q0(const kmr_read_view *read, kmr_write_buf *write, const kmr_time_view *time,
               void *ctx) {
    (void)read;
    (void)time;
    (void)ctx;
    kmr_set_q(write, 0, NAN);
    return KMR_CONTINUE;
}
