/* C body of tests/history.rs. */
#include "kmr_ffi.h"

/* What `copy_history` hands back: the view as C read it and the sample age C derives. */
typedef struct {
    kmr_read_view view;
    uint32_t sample_age;
} history_out;

/* Copies the view into ctx, a history_out, and derives the age of the oldest sample. */
int32_t copy_history(const kmr_read_view *read, kmr_write_buf *write, const kmr_time_view *time,
                     void *ctx) {
    (void)write;
    (void)time;
    history_out *out = ctx;
    out->view = *read;
    uint32_t oldest = 0;
    for (unsigned i = 0; i < KMR_N; i++) {
        if (read->age[i] == KMR_AGE_NONE) {
            oldest = KMR_AGE_NONE;
            break;
        }
        if (read->age[i] > oldest) {
            oldest = read->age[i];
        }
    }
    out->sample_age = oldest;
    return KMR_CONTINUE;
}
