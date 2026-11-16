/* Controllers of tests/twin.rs and tests/no_alloc.rs; `twin` has a Rust twin there, step for
 * step. */
#include <math.h>

#include "kmr.h"

/* The step added to q, by tick % 7. */
static const float STEP[7] = {0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};

/* Actuators 0 and 2, the group of the gains. */
static const size_t GAINS[2] = {0, 2};

/* Moves some actuators from the newest sample, leaves the others unwritten. */
void twin(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
    float q[KMR_N], q1[KMR_N];
    const float kd[2] = {1.0f, 2.0f};
    uint64_t tick = kmr_tick(time);
    size_t i;
    if (kmr_prev_q(state, 0, q) != KMR_OK) {
        return;
    }
    for (i = 0; i < KMR_N; ++i) {
        if ((tick + i) % 3 != 0) {
            kmr_set_q_at(state, q[i] + STEP[tick % 7], i);
        }
    }
    if (tick % 5 == 0) {
        kmr_set_tau_at(state, 0.25f, 1);
    }
    if (kmr_prev_q(state, 1, q1) == KMR_OK) {
        kmr_set_qd_at(state, q[3] - q1[3], 3);
    }
    if (tick % 4 == 0) {
        kmr_group_set_all_kd(state, GAINS, 2, kd);
    }
}

/* Writes nothing. */
void silent(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
}

/* Writes q[0] and q[2], leaves q[1] to whoever wrote it. */
void skip_one(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
    kmr_set_q_at(state, 0.5f, 0);
    kmr_set_q_at(state, 0.5f, 2);
}

/* Writes its context, a uint32_t, as q[0]. */
void ctx_value(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
    kmr_set_q_at(state, (float)*(const uint32_t *)ctx, 0);
}

/* Counts its calls in its context, a uint32_t, and writes the count as q[0]. */
void count_calls(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
    uint32_t *n = ctx;
    *n += 1;
    kmr_set_q_at(state, (float)*n, 0);
}

/* Writes NaN as q[0]. */
void nan_q0(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
    kmr_set_q_at(state, NAN, 0);
}
