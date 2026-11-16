/* Controller of tests/mirror.rs. */
#include "kmr.h"

#define M (KMR_N + 1) /* every actuator and one past N */
#define D (KMR_HISTORY_DEPTH + 1) /* every depth and one past the History */

/* Every state read through the API, statuses as int32_t; mirrored as `Mirror` in
 * tests/mirror.rs. */
typedef struct {
    float q_at[M];
    float prev_q_at[D][M];
    float prev_kd_at[D][M];
    float prev_tau[KMR_HISTORY_DEPTH][KMR_N];
    float temperature[M];
    float group_q[2];
    float group_prev_qd[2];
    uint64_t stale[D];
    uint64_t offline[D];
    uint64_t telemetry_age[M];
    uint32_t age[M];
    uint32_t read_every[M];
    uint32_t sample_age;
    uint32_t group_sample_age;
    int32_t q_at_status[M];
    int32_t prev_q_at_status[D][M];
    int32_t prev_kd_at_status[D][M];
    int32_t prev_tau_status[D];
    int32_t faults_status[D];
    int32_t overrun[D];
    int32_t age_status[M];
    int32_t read_every_status[M];
    int32_t temperature_status[M];
    int32_t telemetry_age_status[M];
    int32_t stale_at[M];
    int32_t has_q;
    int32_t has_sample_age;
    int32_t group_q_ok;
    int32_t group_prev_qd_status;
    int32_t group_stale;
    int32_t group_sample_age_ok;
} mirror;

/* Actuators 3 and 1, a group out of order. */
static const size_t GROUP[2] = {3, 1};

/* Reads everything; ctx points at a reference to a mirror. */
void read_all(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
    mirror *out = *(mirror **)ctx;
    float row[KMR_N];
    size_t i, d;
    kmr_tick_faults f;
    for (i = 0; i < M; ++i) {
        out->q_at_status[i] = kmr_q_at(state, i, &out->q_at[i]);
        out->age_status[i] = kmr_age_at(state, i, &out->age[i]);
        out->read_every_status[i] = kmr_read_every(state, i, &out->read_every[i]);
        out->temperature_status[i] = kmr_temperature_at(state, i, &out->temperature[i]);
        out->telemetry_age_status[i] = kmr_telemetry_age_at(state, i, &out->telemetry_age[i]);
        out->stale_at[i] = kmr_stale_at(state, i);
        for (d = 0; d < D; ++d) {
            out->prev_q_at_status[d][i] = kmr_prev_q_at(state, i, d, &out->prev_q_at[d][i]);
            out->prev_kd_at_status[d][i] = kmr_prev_kd_at(state, i, d, &out->prev_kd_at[d][i]);
        }
    }
    for (d = 0; d < D; ++d) {
        out->prev_tau_status[d] = kmr_prev_tau(state, d, d < KMR_HISTORY_DEPTH ? out->prev_tau[d] : row);
        out->faults_status[d] = kmr_prev_faults(state, d, &f);
        if (out->faults_status[d] == KMR_OK) {
            out->stale[d] = kmr_faults_stale(&f);
            out->offline[d] = kmr_faults_offline(&f);
            out->overrun[d] = kmr_faults_overrun(&f);
        }
    }
    out->has_q = kmr_q(state, row);
    out->has_sample_age = kmr_sample_age(state, &out->sample_age);
    out->group_q_ok = kmr_group_q(state, GROUP, 2, out->group_q);
    out->group_prev_qd_status = kmr_group_prev_qd(state, GROUP, 2, 1, out->group_prev_qd);
    out->group_stale = kmr_group_stale(state, GROUP, 2);
    out->group_sample_age_ok = kmr_group_sample_age(state, GROUP, 2, &out->group_sample_age);
}
