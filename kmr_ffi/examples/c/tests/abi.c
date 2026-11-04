/* Probes of tests/abi.rs: the layout C sees, and named reads of a view Rust filled. */
#include <stddef.h>
#include <string.h>

#include "kmr_ffi.h"

/* Sizes, alignments and offsets in the order tests/abi.rs lists them; returns how many. */
size_t probe_layout(size_t *out, size_t cap) {
    const size_t v[] = {
        KMR_N,
        KMR_HISTORY_DEPTH,
        sizeof(kmr_actuator_flags),
        _Alignof(kmr_actuator_flags),
        offsetof(kmr_actuator_flags, stale),
        offsetof(kmr_actuator_flags, bus_error),
        offsetof(kmr_actuator_flags, offline),
        offsetof(kmr_actuator_flags, rejected),
        offsetof(kmr_actuator_flags, joint_limit),
        offsetof(kmr_actuator_flags, envelope),
        sizeof(kmr_slot),
        _Alignof(kmr_slot),
        offsetof(kmr_slot, q),
        offsetof(kmr_slot, qd),
        offsetof(kmr_slot, tau),
        offsetof(kmr_slot, kp),
        offsetof(kmr_slot, kd),
        offsetof(kmr_slot, flags),
        offsetof(kmr_slot, tick_flags),
        offsetof(kmr_slot, reserved),
        sizeof(kmr_read_view),
        _Alignof(kmr_read_view),
        offsetof(kmr_read_view, samples),
        offsetof(kmr_read_view, reserved),
        offsetof(kmr_read_view, age),
        offsetof(kmr_read_view, slot),
        sizeof(kmr_written),
        _Alignof(kmr_written),
        offsetof(kmr_written, q),
        offsetof(kmr_written, qd),
        offsetof(kmr_written, tau),
        offsetof(kmr_written, kp),
        offsetof(kmr_written, kd),
        sizeof(kmr_write_buf),
        _Alignof(kmr_write_buf),
        offsetof(kmr_write_buf, q),
        offsetof(kmr_write_buf, qd),
        offsetof(kmr_write_buf, tau),
        offsetof(kmr_write_buf, kp),
        offsetof(kmr_write_buf, kd),
        offsetof(kmr_write_buf, written),
        sizeof(kmr_time_view),
        _Alignof(kmr_time_view),
        offsetof(kmr_time_view, tick),
        offsetof(kmr_time_view, dt_ns),
        offsetof(kmr_time_view, elapsed_ns),
        offsetof(kmr_time_view, overrun_count),
        offsetof(kmr_time_view, last_overrun_ns),
        offsetof(kmr_time_view, accumulated_overrun_ns),
    };
    size_t n = sizeof v / sizeof v[0];
    for (size_t i = 0; i < n && i < cap; i++) {
        out[i] = v[i];
    }
    return n;
}

/* The bits of a float. */
static uint64_t bits(float f) {
    uint32_t u;
    memcpy(&u, &f, sizeof u);
    return u;
}

/* Reads named fields of the view and the clock into ctx, a uint64_t[8]. */
int32_t probe_read(const kmr_read_view *read, kmr_write_buf *write, const kmr_time_view *time,
                   void *ctx) {
    (void)write;
    uint64_t *out = ctx;
    out[0] = read->samples;
    out[1] = read->age[KMR_N - 1];
    out[2] = bits(read->slot[3].q[KMR_N - 1]);
    out[3] = bits(read->slot[1].kd[0]);
    out[4] = read->slot[2].flags.envelope;
    out[5] = read->slot[0].tick_flags;
    out[6] = time->elapsed_ns;
    out[7] = time->accumulated_overrun_ns;
    return KMR_CONTINUE;
}

/* Writes past N through the setters only. */
int32_t probe_past_n(const kmr_read_view *read, kmr_write_buf *write, const kmr_time_view *time,
                     void *ctx) {
    (void)read;
    (void)time;
    (void)ctx;
    kmr_set_q(write, KMR_N, 1.0f);
    kmr_set_tau(write, KMR_N + 60, 1.0f);
    kmr_set_kd(write, (unsigned)-1, 1.0f);
    return KMR_CONTINUE;
}

/* Writes q[1] and qd[0], then fails with -7. */
int32_t probe_fail(const kmr_read_view *read, kmr_write_buf *write, const kmr_time_view *time,
                   void *ctx) {
    (void)read;
    (void)time;
    (void)ctx;
    kmr_set_q(write, 1, 0.5f);
    kmr_set_qd(write, 0, 0.5f);
    return -7;
}
