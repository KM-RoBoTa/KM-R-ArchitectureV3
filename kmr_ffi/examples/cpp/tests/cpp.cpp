// C++ bodies of tests/cpp.rs.
#include <stdexcept>

#include "kmr_ffi.hpp"

// Writes q[0], then throws.
static std::int32_t throwing_body(kmr::In, kmr::Out out, const kmr_time_view &, void *) {
    out.set(0, kmr::Q{0.5f});
    throw std::runtime_error("controller failed");
}
KMR_CPP_CONTROLLER(throwing, throwing_body)

// Writes q[0], its non-throwing twin.
static std::int32_t calm_body(kmr::In, kmr::Out out, const kmr_time_view &, void *) {
    out.set(0, kmr::Q{0.5f});
    return KMR_CONTINUE;
}
KMR_CPP_CONTROLLER(calm, calm_body)

// Rebuilds the view into ctx, a kmr_read_view, through the typed accessors only; the tick
// flags, which have no typed accessor, from raw().
static std::int32_t typed_copy_body(kmr::In in, kmr::Out, const kmr_time_view &, void *ctx) {
    kmr_read_view *copy = static_cast<kmr_read_view *>(ctx);
    *copy = kmr_read_view{};
    copy->samples = in.samples();
    for (std::size_t i = 0; i < KMR_N; i++) {
        copy->age[i] = in.age(i);
    }
    for (std::size_t k = 0; k < KMR_HISTORY_DEPTH; k++) {
        kmr_slot &slot = copy->slot[k];
        slot.tick_flags = in.raw().slot[k].tick_flags;
        for (std::size_t i = 0; i < KMR_N; i++) {
            slot.q[i] = in.q(k, i).v;
            slot.qd[i] = in.qd(k, i).v;
            slot.tau[i] = in.tau(k, i).v;
            slot.kp[i] = in.kp(k, i).v;
            slot.kd[i] = in.kd(k, i).v;
            std::uint64_t bit = std::uint64_t{1} << i;
            slot.flags.stale |= in.stale(k, i) ? bit : 0;
            slot.flags.bus_error |= in.bus_error(k, i) ? bit : 0;
            slot.flags.offline |= in.offline(k, i) ? bit : 0;
            slot.flags.rejected |= in.rejected(k, i) ? bit : 0;
            slot.flags.joint_limit |= in.joint_limit(k, i) ? bit : 0;
            slot.flags.envelope |= in.envelope(k, i) ? bit : 0;
        }
    }
    return KMR_CONTINUE;
}
KMR_CPP_CONTROLLER(typed_copy, typed_copy_body)
