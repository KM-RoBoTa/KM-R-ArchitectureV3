// Moves actuators 0 and 2 by 0.1 rad a tick, from the newest sample.
#include "kmr_ffi.hpp"

static std::int32_t swing_body(kmr::In in, kmr::Out out, const kmr_time_view &, void *) {
    if (in.samples() == 0) {
        return -1;
    }
    out.set(0, kmr::Q{in.q(0, 0).v + 0.1f});
    out.set(2, kmr::Q{in.q(0, 2).v + 0.1f});
    return KMR_CONTINUE;
}

KMR_CPP_CONTROLLER(swing, swing_body)
