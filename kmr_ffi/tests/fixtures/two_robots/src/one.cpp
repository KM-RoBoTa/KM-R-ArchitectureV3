#include <array>
#include <optional>

#include "kmr.hpp"

static void step(const kmr::Time &, kmr::State &state, const kmr::Sensors &) {
    std::optional<kmr::Values> q = state.q();
    std::array<float, KMR_N> next{};
    if (q) {
        next = *q;
    }
    state.set_all_q(next);
    if (auto g = state.group({0}).q()) {
        state.set_q_at((*g)[0], 0);
    }
}
KMR_CONTROLLER(one_cpp, step)
