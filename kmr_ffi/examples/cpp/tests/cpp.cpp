// C++ controllers of tests/cpp.rs.
#include <stdexcept>

#include "kmr.hpp"

// Writes q[0], then throws.
static void throwing(const kmr::Time &, kmr::State &state, const kmr::Sensors &) {
    state.set_q_at(0.5f, 0);
    throw std::runtime_error("controller failed");
}
KMR_CONTROLLER(throwing_cpp, throwing)

// Counts its calls in its context and writes the count as q[0].
static void counting(const kmr::Time &, kmr::State &state, const kmr::Sensors &,
                     std::uint32_t &count) {
    count += 1;
    state.set_q_at(static_cast<float>(count), 0);
}
KMR_CONTROLLER(counting_cpp, counting)

// What the methods answer; mirrored as `CppOut` in tests/cpp.rs.
struct CppOut {
    float q1;
    float group_tau[2];
    std::uint32_t age3;
    std::int32_t q1_status;
    std::int32_t past_n_status;
    std::int32_t deep_status;
    std::int32_t prev_qd_status;
    std::int32_t has_q;
    std::int32_t has_age3;
    std::int32_t age_past_n_status;
    std::int32_t has_faults;
    std::int32_t offline3;
    std::int32_t has_group_tau;
    std::int32_t late;
    std::int32_t has_mic;
};

// Reads through the methods into the CppOut its context points at.
static void read_methods(const kmr::Time &time, kmr::State &state, const kmr::Sensors &sensors,
                         CppOut *&out) {
    auto q1 = state.q_at(1);
    out->q1_status = q1.error();
    out->q1 = q1.value_or(-1.0f);
    out->past_n_status = state.q_at(KMR_N).error();
    out->deep_status = state.prev_kp_at(0, KMR_HISTORY_DEPTH).error();
    out->prev_qd_status = state.prev_qd(1).error();
    out->has_q = state.q().has_value();
    auto age = state.age_at(3);
    out->has_age3 = age && age->has_value();
    out->age3 = age && age->has_value() ? **age : 0;
    out->age_past_n_status = state.age_at(KMR_N).error();
    auto faults = state.faults();
    out->has_faults = faults.has_value();
    out->offline3 = faults && faults->offline().get(3);
    auto tau = state.group({2, 0}).tau();
    out->has_group_tau = tau.has_value();
    out->group_tau[0] = tau ? (*tau)[0] : 0.0f;
    out->group_tau[1] = tau ? (*tau)[1] : 0.0f;
    out->late = time.last_overrun().has_value();
    out->has_mic = sensors.mic(0).has_value();
}
KMR_CONTROLLER(read_methods_cpp, read_methods)
