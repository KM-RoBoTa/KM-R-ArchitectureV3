// The clap controller of src/main.rs: once home, a clap runs both snakes' sines for 2 s.
// Rust keeps the builder, the buses, the actuators and the HOME ramp (examples/clap_cpp.rs).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "kmr.hpp"

namespace {

constexpr double HOME_TIME = 3.0;   // s, the time HOME takes to ramp to the centre
constexpr double TIME_LIMIT = 60.0; // s, the run shuts down after this
constexpr std::size_t AX_LEN = 12;  // the AX-12A snake: the URDF's first actuators
constexpr std::size_t RX_LEN = KMR_N - AX_LEN; // the RX-28 snake: every actuator after them
constexpr float CENTER = 0.0f;      // rad: raw 512 on both models
constexpr float AMPLITUDE = 0.15f;  // rad, the RX-28 sine; the AX-12A sine has twice it
constexpr int PEAK = 1000;          // a microphone sample louder than this is a clap
constexpr double MOVE_FOR = 2.0;    // s, one period of the 0.5 Hz sine
constexpr double TAU = 6.283185307179586;

// `K` consecutive actuator indices from `first`.
template <std::size_t K>
constexpr std::array<std::size_t, K> from(std::size_t first) {
    std::array<std::size_t, K> indices{};
    for (std::size_t i = 0; i < K; ++i) {
        indices[i] = first + i;
    }
    return indices;
}

constexpr auto AX = from<AX_LEN>(0);
constexpr auto RX = from<RX_LEN>(AX_LEN);

// When the last clap's motion started: the context, mirrored as `Clap` in examples/clap_cpp.rs.
struct Clap {
    double since;
    bool clapped;
};

float sine(float amplitude, double t) {
    return CENTER + amplitude * static_cast<float>(std::sin(TAU * 0.5 * t));
}

// The microphone's loudest sample this tick; 0 without a microphone or a sample.
int loudest(const kmr::Sensors &sensors) {
    int peak = 0;
    if (auto mic = sensors.mic(0)) {
        for (std::int16_t s : mic->samples()) {
            peak = std::max(peak, std::abs(static_cast<int>(s)));
        }
    }
    return peak;
}

void move_on_clap(const kmr::Time &time, kmr::State &state, const kmr::Sensors &sensors,
                  Clap &clap) {
    double now = time.elapsed();
    if (now > TIME_LIMIT) {
        state.shutdown("time limit");
    }
    if (now < HOME_TIME) {
        return;
    }
    // A peak heard while moving is ignored (the servos are loud): one clap, one motion.
    bool moving = clap.clapped && now - clap.since < MOVE_FOR;
    int peak = loudest(sensors);
    if (!moving && peak > PEAK) {
        std::printf("clap at %.2f s: peak %d > %d, both snakes move for %.0f s\n", now, peak,
                    PEAK, MOVE_FOR);
        clap.since = now;
        clap.clapped = true;
        moving = true;
    }

    double t = moving ? now - clap.since : 0.0;
    std::array<float, AX_LEN> ax;
    std::array<float, RX_LEN> rx;
    ax.fill(moving ? sine(2.0f * AMPLITUDE, t) : CENTER);
    rx.fill(moving ? sine(AMPLITUDE, t) : CENTER);
    if (state.group(AX).set_all_q(ax) != KMR_OK) {
        state.shutdown("AX-12A group write failed");
    }
    if (state.group(RX).set_all_q(rx) != KMR_OK) {
        state.shutdown("RX-28 group write failed");
    }
}

} // namespace

KMR_CONTROLLER(move_on_clap_cpp, move_on_clap)
