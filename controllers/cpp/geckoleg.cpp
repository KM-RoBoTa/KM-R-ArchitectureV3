// The GeckoLeg gait of examples/geckoleg.rs, its kinematics done by Pinocchio on the
// model of models/geckoleg.urdf, which build.rs embeds as GECKOLEG_URDF. Rust keeps
// the builder, the bus and the actuators (examples/geckoleg_pinocchio.rs).
//
// What replaced which Rust function:
// - `control_points`, `bezier`, `foot`: a straight port on Eigen::Vector3d.
// - the DH table: pinocchio::urdf::buildModelFromXML, once. The URDF's joint frames are the DH
//   chain, so Pinocchio's joint angle is the DH theta (q = motor + OFFSET) and its world frame
//   is the Rust "hip reference".
// - `forward_kinematics`, `origin`: pinocchio::forwardKinematics, then
//   pinocchio::updateFramePlacement and data.oMf[foot].translation().
// - `jacobian`, `solve`: pinocchio::computeFrameJacobian in LOCAL_WORLD_ALIGNED, its three linear
//   rows, and the damped least-squares step of Pinocchio's inverse-kinematics example. It is the
//   true geometric Jacobian: the C++ original's z_prev = H * (0,0,1,1), kept in Rust, is not, so
//   the goals converge to the same posture with slightly different transients.
// - `theta + dtheta`: pinocchio::integrate.
// - `cycle_stats`: `check_gait`, on Init, against the limits Rust gives the actuators.
// - `walk`: `walk`.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "geckoleg_urdf.hpp"
#include "kmr.hpp"

namespace {

using Vec3 = Eigen::Vector3d;
// Motor angles, rad, in kmr's joint order.
using Motors = std::array<double, 3>;

constexpr double PI = 3.14159265358979323846;
constexpr double PROGRAM_S = 15.0; // run length in model seconds (C++ `programDuration`)
constexpr int IK_ITERATIONS = 5;   // Newton steps per tick at most
// DECISION: a step stops under 1 µm of foot error, far below what the leg can resolve.
constexpr double IK_TOLERANCE = 1e-6;
// DECISION: λ = 1e-6 m², the damping of Pinocchio's example: J Jᵀ stays invertible with the leg
// stretched, and away from it the converged goal is the undamped one.
constexpr double IK_DAMPING = 1e-6;

// Gait (C++ `global.config` + `BaseTrajectory`).
constexpr double TG = 2.0;
constexpr double DUTY = 0.7;
constexpr double LX = 0.20;
constexpr double R1 = 0.08;
constexpr double R2 = 0.08;
constexpr double PHI1 = 0.0;
constexpr double PHI2 = 0.0;
// The foot at all-zero motors, global frame.
const Vec3 FOOT0(-0.107, -0.089, -0.217);
// Motor 0 is DH theta = OFFSET (C++ `Controller`), per joint hip, knee, ankle.
constexpr Motors OFFSET = {-PI / 2.0, 0.52, -1.447};

// kmr's indices of the hip, knee and ankle: the URDF's actuator order (kmr.hpp names none).
constexpr std::array<std::size_t, 3> JOINTS = {0, 1, 2};
// The same joints in the URDF, by which Pinocchio's q order is looked up.
constexpr std::array<const char *, 3> NAMES = {"hip", "knee", "ankle"};

// The gait's state between ticks: the context of walk_cpp, mirrored as `Gait` in
// examples/geckoleg_pinocchio/leg.rs.
struct Gait {
    double speed;      // model time per wall second
    Motors prev;       // last goals: where the next Newton steps start
    double next_print; // wall time of the next printout, s
};

// The joint limits the actuators get: the context of check_gait_cpp, `Limits` in
// examples/geckoleg_pinocchio/leg.rs.
struct Limits {
    Motors lo;
    Motors hi;
    double peak; // rad/s at SPEED 1
};

static_assert(std::is_standard_layout_v<Gait> && sizeof(Gait) == 5 * sizeof(double));
static_assert(std::is_standard_layout_v<Limits> && sizeof(Limits) == 7 * sizeof(double));

// Bezier control points P0..P3 (`WalkingTrajectory` constructor), global frame.
std::array<Vec3, 4> control_points() {
    const Vec3 p0 = FOOT0 + Vec3(LX / 2.0, 0.0, 0.0);
    const Vec3 p3 = FOOT0 - Vec3(LX / 2.0, 0.0, 0.0);
    const Vec3 p1(p0.x() + (p3.x() - p0.x()) / 3.0, FOOT0.y() + R1 * std::sin(PHI1),
                  FOOT0.z() + R1 * std::cos(PHI1));
    const Vec3 p2(p0.x() + (p3.x() - p0.x()) * 2.0 / 3.0, FOOT0.y() + R2 * std::sin(PHI2),
                  FOOT0.z() + R2 * std::cos(PHI2));
    return {p0, p1, p2, p3};
}

Vec3 bezier(const std::array<Vec3, 4> &p, double t) {
    const double u = 1.0 - t;
    return u * u * u * p[3] + 3.0 * u * u * t * p[2] + 3.0 * u * t * t * p[1] + t * t * t * p[0];
}

// Foot position in the global frame at model time `tau` (forward gait).
Vec3 foot(double tau) {
    static const std::array<Vec3, 4> p = control_points();
    const double stance = DUTY * TG;
    const double swing = TG - stance;
    // The C++ starts in stance at t = 0.5.
    double m = std::fmod(tau + 0.5 * stance, TG);
    if (m < 0.0) {
        m += TG;
    }
    if (m < stance) {
        return p[0] + (p[3] - p[0]) * (m / stance);
    }
    return bezier(p, (m - stance) / swing);
}

// Pinocchio's model of the leg and every buffer a tick needs, allocated once.
struct Kinematics {
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex foot;
    std::array<int, 3> idx_q;      // Pinocchio's q index of each kmr joint
    Eigen::VectorXd q;             // DH angles of the step in progress
    Eigen::VectorXd q_next;        // pinocchio::integrate's output
    pinocchio::Data::Matrix6x jac; // the foot frame's 6 x nv Jacobian

    explicit Kinematics(pinocchio::Model built)
        : model(std::move(built)), data(model), foot(model.getFrameId("foot", pinocchio::BODY)),
          q(model.nq), q_next(model.nq), jac(6, model.nv) {
        if (model.nq != 3 || model.nv != 3 || foot >= model.frames.size()) {
            throw std::runtime_error("geckoleg.urdf: expected 3 revolute joints and a foot link");
        }
        for (std::size_t k = 0; k < NAMES.size(); ++k) {
            if (!model.existJointName(NAMES[k])) {
                throw std::runtime_error("geckoleg.urdf: a joint of the leg is missing");
            }
            idx_q[k] = model.idx_qs[model.getJointId(NAMES[k])];
        }
    }
};

pinocchio::Model leg_model() {
    pinocchio::Model model;
    pinocchio::urdf::buildModelFromXML(GECKOLEG_URDF, model);
    return model;
}

// The leg's kinematics, built on first use: by check_gait, at startup.
Kinematics &kinematics() {
    // DECISION: thread_local rather than a plain static: robots on two threads (parallel tests)
    // never share one Data, and kmr calls a robot's inline controllers on its loop thread.
    thread_local Kinematics kin(leg_model());
    return kin;
}

// `getInverseKinematics`: Newton steps from the motor angles `from` to the global foot `goal`.
Motors inverse_kinematics(Kinematics &kin, const Motors &from, const Vec3 &goal) {
    // Global to hip reference: the transpose of the hip-to-global (0,0,-1; -1,0,0; 0,1,0).
    const Vec3 target(-goal.y(), goal.z(), -goal.x());
    for (std::size_t k = 0; k < JOINTS.size(); ++k) {
        kin.q[kin.idx_q[k]] = from[k] + OFFSET[k];
    }
    for (int i = 0; i < IK_ITERATIONS; ++i) {
        pinocchio::forwardKinematics(kin.model, kin.data, kin.q);
        pinocchio::updateFramePlacement(kin.model, kin.data, kin.foot);
        const Vec3 err = target - kin.data.oMf[kin.foot].translation();
        if (err.norm() < IK_TOLERANCE) {
            break;
        }
        kin.jac.setZero();
        pinocchio::computeFrameJacobian(kin.model, kin.data, kin.q, kin.foot,
                                        pinocchio::LOCAL_WORLD_ALIGNED, kin.jac);
        const Eigen::Matrix3d j = kin.jac.topRows<3>();
        Eigen::Matrix3d jjt = j * j.transpose();
        jjt.diagonal().array() += IK_DAMPING;
        const Vec3 v = j.transpose() * jjt.ldlt().solve(err);
        pinocchio::integrate(kin.model, kin.q, v, kin.q_next);
        kin.q.swap(kin.q_next);
    }
    Motors motors;
    for (std::size_t k = 0; k < JOINTS.size(); ++k) {
        motors[k] = kin.q[kin.idx_q[k]] - OFFSET[k];
    }
    return motors;
}

double degrees(double rad) { return rad * 180.0 / PI; }

// Builds the model, then simulates two cycles from the zero posture at the loop's dt: prints the
// joint range and peak speed, and stops the run when they leave `limits`.
void check_gait(const kmr::Time &time, kmr::State &state, const kmr::Sensors &,
                const Limits &limits) {
    Kinematics &kin = kinematics();
    // DECISION: simulated at the loop's dt rather than a second copy of TC: it checks the
    // cadence the robot runs.
    const double dt = time.dt();
    Motors lo, hi, prev{};
    lo.fill(std::numeric_limits<double>::max());
    hi.fill(std::numeric_limits<double>::lowest());
    double peak = 0.0;
    const int steps = static_cast<int>(2.0 * TG / dt);
    for (int k = 0; k <= steps; ++k) {
        const Motors q = inverse_kinematics(kin, prev, foot(k * dt));
        for (std::size_t j = 0; j < q.size(); ++j) {
            lo[j] = std::min(lo[j], q[j]);
            hi[j] = std::max(hi[j], q[j]);
            peak = std::max(peak, std::abs(q[j] - prev[j]) / dt);
        }
        prev = q;
    }
    std::printf("range[deg] lo=[%.1f, %.1f, %.1f] hi=[%.1f, %.1f, %.1f] peak=%.2f rad/s at "
                "SPEED 1\n",
                degrees(lo[0]), degrees(lo[1]), degrees(lo[2]), degrees(hi[0]), degrees(hi[1]),
                degrees(hi[2]), peak);
    std::fflush(stdout);

    char reason[KMR_REASON_LEN];
    for (std::size_t j = 0; j < JOINTS.size(); ++j) {
        if (lo[j] < limits.lo[j] || hi[j] > limits.hi[j]) {
            std::snprintf(reason, sizeof reason,
                          "geckoleg: the %s swings over [%.3f, %.3f] rad, outside [%.3f, %.3f]",
                          NAMES[j], lo[j], hi[j], limits.lo[j], limits.hi[j]);
            state.emergency_stop(reason);
            return;
        }
    }
    if (peak > limits.peak) {
        std::snprintf(reason, sizeof reason, "geckoleg: peak %.2f rad/s over the declared %.2f",
                      peak, limits.peak);
        state.emergency_stop(reason);
    }
}

// Writes the three goals of model time elapsed x speed; prints goals and measures once a
// second; stops after PROGRAM_S.
void walk(const kmr::Time &time, kmr::State &state, const kmr::Sensors &, Gait &gait) {
    const double wall = time.elapsed();
    const double tau = wall * gait.speed;
    if (tau > PROGRAM_S) {
        state.shutdown("geckoleg: program over");
        return;
    }
    const Motors q = inverse_kinematics(kinematics(), gait.prev, foot(tau));
    gait.prev = q;
    std::array<float, 3> goals;
    std::transform(q.begin(), q.end(), goals.begin(),
                   [](double v) { return static_cast<float>(v); });
    if (state.group(JOINTS).set_all_q(goals) != KMR_OK) {
        state.shutdown("geckoleg: goal write failed");
        return;
    }
    if (wall >= gait.next_print) {
        gait.next_print += 1.0;
        std::array<double, 3> measured;
        for (std::size_t j = 0; j < JOINTS.size(); ++j) {
            auto m = state.q_at(JOINTS[j]);
            measured[j] = m ? degrees(*m) : std::numeric_limits<double>::quiet_NaN();
        }
        std::printf("t=%7.3fs goal[deg]=[%7.2f, %7.2f, %7.2f] meas[deg]=[%7.2f, %7.2f, %7.2f]\n",
                    tau, degrees(q[0]), degrees(q[1]), degrees(q[2]), measured[0], measured[1],
                    measured[2]);
        std::fflush(stdout);
    }
}

} // namespace

KMR_CONTROLLER(check_gait_cpp, check_gait)
KMR_CONTROLLER(walk_cpp, walk)
