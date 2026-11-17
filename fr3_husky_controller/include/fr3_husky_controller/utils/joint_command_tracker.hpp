#pragma once

// 1 kHz joint position command tracker for FR3 position mode.
//
// The policy sends a joint target every 50 ms (zero-order hold).  Sending that
// staircase to the robot would break the FR3 velocity/acceleration/jerk limits
// (motion abort), so the tracker turns it into a smooth command q_r that
//   * converges to the held target without overshoot (sqrt braking curve),
//   * keeps |v| (incl. the position-dependent limit), |a|, |j| below
//     limit_fraction of the FR3 limits, evaluated on the 1 kHz command sequence.
// Linear-region delay is 1/wp (wp = 50 -> 20 ms); larger steps are
// acceleration limited (0.1 rad step: ~113 ms to 50 %).
//
// Reference implementation (must stay identical):
//   dual_fr3_lab/scripts/tools/analyze_position_tracker.py  tracker_update()
// and the Isaac Lab position-mode action term uses the same equations.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include <Eigen/Core>

namespace fr3_husky_controller::utils
{

struct JointCommandTrackerParams
{
    double wp{50.0};              // position gain [1/s]; velocity gain = 4 wp (critically damped)
    double limit_fraction{0.8};   // fraction of the FR3 v/a/j limits used
    double brake_fraction{0.7};   // braking deceleration as a fraction of the acceleration limit
    double dt{0.001};             // command period [s] (the robot differentiates with 1 ms)
};

class JointCommandTracker
{
public:
    static constexpr std::size_t kArmDof = 7;
    // FR3 joint limits, Franka docs "Robot and interface specifications" (FR3).
    static constexpr std::array<double, kArmDof> kMaxVelocity{2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26};
    static constexpr double kMaxAcceleration = 10.0;   // rad/s^2, all joints
    static constexpr double kMaxJerk = 5000.0;         // rad/s^3, all joints
    // Position-dependent velocity limit constants (same docs page).
    static constexpr std::array<double, kArmDof> kVelocityOffset{
        0.6599, 0.2517, 0.2000, 0.3533, 0.5757, 0.4878, 0.4628};
    static constexpr std::array<double, kArmDof> kDeceleration{6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0};
    // Joint position limits used in that formula: the older, tighter FR3 values
    // (also used by the Isaac box task), so the limit holds on either firmware.
    static constexpr std::array<double, kArmDof> kPositionMax{
        2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159};
    static constexpr std::array<double, kArmDof> kPositionMin{
        -2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159};

    JointCommandTracker() = default;
    explicit JointCommandTracker(const JointCommandTrackerParams& params) : params_(params) {}

    void setParams(const JointCommandTrackerParams& params) { params_ = params; }
    const JointCommandTrackerParams& params() const { return params_; }

    // Start at q with zero velocity and acceleration (the robot is at rest).
    void reset(const Eigen::VectorXd& q)
    {
        q_ = q;
        v_ = Eigen::VectorXd::Zero(q.size());
        a_ = Eigen::VectorXd::Zero(q.size());
        velocity_clamped_ = false;
        acceleration_clamped_ = false;
    }

    // One command period towards the held target.  Joint i uses the limits of
    // FR3 joint (i % 7), so a 14-vector covers both arms.
    void step(const Eigen::VectorXd& target)
    {
        const double frac = params_.limit_fraction;
        const double wp = params_.wp;
        const double wv = 4.0 * wp;
        const double dt = params_.dt;
        const double a_max = frac * kMaxAcceleration;
        const double j_step = frac * kMaxJerk * dt;
        const double a_brake = params_.brake_fraction * a_max;
        velocity_clamped_ = false;
        acceleration_clamped_ = false;

        for (Eigen::Index i = 0; i < q_.size(); ++i)
        {
            const std::size_t j = static_cast<std::size_t>(i) % kArmDof;
            const double v_star = sqrtController(target(i) - q_(i), wp, a_brake);

            double lo = 0.0;
            double hi = 0.0;
            velocityLimits(j, q_(i), lo, hi);
            const double v_cl = std::clamp(v_star, lo, hi);

            const double a_star = wv * (v_cl - v_(i));
            const double a_cl = std::clamp(a_star, -a_max, a_max);
            a_(i) += std::clamp(a_cl - a_(i), -j_step, j_step);
            v_(i) += a_(i) * dt;
            q_(i) += v_(i) * dt;

            velocity_clamped_ = velocity_clamped_ || (v_cl != v_star);
            acceleration_clamped_ = acceleration_clamped_ || (a_cl != a_star);
        }
    }

    const Eigen::VectorXd& position() const { return q_; }
    const Eigen::VectorXd& velocity() const { return v_; }
    const Eigen::VectorXd& acceleration() const { return a_; }
    bool velocityClamped() const { return velocity_clamped_; }
    bool accelerationClamped() const { return acceleration_clamped_; }

    // Position at which each joint comes to rest if it starts braking now.
    Eigen::VectorXd stoppingTarget() const
    {
        const double a_brake = params_.brake_fraction * params_.limit_fraction * kMaxAcceleration;
        Eigen::VectorXd target = q_;
        for (Eigen::Index i = 0; i < q_.size(); ++i)
        {
            target(i) += v_(i) * std::abs(v_(i)) / (2.0 * a_brake);
        }
        return target;
    }

    bool isAtRest(double velocity_tol = 1.0e-4, double acceleration_tol = 1.0e-2) const
    {
        return v_.cwiseAbs().maxCoeff() <= velocity_tol && a_.cwiseAbs().maxCoeff() <= acceleration_tol;
    }

    // Linear (kp e) near zero, sqrt(2 a |e|) braking curve further out; C0-continuous.
    static double sqrtController(double error, double kp, double accel)
    {
        const double linear_dist = accel / (kp * kp);
        if (std::abs(error) <= linear_dist)
        {
            return kp * error;
        }
        return std::copysign(std::sqrt(2.0 * accel * (std::abs(error) - 0.5 * linear_dist)), error);
    }

private:
    // FR3 position-dependent velocity limits scaled by limit_fraction, capped at
    // limit_fraction * kMaxVelocity.
    void velocityLimits(std::size_t j, double q, double& lo, double& hi) const
    {
        const double frac = params_.limit_fraction;
        const double v_max = frac * kMaxVelocity[j];
        const double up = std::max(
            0.0, -kVelocityOffset[j] + std::sqrt(std::max(0.0, 2.0 * kDeceleration[j] * (kPositionMax[j] - q))));
        const double down = std::min(
            0.0, kVelocityOffset[j] - std::sqrt(std::max(0.0, 2.0 * kDeceleration[j] * (q - kPositionMin[j]))));
        hi = std::min(v_max, frac * up);
        lo = std::max(-v_max, frac * down);
    }

    JointCommandTrackerParams params_{};
    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
    Eigen::VectorXd a_;
    bool velocity_clamped_{false};
    bool acceleration_clamped_{false};
};

}  // namespace fr3_husky_controller::utils
