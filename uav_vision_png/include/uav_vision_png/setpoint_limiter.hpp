#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vpng {

struct SetpointLimits {
    double speed = 3.0;
    double vertical_speed = 1.0;
    double accel_xy = 0.8;
    double accel_z = 0.6;
    double yaw_rate = 0.5;
    double yaw_accel = 0.5;

    void validate() const {
        for (double value : {speed, vertical_speed, accel_xy, accel_z, yaw_rate, yaw_accel}) {
            if (!std::isfinite(value) || value <= 0.0)
                throw std::invalid_argument("Setpoint limits must be finite and positive");
        }
    }
};

// Shared by tracking, coasting, braking and recovery so state changes cannot
// bypass the velocity slew limits. This bounds commands, not actual PX4 attitude.
class SetpointLimiter {
public:
    void reset(const Eigen::Vector3d &velocity, double yaw_rate = 0.0) {
        velocity_ = velocity.allFinite() ? velocity : Eigen::Vector3d::Zero();
        yaw_rate_ = std::isfinite(yaw_rate) ? yaw_rate : 0.0;
    }

    void update(Eigen::Vector3d target, double yaw_target, double dt,
                const SetpointLimits &limits) {
        if (!target.allFinite()) target.setZero();
        if (!std::isfinite(yaw_target)) yaw_target = 0.0;
        target.z() = std::clamp(target.z(), -limits.vertical_speed, limits.vertical_speed);
        if (target.norm() > limits.speed) target *= limits.speed / target.norm();
        yaw_target = std::clamp(yaw_target, -limits.yaw_rate, limits.yaw_rate);
        // A delayed callback must not produce a large one-step command change.
        dt = std::isfinite(dt) ? std::clamp(dt, 0.0, 0.05) : 0.0;
        const Eigen::Vector3d delta = target - velocity_;
        double fraction = 1.0;
        if (delta.head<2>().norm() > 0.0)
            fraction = std::min(fraction, limits.accel_xy * dt / delta.head<2>().norm());
        if (std::abs(delta.z()) > 0.0)
            fraction = std::min(fraction, limits.accel_z * dt / std::abs(delta.z()));
        // A common fraction keeps the entire segment inside the speed bounds,
        // including transitions between horizontal and vertical motion.
        velocity_ += fraction * delta;
        yaw_rate_ += std::clamp(yaw_target - yaw_rate_,
                                -limits.yaw_accel * dt, limits.yaw_accel * dt);
    }

    const Eigen::Vector3d &velocity() const { return velocity_; }
    double yaw_rate() const { return yaw_rate_; }

private:
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    double yaw_rate_ = 0.0;
};

}  // namespace vpng
