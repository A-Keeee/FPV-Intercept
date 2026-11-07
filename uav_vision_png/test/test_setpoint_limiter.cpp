#include <gtest/gtest.h>
#include <limits>
#include <vector>

#include "uav_vision_png/setpoint_limiter.hpp"

TEST(SetpointLimiter, BoundsSpeedAndAccelerationThroughTurnsLossAndRecovery) {
    vpng::SetpointLimits limits;
    vpng::SetpointLimiter limiter;
    const std::vector<Eigen::Vector3d> targets{
        {20, 20, -20}, {-20, 20, 20}, {0, 0, 0}, {3, 0, 0}, {0, 0, 20}, {0, -20, -20}};
    for (const auto &target : targets) {
        for (int i = 0; i < 1600; ++i) {
            const auto previous = limiter.velocity().eval();
            const double previous_yaw = limiter.yaw_rate();
            limiter.update(target, target.x(), 0.005, limits);
            EXPECT_LE((limiter.velocity() - previous).head<2>().norm(), limits.accel_xy * 0.005 + 1e-12);
            EXPECT_LE(std::abs(limiter.velocity().z() - previous.z()), limits.accel_z * 0.005 + 1e-12);
            EXPECT_LE(std::abs(limiter.yaw_rate() - previous_yaw), limits.yaw_accel * 0.005 + 1e-12);
            EXPECT_LE(limiter.velocity().norm(), limits.speed + 1e-12);
            EXPECT_LE(std::abs(limiter.velocity().z()), limits.vertical_speed + 1e-12);
            EXPECT_LE(std::abs(limiter.yaw_rate()), limits.yaw_rate + 1e-12);
        }
    }
}

TEST(SetpointLimiter, StartsFromMeasuredVelocityAndHandlesDelayedCallbacks) {
    vpng::SetpointLimits limits;
    vpng::SetpointLimiter limiter;
    limiter.reset({4, 0, 0});
    // Existing motion above the configured limit must brake gradually.
    limiter.update({0, 0, 0}, 1.0, 5.0, limits);
    EXPECT_NEAR(limiter.velocity().x(), 4.0 - limits.accel_xy * 0.05, 1e-12);
    EXPECT_NEAR(limiter.yaw_rate(), limits.yaw_accel * 0.05, 1e-12);
    const auto previous = limiter.velocity().eval();
    limiter.update({-3, 0, 0}, -1.0, -0.1, limits);
    EXPECT_TRUE(limiter.velocity().isApprox(previous));
}

TEST(SetpointLimiter, RejectsInvalidLimitsAndBrakesOnNonfiniteTargets) {
    vpng::SetpointLimits limits;
    vpng::SetpointLimiter limiter;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    limits.accel_xy = nan;
    EXPECT_THROW(limits.validate(), std::invalid_argument);
    limits.accel_xy = 0.0;
    EXPECT_THROW(limits.validate(), std::invalid_argument);
    limits.accel_xy = 0.8;
    limiter.reset({1, 0, 0});
    limiter.update({nan, 0, 0}, nan, 0.005, limits);
    EXPECT_NEAR(limiter.velocity().x(), 0.996, 1e-12);
    EXPECT_TRUE(limiter.velocity().allFinite());
}
