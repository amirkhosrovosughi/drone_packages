#ifndef SLAM__EKF_BEARING_INITIALIZATION_STRATEGY_HPP_
#define SLAM__EKF_BEARING_INITIALIZATION_STRATEGY_HPP_

#include <cmath>

#include "association/under_constrained_initialization_strategy.hpp"

/**
 * @brief EKF-oriented bearing initialization strategy.
 *
 * Converts a bearing measurement (yaw, pitch) into a world-frame landmark
 * hypothesis by projecting a ray from current robot pose with a fixed default depth.
 */
class EkfBearingInitializationStrategy : public UnderConstrainedInitializationStrategy {
public:
    explicit EkfBearingInitializationStrategy(double defaultDepthMeters = 6.0)
        : _defaultDepthMeters(defaultDepthMeters)
    {
    }

    std::optional<Position> initialize(
        const Measurement& measurement,
        const Pose& robotPose) const override
    {
        if (measurement.payload.size() < 2)
        {
            return std::nullopt;
        }

        const double yaw = measurement.payload[0];
        const double pitch = measurement.payload[1];

        const double x = std::tan(yaw);
        const double y = std::tan(pitch);
        const double z = 1.0;

        Eigen::Vector3d dirRobot(x, y, z);
        const double norm = dirRobot.norm();
        if (norm <= 1e-9)
        {
            return std::nullopt;
        }
        dirRobot /= norm;

        Eigen::Quaterniond qWorldRobot(
            robotPose.quaternion.w,
            robotPose.quaternion.x,
            robotPose.quaternion.y,
            robotPose.quaternion.z);
        qWorldRobot.normalize();

        const Eigen::Vector3d dirWorld = qWorldRobot.toRotationMatrix() * dirRobot;
        const Eigen::Vector3d robotWorld(
            robotPose.position.x,
            robotPose.position.y,
            robotPose.position.z);

        const Eigen::Vector3d landmarkWorld = robotWorld + _defaultDepthMeters * dirWorld;
        return Position(landmarkWorld);
    }

private:
    double _defaultDepthMeters;
};

#endif  // SLAM__EKF_BEARING_INITIALIZATION_STRATEGY_HPP_
