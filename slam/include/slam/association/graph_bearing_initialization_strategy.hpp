#ifndef SLAM__GRAPH_BEARING_INITIALIZATION_STRATEGY_HPP_
#define SLAM__GRAPH_BEARING_INITIALIZATION_STRATEGY_HPP_

#include <cmath>

#include "association/under_constrained_initialization_strategy.hpp"

/**
 * @brief Graph-oriented bearing initialization strategy.
 *
 * For the initial skeleton this uses a depth prior, similar to EKF strategy,
 * while keeping a separate type for pipeline-specific tuning later.
 */
class GraphBearingInitializationStrategy : public UnderConstrainedInitializationStrategy
{
public:
  explicit GraphBearingInitializationStrategy(double defaultDepthMeters = 8.0)
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

#endif  // SLAM__GRAPH_BEARING_INITIALIZATION_STRATEGY_HPP_
