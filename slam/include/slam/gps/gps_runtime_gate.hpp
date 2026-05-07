#ifndef SLAM__GPS__GPS_RUNTIME_GATE_HPP_
#define SLAM__GPS__GPS_RUNTIME_GATE_HPP_

#include <optional>

#include <Eigen/Dense>

#include "common/def_slam_core.hpp"

namespace slam
{

class GpsRuntimeGate
{
public:
  explicit GpsRuntimeGate(const GpsRuntimeGateConfig& config = GpsRuntimeGateConfig());

  void setConfig(const GpsRuntimeGateConfig& config);
  const GpsRuntimeGateConfig& config() const;

  void reset();
  std::size_t badStreakCount() const;

  GpsRuntimeGateDecision evaluate(
    const GpsConstraint& constraint,
    const Eigen::Vector3d& predictedEnuPosition,
    std::optional<double> predictedSpeedMps = std::nullopt);

private:
  static bool isFiniteVector(const Eigen::Vector3d& value);

  GpsRuntimeGateConfig _config;
  std::size_t _badStreakCount = 0;
};

}  // namespace slam

#endif  // SLAM__GPS__GPS_RUNTIME_GATE_HPP_
