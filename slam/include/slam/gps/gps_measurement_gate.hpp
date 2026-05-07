#ifndef SLAM__GPS__GPS_MEASUREMENT_GATE_HPP_
#define SLAM__GPS__GPS_MEASUREMENT_GATE_HPP_

#include <optional>

#include <Eigen/Dense>

#include "common/def_slam_core.hpp"
#include "common/slam_logger.hpp"
#include "gps/gps_runtime_gate.hpp"

namespace slam
{

class GpsMeasurementGate
{
public:
  explicit GpsMeasurementGate(
    const GpsRuntimeGateConfig& config = GpsRuntimeGateConfig());

  void setLogger(LoggerPtr logger);
  void reset();

  bool shouldAccept(
    const GpsConstraint& constraint,
    const Eigen::Vector3d& predictedEnuPosition,
    std::optional<double> predictedSpeedMps = std::nullopt);

  GpsMeasurementGateHealth health() const;

private:
  static bool hasDefaultQualityMetadata(const GpsConstraint& constraint);
  static std::string reasonToString(GpsRuntimeGateRejectReason reason);

  GpsRuntimeGate _runtimeGate;
  std::size_t _acceptedCount = 0;
  std::size_t _rejectedMeasurementCount = 0;
  std::size_t _maxBadStreak = 0;
  bool _inDegradedMode = false;
  LoggerPtr _logger;
};

}  // namespace slam

#endif  // SLAM__GPS__GPS_MEASUREMENT_GATE_HPP_