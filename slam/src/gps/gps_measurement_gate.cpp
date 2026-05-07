#include "gps/gps_measurement_gate.hpp"

#include <cmath>

namespace slam
{

namespace
{
constexpr std::size_t kRejectLogThrottleInterval = 10;
constexpr float kDefaultGpsUncertaintySentinel = 999.0f;
}

GpsMeasurementGate::GpsMeasurementGate(const GpsRuntimeGateConfig& config)
: _runtimeGate(config)
{
}

void GpsMeasurementGate::setLogger(LoggerPtr logger)
{
  _logger = std::move(logger);
}

void GpsMeasurementGate::reset()
{
  _runtimeGate.reset();
  _acceptedCount = 0;
  _rejectedMeasurementCount = 0;
  _maxBadStreak = 0;
  _inDegradedMode = false;
}

GpsMeasurementGateHealth GpsMeasurementGate::health() const
{
  GpsMeasurementGateHealth h;
  h.acceptedCount = _acceptedCount;
  h.rejectedCount = _rejectedMeasurementCount;
  h.currentBadStreak = _runtimeGate.badStreakCount();
  h.maxBadStreak = _maxBadStreak;
  h.inDegradedMode = _inDegradedMode;
  return h;
}

bool GpsMeasurementGate::shouldAccept(
  const GpsConstraint& constraint,
  const Eigen::Vector3d& predictedEnuPosition,
  std::optional<double> predictedSpeedMps)
{
  if (hasDefaultQualityMetadata(constraint))
  {
    return true;
  }

  const GpsRuntimeGateDecision decision =
    _runtimeGate.evaluate(constraint, predictedEnuPosition, predictedSpeedMps);

  if (decision.accepted)
  {
    ++_acceptedCount;
    const bool wasInDegradedMode = _inDegradedMode;
    _inDegradedMode = false;

    if (wasInDegradedMode && _logger)
    {
      _logger->logInfo(
        "GPS recovered from degraded mode: accepted after streak of ",
        decision.badStreakCount + 1,  // streak just reset to 0 in runtime gate
        " rejections, innovation=", decision.innovationM, "m");
    }
    else if (_logger)
    {
      _logger->logDebug(
        "GPS measurement accepted: innovation=", decision.innovationM,
        "m, streak=0");
    }
    return true;
  }

  ++_rejectedMeasurementCount;
  if (decision.badStreakCount > _maxBadStreak)
  {
    _maxBadStreak = decision.badStreakCount;
  }

  const bool nowInDegradedMode =
    decision.badStreakCount >= _runtimeGate.config().badStreakWarnThreshold;
  const bool enteringDegradedMode = nowInDegradedMode && !_inDegradedMode;
  _inDegradedMode = nowInDegradedMode;

  if (enteringDegradedMode && _logger)
  {
    _logger->logWarn(
      "GPS entering degraded mode: ", decision.badStreakCount,
      " consecutive rejections (threshold=",
      _runtimeGate.config().badStreakWarnThreshold,
      "), last reason=", reasonToString(decision.reason),
      ", innovation=", decision.innovationM, "m. Skipping GPS, continuing pure SLAM.");
  }
  else if (!enteringDegradedMode &&
           (_rejectedMeasurementCount % kRejectLogThrottleInterval) == 0 && _logger)
  {
    _logger->logInfo(
      "GPS measurement rejected: reason=", reasonToString(decision.reason),
      ", innovation=", decision.innovationM,
      "m, streak=", decision.badStreakCount);
  }

  return false;
}

bool GpsMeasurementGate::hasDefaultQualityMetadata(const GpsConstraint& constraint)
{
  return constraint.fixType == 0 &&
    constraint.satellitesUsed == 0 &&
    !constraint.hasVelocity &&
    std::abs(constraint.eph - kDefaultGpsUncertaintySentinel) < 1e-6f &&
    std::abs(constraint.epv - kDefaultGpsUncertaintySentinel) < 1e-6f;
}

std::string GpsMeasurementGate::reasonToString(GpsRuntimeGateRejectReason reason)
{
  switch (reason)
  {
    case GpsRuntimeGateRejectReason::None:
      return "None";
    case GpsRuntimeGateRejectReason::FixTypeTooLow:
      return "FixTypeTooLow";
    case GpsRuntimeGateRejectReason::EphTooHigh:
      return "EphTooHigh";
    case GpsRuntimeGateRejectReason::EpvTooHigh:
      return "EpvTooHigh";
    case GpsRuntimeGateRejectReason::InnovationTooHigh:
      return "InnovationTooHigh";
    case GpsRuntimeGateRejectReason::SpeedTooHigh:
      return "SpeedTooHigh";
    case GpsRuntimeGateRejectReason::NonFiniteInput:
      return "NonFiniteInput";
    default:
      return "Unknown";
  }
}

}  // namespace slam