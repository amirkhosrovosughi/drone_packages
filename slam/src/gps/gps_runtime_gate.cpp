#include "gps/gps_runtime_gate.hpp"

#include <cmath>

namespace slam
{

GpsRuntimeGate::GpsRuntimeGate(const GpsRuntimeGateConfig& config)
: _config(config)
{
}

void GpsRuntimeGate::setConfig(const GpsRuntimeGateConfig& config)
{
  _config = config;
}

const GpsRuntimeGateConfig& GpsRuntimeGate::config() const
{
  return _config;
}

void GpsRuntimeGate::reset()
{
  _badStreakCount = 0;
}

std::size_t GpsRuntimeGate::badStreakCount() const
{
  return _badStreakCount;
}

GpsRuntimeGateDecision GpsRuntimeGate::evaluate(
  const GpsConstraint& constraint,
  const Eigen::Vector3d& predictedEnuPosition,
  std::optional<double> predictedSpeedMps)
{
  GpsRuntimeGateDecision decision;
  decision.accepted = false;

  if (!_config.enabled)
  {
    decision.accepted = true;
    decision.reason = GpsRuntimeGateRejectReason::None;
    decision.innovationM = (constraint.enuPosition - predictedEnuPosition).norm();
    _badStreakCount = 0;
    decision.badStreakCount = _badStreakCount;
    return decision;
  }

  if (!isFiniteVector(constraint.enuPosition) || !isFiniteVector(predictedEnuPosition) ||
      !std::isfinite(constraint.eph) || !std::isfinite(constraint.epv))
  {
    decision.reason = GpsRuntimeGateRejectReason::NonFiniteInput;
    ++_badStreakCount;
    decision.badStreakCount = _badStreakCount;
    return decision;
  }

  decision.innovationM = (constraint.enuPosition - predictedEnuPosition).norm();

  if (constraint.fixType < _config.minFixType)
  {
    decision.reason = GpsRuntimeGateRejectReason::FixTypeTooLow;
    ++_badStreakCount;
    decision.badStreakCount = _badStreakCount;
    return decision;
  }

  if (static_cast<double>(constraint.eph) > _config.maxEphM)
  {
    decision.reason = GpsRuntimeGateRejectReason::EphTooHigh;
    ++_badStreakCount;
    decision.badStreakCount = _badStreakCount;
    return decision;
  }

  if (static_cast<double>(constraint.epv) > _config.maxEpvM)
  {
    decision.reason = GpsRuntimeGateRejectReason::EpvTooHigh;
    ++_badStreakCount;
    decision.badStreakCount = _badStreakCount;
    return decision;
  }

  if (!std::isfinite(decision.innovationM) || decision.innovationM > _config.maxInnovationM)
  {
    decision.reason = GpsRuntimeGateRejectReason::InnovationTooHigh;
    ++_badStreakCount;
    decision.badStreakCount = _badStreakCount;
    return decision;
  }

  if (_config.enableSpeedGate)
  {
    std::optional<double> speedToCheck;
    if (constraint.hasVelocity)
    {
      speedToCheck = static_cast<double>(constraint.velMps);
    }
    else if (predictedSpeedMps.has_value())
    {
      speedToCheck = *predictedSpeedMps;
    }

    if (speedToCheck.has_value())
    {
      const double speedMps = *speedToCheck;
      if (!std::isfinite(speedMps) || std::abs(speedMps) > _config.maxSpeedMps)
      {
        decision.reason = GpsRuntimeGateRejectReason::SpeedTooHigh;
        ++_badStreakCount;
        decision.badStreakCount = _badStreakCount;
        return decision;
      }
    }
  }

  decision.accepted = true;
  decision.reason = GpsRuntimeGateRejectReason::None;
  _badStreakCount = 0;
  decision.badStreakCount = _badStreakCount;
  return decision;
}

bool GpsRuntimeGate::isFiniteVector(const Eigen::Vector3d& value)
{
  return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

}  // namespace slam
