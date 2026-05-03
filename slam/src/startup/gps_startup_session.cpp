#include "startup/gps_startup_session.hpp"

GpsStartupSession::GpsStartupSession(std::unique_ptr<GpsStartupInitializer> initializer)
: _initializer(std::move(initializer))
{
}

GpsStartupSession::Result GpsStartupSession::ingestSample(
  const px4_msgs::msg::SensorGps& msg,
  const rclcpp::Time& receiveTime)
{
  Result result;

  if (!_initializer)
  {
    result.status = ResultStatus::Rejected;
    result.reason = "GPS startup initializer is not available.";
    return result;
  }

  const GpsInitializationOutcome outcome = _initializer->ingest(msg, receiveTime);
  result.acceptedSampleCount = outcome.acceptedSampleCount;

  if (outcome.rejected)
  {
    result.status = ResultStatus::Rejected;
    result.reason = outcome.reason;
    return result;
  }

  if (!outcome.ready)
  {
    result.status = ResultStatus::Pending;
    return result;
  }

  _reference.latitudeDeg = outcome.latitudeDeg;
  _reference.longitudeDeg = outcome.longitudeDeg;
  _reference.altitudeM = outcome.altitudeM;
  _hasReference = true;

  result.status = ResultStatus::Ready;
  return result;
}

bool GpsStartupSession::hasReference() const
{
  return _hasReference;
}

const GpsReference& GpsStartupSession::reference() const
{
  if (!_hasReference)
  {
    throw std::logic_error("GPS reference is not available yet.");
  }

  return _reference;
}

std::unique_ptr<GpsStartupSession> createGpsStartupSessionFromNodeParameters(
  rclcpp::Node& node)
{
  GpsInitializationPolicy policy;

  policy.minSamples = node.declare_parameter<int>(
    "gps_init_min_samples",
    policy.minSamples);
  policy.maxWindowSec = node.declare_parameter<double>(
    "gps_init_max_window_sec",
    policy.maxWindowSec);
  policy.requiredFixType = node.declare_parameter<int>(
    "gps_init_required_fix_type",
    policy.requiredFixType);
  policy.maxEph = node.declare_parameter<double>(
    "gps_init_max_eph",
    policy.maxEph);
  policy.maxEpv = node.declare_parameter<double>(
    "gps_init_max_epv",
    policy.maxEpv);
  policy.stationaryMaxSpeedMps = node.declare_parameter<double>(
    "gps_init_stationary_max_speed_mps",
    policy.stationaryMaxSpeedMps);
  policy.stationaryMaxDispersionM = node.declare_parameter<double>(
    "gps_init_stationary_max_dispersion_m",
    policy.stationaryMaxDispersionM);
  policy.useAverage = node.declare_parameter<bool>(
    "gps_init_use_average",
    policy.useAverage);

  return std::make_unique<GpsStartupSession>(
    std::make_unique<GpsStartupInitializer>(policy));
}
