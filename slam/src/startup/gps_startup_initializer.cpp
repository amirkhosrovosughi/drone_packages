#include "startup/gps_startup_initializer.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace
{
constexpr double kApproxMetersPerDegLat = 111320.0;
}

GpsStartupInitializer::GpsStartupInitializer(GpsInitializationPolicy policy)
: _policy(std::move(policy))
{
}

GpsInitializationOutcome GpsStartupInitializer::ingest(
  const px4_msgs::msg::SensorGps& msg,
  const rclcpp::Time& receiveTime)
{
  GpsInitializationOutcome outcome;

  std::string rejectReason;
  if (!isSampleAcceptable(msg, &rejectReason))
  {
    outcome.rejected = true;
    outcome.reason = rejectReason;
    outcome.acceptedSampleCount = _samples.size();
    outcome.metrics = buildMetrics(0.0);
    return outcome;
  }

  trimToWindow(receiveTime);

  Sample sample;
  sample.latitudeDeg = msg.latitude_deg;
  sample.longitudeDeg = msg.longitude_deg;
  sample.altitudeM = msg.altitude_msl_m;
  sample.eph = msg.eph;
  sample.epv = msg.epv;
  sample.speedMps = msg.vel_m_s;
  sample.fixType = msg.fix_type;
  sample.receiveTime = receiveTime;
  _samples.push_back(sample);

  outcome.acceptedSampleCount = _samples.size();

  const double maxDispersionM = computeMaxDispersionMeters();
  outcome.metrics = buildMetrics(maxDispersionM);

  if (static_cast<int>(_samples.size()) < _policy.minSamples)
  {
    outcome.reason = "GPS init collecting accepted samples.";
    return outcome;
  }

  if (!std::isfinite(maxDispersionM) || maxDispersionM > _policy.stationaryMaxDispersionM)
  {
    outcome.rejected = true;
    std::ostringstream oss;
    oss << "GPS init sample cluster dispersion is too high ("
        << maxDispersionM
        << " m > "
        << _policy.stationaryMaxDispersionM
        << " m).";
    outcome.reason = oss.str();
    outcome.acceptedSampleCount = _samples.size();
    _samples.clear();
    return outcome;
  }

  outcome = buildReadyOutcome();
  outcome.ready = true;
  outcome.metrics = buildMetrics(maxDispersionM);
  return outcome;
}

void GpsStartupInitializer::reset()
{
  _samples.clear();
}

bool GpsStartupInitializer::isSampleAcceptable(
  const px4_msgs::msg::SensorGps& msg,
  std::string* reason) const
{
  if (msg.fix_type < _policy.requiredFixType)
  {
    if (reason)
    {
      std::ostringstream oss;
      oss << "GPS fix type below required threshold ("
          << static_cast<int>(msg.fix_type)
          << " < "
          << _policy.requiredFixType
          << ").";
      *reason = oss.str();
    }
    return false;
  }

  if (!std::isfinite(msg.eph) || msg.eph > _policy.maxEph)
  {
    if (reason)
    {
      std::ostringstream oss;
      oss << "GPS eph exceeds allowed threshold ("
          << msg.eph
          << " > "
          << _policy.maxEph
          << ").";
      *reason = oss.str();
    }
    return false;
  }

  if (!std::isfinite(msg.epv) || msg.epv > _policy.maxEpv)
  {
    if (reason)
    {
      std::ostringstream oss;
      oss << "GPS epv exceeds allowed threshold ("
          << msg.epv
          << " > "
          << _policy.maxEpv
          << ").";
      *reason = oss.str();
    }
    return false;
  }

  if (!std::isfinite(msg.vel_m_s) || std::abs(msg.vel_m_s) > _policy.stationaryMaxSpeedMps)
  {
    if (reason)
    {
      std::ostringstream oss;
      oss << "GPS speed is too high for stationary initialization ("
          << msg.vel_m_s
          << " m/s > "
          << _policy.stationaryMaxSpeedMps
          << " m/s).";
      *reason = oss.str();
    }
    return false;
  }

  if (!std::isfinite(msg.latitude_deg) || !std::isfinite(msg.longitude_deg) ||
      !std::isfinite(msg.altitude_msl_m))
  {
    if (reason)
    {
      *reason = "GPS sample has non-finite geodetic values.";
    }
    return false;
  }

  return true;
}

void GpsStartupInitializer::trimToWindow(const rclcpp::Time& now)
{
  if (_samples.empty())
  {
    return;
  }

  _samples.erase(
    std::remove_if(
      _samples.begin(),
      _samples.end(),
      [this, &now](const Sample& sample)
      {
        return (now - sample.receiveTime).seconds() > _policy.maxWindowSec;
      }),
    _samples.end());
}

double GpsStartupInitializer::computeMaxDispersionMeters() const
{
  if (_samples.empty())
  {
    return 0.0;
  }

  double centerLat = _samples.front().latitudeDeg;
  double centerLon = _samples.front().longitudeDeg;

  if (_policy.useAverage)
  {
    double latSum = 0.0;
    double lonSum = 0.0;
    for (const Sample& sample : _samples)
    {
      latSum += sample.latitudeDeg;
      lonSum += sample.longitudeDeg;
    }

    centerLat = latSum / static_cast<double>(_samples.size());
    centerLon = lonSum / static_cast<double>(_samples.size());
  }

  double maxDistance = 0.0;
  for (const Sample& sample : _samples)
  {
    const double distance = planarDistanceMeters(
      centerLat,
      centerLon,
      sample.latitudeDeg,
      sample.longitudeDeg);
    maxDistance = std::max(maxDistance, distance);
  }

  return maxDistance;
}

double GpsStartupInitializer::computeWindowDurationSec() const
{
  if (_samples.size() < 2)
  {
    return 0.0;
  }

  const rclcpp::Time minTime = _samples.front().receiveTime;
  const rclcpp::Time maxTime = _samples.back().receiveTime;
  return (maxTime - minTime).seconds();
}

GpsInitializationMetrics GpsStartupInitializer::buildMetrics(double maxDispersionM) const
{
  GpsInitializationMetrics metrics;
  metrics.acceptedSampleCount = _samples.size();
  metrics.minSamplesRequired = static_cast<std::size_t>(std::max(0, _policy.minSamples));
  metrics.windowDurationSec = computeWindowDurationSec();
  metrics.maxWindowSec = _policy.maxWindowSec;
  metrics.maxDispersionM = maxDispersionM;
  metrics.dispersionThresholdM = _policy.stationaryMaxDispersionM;
  metrics.ephThreshold = _policy.maxEph;
  metrics.epvThreshold = _policy.maxEpv;
  metrics.speedThresholdMps = _policy.stationaryMaxSpeedMps;
  metrics.requiredFixType = _policy.requiredFixType;
  metrics.useAverage = _policy.useAverage;

  if (_samples.empty())
  {
    return metrics;
  }

  double ephSum = 0.0;
  double epvSum = 0.0;
  double speedSum = 0.0;
  double ephMax = _samples.front().eph;
  double epvMax = _samples.front().epv;
  double speedMax = std::abs(_samples.front().speedMps);

  for (const Sample& sample : _samples)
  {
    ephSum += sample.eph;
    epvSum += sample.epv;
    speedSum += std::abs(sample.speedMps);
    ephMax = std::max(ephMax, sample.eph);
    epvMax = std::max(epvMax, sample.epv);
    speedMax = std::max(speedMax, std::abs(sample.speedMps));
  }

  const double count = static_cast<double>(_samples.size());
  metrics.meanEph = ephSum / count;
  metrics.maxEph = ephMax;
  metrics.meanEpv = epvSum / count;
  metrics.maxEpv = epvMax;
  metrics.meanSpeedMps = speedSum / count;
  metrics.maxSpeedMps = speedMax;
  return metrics;
}

double GpsStartupInitializer::planarDistanceMeters(
  double latRefDeg,
  double lonRefDeg,
  double latDeg,
  double lonDeg)
{
  constexpr double kDegToRad = M_PI / 180.0;
  const double latRefRad = latRefDeg * kDegToRad;
  const double metersPerDegLon = kApproxMetersPerDegLat * std::cos(latRefRad);

  const double dx = (lonDeg - lonRefDeg) * metersPerDegLon;
  const double dy = (latDeg - latRefDeg) * kApproxMetersPerDegLat;
  return std::sqrt(dx * dx + dy * dy);
}

GpsInitializationOutcome GpsStartupInitializer::buildReadyOutcome() const
{
  GpsInitializationOutcome outcome;
  outcome.acceptedSampleCount = _samples.size();

  if (_samples.empty())
  {
    outcome.rejected = true;
    outcome.reason = "No GPS samples available for initialization.";
    return outcome;
  }

  if (_policy.useAverage)
  {
    double latSum = 0.0;
    double lonSum = 0.0;
    double altSum = 0.0;

    for (const Sample& sample : _samples)
    {
      latSum += sample.latitudeDeg;
      lonSum += sample.longitudeDeg;
      altSum += sample.altitudeM;
    }

    const double count = static_cast<double>(_samples.size());
    outcome.latitudeDeg = latSum / count;
    outcome.longitudeDeg = lonSum / count;
    outcome.altitudeM = altSum / count;
  }
  else
  {
    const Sample& sample = _samples.back();
    outcome.latitudeDeg = sample.latitudeDeg;
    outcome.longitudeDeg = sample.longitudeDeg;
    outcome.altitudeM = sample.altitudeM;
  }

  return outcome;
}
