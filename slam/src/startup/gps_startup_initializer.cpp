#include "startup/gps_startup_initializer.hpp"

#include <algorithm>
#include <cmath>

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
    return outcome;
  }

  trimToWindow(receiveTime);

  Sample sample;
  sample.latitudeDeg = msg.latitude_deg;
  sample.longitudeDeg = msg.longitude_deg;
  sample.altitudeM = msg.altitude_msl_m;
  sample.speedMps = msg.vel_m_s;
  sample.receiveTime = receiveTime;
  _samples.push_back(sample);

  outcome.acceptedSampleCount = _samples.size();

  if (static_cast<int>(_samples.size()) < _policy.minSamples)
  {
    return outcome;
  }

  const double maxDispersionM = computeMaxDispersionMeters();
  if (!std::isfinite(maxDispersionM) || maxDispersionM > _policy.stationaryMaxDispersionM)
  {
    outcome.rejected = true;
    outcome.reason = "GPS init sample cluster dispersion is too high.";
    outcome.acceptedSampleCount = _samples.size();
    _samples.clear();
    return outcome;
  }

  outcome = buildReadyOutcome();
  outcome.ready = true;
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
      *reason = "GPS fix type below required threshold.";
    }
    return false;
  }

  if (!std::isfinite(msg.eph) || msg.eph > _policy.maxEph)
  {
    if (reason)
    {
      *reason = "GPS eph exceeds allowed threshold.";
    }
    return false;
  }

  if (!std::isfinite(msg.epv) || msg.epv > _policy.maxEpv)
  {
    if (reason)
    {
      *reason = "GPS epv exceeds allowed threshold.";
    }
    return false;
  }

  if (!std::isfinite(msg.vel_m_s) || std::abs(msg.vel_m_s) > _policy.stationaryMaxSpeedMps)
  {
    if (reason)
    {
      *reason = "GPS speed is too high for stationary initialization.";
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
