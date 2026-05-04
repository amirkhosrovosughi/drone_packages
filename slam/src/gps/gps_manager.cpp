#include "gps/gps_manager.hpp"

#include <Eigen/Dense>

#include "startup/gps_startup_initializer.hpp"

GpsManager::GpsManager(
  SlamStartupGate& startupGate,
  GpsLocalFrame& gpsLocalFrame,
  slam::SlamPipeline& pipeline,
  bool fusionEnabled,
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock)
: _startupGate(startupGate)
, _gpsLocalFrame(gpsLocalFrame)
, _pipeline(pipeline)
, _fusionEnabled(fusionEnabled)
, _logger(logger)
, _clock(clock)
{
}

void GpsManager::onSample(
  const px4_msgs::msg::SensorGps& msg,
  const rclcpp::Time& now)
{
  if (_gpsLocalFrame.hasAnchor())
  {
    const Eigen::Vector3d projectedPosition = _gpsLocalFrame.toEnu(msg);
    RCLCPP_DEBUG(
      _logger,
      "GPS sample projected to local ENU: x=%.3f y=%.3f z=%.3f",
      projectedPosition.x(),
      projectedPosition.y(),
      projectedPosition.z());

    if (_fusionEnabled)
    {
      GpsConstraint constraint;
      constraint.enuPosition    = projectedPosition;
      constraint.sigmaXyM       = static_cast<double>(msg.eph);
      constraint.sigmaZM        = static_cast<double>(msg.epv);
      constraint.timestampUs    = msg.timestamp;
      constraint.fixType        = msg.fix_type;
      constraint.eph            = msg.eph;
      constraint.epv            = msg.epv;
      constraint.satellitesUsed = msg.satellites_used;
      constraint.velMps         = msg.vel_m_s;
      constraint.hasVelocity    = true;
      _pipeline.processGpsMeasurement(constraint);
    }
    return;
  }

  const SlamStartupGate::GpsSampleResult result =
    _startupGate.onGpsSample(msg, now);

  SlamStartupGate::logTransition(_logger, result.transition);

  if (result.status == SlamStartupGate::GpsSampleResult::Status::Rejected)
  {
    RCLCPP_WARN_THROTTLE(
      _logger,
      *_clock,
      2000,
      "GPS init rejected: %s (fix=%u req_fix=%d eph=%.3f epv=%.3f vel=%.3f)",
      result.reason.c_str(),
      static_cast<unsigned int>(msg.fix_type),
      result.metrics.has_value() ? result.metrics->requiredFixType : -1,
      msg.eph,
      msg.epv,
      msg.vel_m_s);
    return;
  }

  if (result.status == SlamStartupGate::GpsSampleResult::Status::Pending)
  {
    if (result.metrics.has_value())
    {
      const GpsInitializationMetrics& metrics = *result.metrics;
      RCLCPP_INFO_THROTTLE(
        _logger,
        *_clock,
        3000,
        "GPS init pending: accepted=%zu/%zu window=%.2fs/%.2fs dispersion=%.3fm/%.3fm"
        " (eph mean/max=%.3f/%.3f, epv mean/max=%.3f/%.3f, speed mean/max=%.3f/%.3f m/s). %s",
        metrics.acceptedSampleCount,
        metrics.minSamplesRequired,
        metrics.windowDurationSec,
        metrics.maxWindowSec,
        metrics.maxDispersionM,
        metrics.dispersionThresholdM,
        metrics.meanEph,
        metrics.maxEph,
        metrics.meanEpv,
        metrics.maxEpv,
        metrics.meanSpeedMps,
        metrics.maxSpeedMps,
        result.reason.c_str());
    }
    else
    {
      RCLCPP_INFO_THROTTLE(
        _logger,
        *_clock,
        3000,
        "GPS init pending: accepted samples=%zu. %s",
        result.acceptedSampleCount,
        result.reason.c_str());
    }
    return;
  }

  if (result.status != SlamStartupGate::GpsSampleResult::Status::Ready ||
      !result.reference.has_value())
  {
    return;
  }

  const GpsReference& reference = *result.reference;

  initializeLocalFrameAnchor(reference, msg);

  RCLCPP_INFO(
    _logger,
    "GPS init complete: accepted=%zu lat=%.10f lon=%.10f alt=%.3f",
    result.acceptedSampleCount,
    reference.latitudeDeg,
    reference.longitudeDeg,
    reference.altitudeM);

  if (result.metrics.has_value())
  {
    const GpsInitializationMetrics& metrics = *result.metrics;
    RCLCPP_INFO(
      _logger,
      "GPS init quality: fix>=%d samples=%zu/%zu window=%.2fs/%.2fs"
      " dispersion=%.3fm/%.3fm eph(mean/max/thr)=%.3f/%.3f/%.3f"
      " epv(mean/max/thr)=%.3f/%.3f/%.3f speed(mean/max/thr)=%.3f/%.3f/%.3f m/s averaging=%s",
      metrics.requiredFixType,
      metrics.acceptedSampleCount,
      metrics.minSamplesRequired,
      metrics.windowDurationSec,
      metrics.maxWindowSec,
      metrics.maxDispersionM,
      metrics.dispersionThresholdM,
      metrics.meanEph,
      metrics.maxEph,
      metrics.ephThreshold,
      metrics.meanEpv,
      metrics.maxEpv,
      metrics.epvThreshold,
      metrics.meanSpeedMps,
      metrics.maxSpeedMps,
      metrics.speedThresholdMps,
      metrics.useAverage ? "true" : "false");
  }
}

void GpsManager::initializeLocalFrameAnchor(
  const GpsReference& reference,
  const px4_msgs::msg::SensorGps& msg)
{
  LocalFrameAnchor anchor;
  anchor.anchorReference = reference;
  anchor.initialEnuPosition = Eigen::Vector3d::Zero();
  anchor.anchorTimestampUs = msg.timestamp;

  // TODO(avosughi): Verify whether msg.timestamp, timestamp_sample, time_utc_usec,
  // or ROS receive time is the correct anchor timebase for later GPS fusion.
  _gpsLocalFrame.setAnchor(anchor);
  _pipeline.applyStartupAnchor(anchor);

  RCLCPP_INFO(
    _logger,
    "Local GPS anchor created: lat=%.10f lon=%.10f alt=%.3f timestamp_us=%llu",
    anchor.anchorReference.latitudeDeg,
    anchor.anchorReference.longitudeDeg,
    anchor.anchorReference.altitudeM,
    static_cast<unsigned long long>(anchor.anchorTimestampUs));
}
