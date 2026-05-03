#include "startup/slam_startup_gate.hpp"

SlamStartupGate::StateTransition SlamStartupGate::configure(
  SlamStartupContractConfig&& config,
  const rclcpp::Time& startupBeginTime)
{
  _dropInputWhileWaitingGpsInit = config.dropInputWhileWaitingGpsInit;
  _allowDegradedNoGps = config.allowDegradedNoGps;
  _gpsInitTimeoutSec = config.gpsInitTimeoutSec;
  _gpsStartupSession = std::move(config.gpsStartupSession);
  _startupBeginTime = startupBeginTime;

  if (config.gpsRequiredInit)
  {
    _mode = StartupMode::GpsRequiredInit;
    return setRuntimeState(
      RuntimeState::WaitGpsInit,
      "GPS-enabled startup: waiting for valid GPS initialization sample before SLAM input flow.");
  }

  _mode = StartupMode::GpsDisabled;
  return setRuntimeState(
    RuntimeState::Running,
    "GPS startup gate disabled: preserving current SLAM startup behavior.");
}

bool SlamStartupGate::shouldDropSlamInput() const
{
  return !canProcessSlamInput() && _dropInputWhileWaitingGpsInit;
}

bool SlamStartupGate::canProcessSlamInput() const
{
  if (_runtimeState == RuntimeState::Running ||
      _runtimeState == RuntimeState::DegradedNoGps)
  {
    return true;
  }

  return !_dropInputWhileWaitingGpsInit;
}

bool SlamStartupGate::requiresGpsSubscription() const
{
  return _mode == StartupMode::GpsRequiredInit;
}

SlamStartupGate::GpsSampleResult SlamStartupGate::onGpsSample(
  const px4_msgs::msg::SensorGps& msg,
  const rclcpp::Time& receiveTime)
{
  GpsSampleResult result;

  if (_mode != StartupMode::GpsRequiredInit || _runtimeState != RuntimeState::WaitGpsInit)
  {
    return result;
  }

  if (!_gpsStartupSession)
  {
    result.status = GpsSampleResult::Status::Rejected;
    result.reason = "GPS startup session is not available while GPS init is required.";
    return result;
  }

  const GpsStartupSession::Result sessionResult =
    _gpsStartupSession->ingestSample(msg, receiveTime);
  result.acceptedSampleCount = sessionResult.acceptedSampleCount;
  result.metrics = sessionResult.metrics;

  if (sessionResult.status == GpsStartupSession::ResultStatus::Rejected)
  {
    result.status = GpsSampleResult::Status::Rejected;
    result.reason = sessionResult.reason;
    return result;
  }

  if (sessionResult.status != GpsStartupSession::ResultStatus::Ready)
  {
    result.status = GpsSampleResult::Status::Pending;
    result.reason = sessionResult.reason;
    return result;
  }

  result.status = GpsSampleResult::Status::Ready;
  result.reference = _gpsStartupSession->reference();
  result.transition = setRuntimeState(
    RuntimeState::Running,
    "GPS startup gate satisfied by quality-filtered initialization window.");

  return result;
}

SlamStartupGate::WatchdogResult SlamStartupGate::onWatchdogTick(const rclcpp::Time& now)
{
  WatchdogResult result;

  if (_runtimeState != RuntimeState::WaitGpsInit)
  {
    return result;
  }

  result.elapsedSec = (now - _startupBeginTime).seconds();
  if (result.elapsedSec < _gpsInitTimeoutSec)
  {
    return result;
  }

  if (_allowDegradedNoGps)
  {
    result.transition = setRuntimeState(
      RuntimeState::DegradedNoGps,
      "GPS init timeout reached; proceeding in degraded no-GPS startup mode.");
    return result;
  }

  result.shouldWarnBlocked = true;
  return result;
}

const char* SlamStartupGate::runtimeStateToString(RuntimeState state)
{
  switch (state)
  {
    case RuntimeState::WaitGpsInit:
      return "WAIT_GPS_INIT";
    case RuntimeState::Running:
      return "RUNNING";
    case RuntimeState::DegradedNoGps:
      return "DEGRADED_NO_GPS";
    default:
      return "UNKNOWN";
  }
}

void SlamStartupGate::logTransition(
  const rclcpp::Logger& logger,
  const StateTransition& transition)
{
  if (!transition.changed)
  {
    return;
  }

  RCLCPP_INFO(
    logger,
    "SLAM startup state transition: %s -> %s. %s",
    runtimeStateToString(transition.from),
    runtimeStateToString(transition.to),
    transition.reason.c_str());
}

SlamStartupGate::StateTransition SlamStartupGate::setRuntimeState(
  RuntimeState newState,
  const std::string& reason)
{
  StateTransition transition;

  if (_runtimeState == newState)
  {
    return transition;
  }

  transition.changed = true;
  transition.from = _runtimeState;
  transition.to = newState;
  transition.reason = reason;
  _runtimeState = newState;
  return transition;
}
