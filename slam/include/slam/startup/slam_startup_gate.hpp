#ifndef SLAM__NODE__SLAM_STARTUP_GATE_HPP_
#define SLAM__NODE__SLAM_STARTUP_GATE_HPP_

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/sensor_gps.hpp>

#include "startup/slam_startup_contract.hpp"

class SlamStartupGate
{
public:
  enum class StartupMode
  {
    GpsDisabled,
    GpsRequiredInit
  };

  enum class RuntimeState
  {
    WaitGpsInit,
    Running,
    DegradedNoGps
  };

  struct StateTransition
  {
    bool changed = false;
    RuntimeState from = RuntimeState::Running;
    RuntimeState to = RuntimeState::Running;
    std::string reason;
  };

  struct GpsSampleResult
  {
    enum class Status
    {
      Ignored,
      Rejected,
      Pending,
      Ready
    };

    Status status = Status::Ignored;
    std::size_t acceptedSampleCount = 0;
    std::string reason;
    std::optional<GpsInitializationMetrics> metrics;
    std::optional<GpsReference> reference;
    StateTransition transition;
  };

  struct WatchdogResult
  {
    bool shouldWarnBlocked = false;
    double elapsedSec = 0.0;
    StateTransition transition;
  };

  StateTransition configure(
    SlamStartupContractConfig&& config,
    const rclcpp::Time& startupBeginTime);

  bool shouldDropSlamInput() const;
  bool canProcessSlamInput() const;
  bool requiresGpsSubscription() const;

  GpsSampleResult onGpsSample(
    const px4_msgs::msg::SensorGps& msg,
    const rclcpp::Time& receiveTime);

  WatchdogResult onWatchdogTick(const rclcpp::Time& now);

  static const char* runtimeStateToString(RuntimeState state);
  static void logTransition(
    const rclcpp::Logger& logger,
    const StateTransition& transition);

private:
  StateTransition setRuntimeState(RuntimeState newState, const std::string& reason);

private:
  StartupMode _mode = StartupMode::GpsDisabled;
  RuntimeState _runtimeState = RuntimeState::Running;
  rclcpp::Time _startupBeginTime;
  bool _dropInputWhileWaitingGpsInit = true;
  bool _allowDegradedNoGps = false;
  double _gpsInitTimeoutSec = 15.0;
  std::unique_ptr<GpsStartupSession> _gpsStartupSession;
};

#endif  // SLAM__NODE__SLAM_STARTUP_GATE_HPP_
