#ifndef SLAM__NODE__GPS_STARTUP_SESSION_HPP_
#define SLAM__NODE__GPS_STARTUP_SESSION_HPP_

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/sensor_gps.hpp>

#include "startup/gps_startup_initializer.hpp"

struct GpsReference
{
  double latitudeDeg = 0.0;
  double longitudeDeg = 0.0;
  double altitudeM = 0.0;
};

class GpsStartupSession
{
public:
  enum class ResultStatus
  {
    Pending,
    Rejected,
    Ready
  };

  struct Result
  {
    ResultStatus status = ResultStatus::Pending;
    std::size_t acceptedSampleCount = 0;
    std::string reason;
  };

  explicit GpsStartupSession(std::unique_ptr<GpsStartupInitializer> initializer);

  Result ingestSample(
    const px4_msgs::msg::SensorGps& msg,
    const rclcpp::Time& receiveTime);

  bool hasReference() const;
  const GpsReference& reference() const;

private:
  std::unique_ptr<GpsStartupInitializer> _initializer;
  bool _hasReference = false;
  GpsReference _reference;
};

std::unique_ptr<GpsStartupSession> createGpsStartupSessionFromNodeParameters(
  rclcpp::Node& node);

#endif  // SLAM__NODE__GPS_STARTUP_SESSION_HPP_
