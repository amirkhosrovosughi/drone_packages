#ifndef SLAM__NODE__GPS_STARTUP_INITIALIZER_HPP_
#define SLAM__NODE__GPS_STARTUP_INITIALIZER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/sensor_gps.hpp>

struct GpsInitializationPolicy
{
  int minSamples = 5;
  double maxWindowSec = 3.0;
  int requiredFixType = 3;
  double maxEph = 2.5;
  double maxEpv = 4.0;
  double stationaryMaxSpeedMps = 0.3;
  double stationaryMaxDispersionM = 1.5;
  bool useAverage = true;
};

struct GpsInitializationOutcome
{
  bool ready = false;
  bool rejected = false;
  std::size_t acceptedSampleCount = 0;
  double latitudeDeg = 0.0;
  double longitudeDeg = 0.0;
  double altitudeM = 0.0;
  std::string reason;
};

class GpsStartupInitializer
{
public:
  explicit GpsStartupInitializer(GpsInitializationPolicy policy);

  GpsInitializationOutcome ingest(
    const px4_msgs::msg::SensorGps& msg,
    const rclcpp::Time& receiveTime);

  void reset();

private:
  struct Sample
  {
    double latitudeDeg;
    double longitudeDeg;
    double altitudeM;
    double speedMps;
    rclcpp::Time receiveTime;
  };

  bool isSampleAcceptable(
    const px4_msgs::msg::SensorGps& msg,
    std::string* reason) const;

  void trimToWindow(const rclcpp::Time& now);

  double computeMaxDispersionMeters() const;

  static double planarDistanceMeters(
    double latRefDeg,
    double lonRefDeg,
    double latDeg,
    double lonDeg);

  GpsInitializationOutcome buildReadyOutcome() const;

private:
  GpsInitializationPolicy _policy;
  std::vector<Sample> _samples;
};

#endif  // SLAM__NODE__GPS_STARTUP_INITIALIZER_HPP_
