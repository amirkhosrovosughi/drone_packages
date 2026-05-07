#ifndef SLAM__GPS__GPS_MANAGER_HPP_
#define SLAM__GPS__GPS_MANAGER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

#include "common/def_slam_core.hpp"
#include "gps/gps_local_frame.hpp"
#include "pipeline/slam_pipeline.hpp"
#include "startup/slam_startup_gate.hpp"

/**
 * @class GpsManager
 * @brief GPS signal handler for the SLAM node.
 *
 * Owns the full GPS sample lifecycle:
 *  - During startup: feeds samples to SlamStartupGate and triggers local-frame
 *    anchor creation once quality criteria are met.
 *  - After startup: projects live GPS samples to local ENU via GpsLocalFrame
 *    (Pipeline fusion will extend this path).
 *
 * Extracted from SlamManager::gpsCallback to keep GPS-domain logic in one place
 * and make SlamManager::gpsCallback a thin 1-line delegate.
 */
class GpsManager
{
public:
  GpsManager(
    SlamStartupGate& startupGate,
    GpsLocalFrame& gpsLocalFrame,
    slam::SlamPipeline& pipeline,
    bool fusionEnabled,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Process one GPS sample.
   * @param msg Raw GPS message from PX4.
   * @param now ROS receive time.
   */
  void onSample(
    const px4_msgs::msg::SensorGps& msg,
    const rclcpp::Time& now);

private:
  /**
   * @brief Build and store the local ENU anchor, then notify the pipeline.
   */
  void initializeLocalFrameAnchor(
    const GpsReference& reference,
    const px4_msgs::msg::SensorGps& msg);

  SlamStartupGate& _startupGate;
  GpsLocalFrame& _gpsLocalFrame;
  slam::SlamPipeline& _pipeline;
  bool _fusionEnabled;
  rclcpp::Logger _logger;
  rclcpp::Clock::SharedPtr _clock;
};

#endif  // SLAM__GPS__GPS_MANAGER_HPP_
