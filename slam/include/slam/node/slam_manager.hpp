#ifndef SLAM__SLAM_MANAGER_HPP_
#define SLAM__SLAM_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "backend/slam_backend.hpp"
#include "common/slam_logger.hpp"

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <drone_msgs/msg/point_list.hpp>
#include <drone_msgs/msg/map_summary.hpp>

/**
 * @class SlamManager
 * @brief Algorithm-agnostic SLAM orchestration node.
 *
 * Responsibilities:
 *  - ROS subscriptions / publications
 *  - Coordinate frames & transforms
 *  - Forward motion & observation data to backend
 */
class SlamManager : public rclcpp::Node
{
public:
  /**
   * @brief Construct SlamManager node.
   */
  SlamManager();

private:
  /**
   * @brief Create ROS subscriptions.
   */
  void createSubscribers();

  /**
   * @brief Create ROS publishers.
   */
  void createPublishers();

  /**
   * @brief Initialize backend and internal state.
   */
  void initialize();

  /**
   * @brief Odometry callback.
   * @param msg Vehicle odometry
   */
  void odometryCallback(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  /**
   * @brief Feature observation callback.
   * @param msg Unified feature observation
   */
  void featureDetectionCallback(
    const drone_msgs::msg::PointList::SharedPtr msg);

  /**
   * @brief Publish map output.
   * @param map Map summary
   */
  void publishMap(const MapSummary& map);

  /**
   * @brief Update sensor transform from TF.
   */
  void updateSensorTransform();

  /**
   * @brief Get current time in seconds.
   * @return double Time in seconds
   */
  double getCurrentTimeInSeconds();

private:
  std::shared_ptr<slam::SlamBackend> _backend; ///< Active SLAM backend
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odomSub; ///< Odometry subscriber
  rclcpp::Subscription<drone_msgs::msg::PointList>::SharedPtr _obsSub; ///< Observation subscriber
  rclcpp::Publisher<drone_msgs::msg::MapSummary>::SharedPtr _mapPub; ///< Map publisher
  rclcpp::TimerBase::SharedPtr _sensor_transform_timer; ///< Timer for sensor transform update
  rclcpp::TimerBase::SharedPtr _map_pub_timer; ///< Timer for publish map tasks.

  std::unique_ptr<tf2_ros::Buffer> _tfBuffer; ///< TF buffer
  std::shared_ptr<tf2_ros::TransformListener> _tfListener; ///< TF listener
  LoggerPtr _logger; ///< Logger instance
  bool _sensorTransformLoaded = false; ///< Transform loaded flag
  Eigen::Vector3f _last_position_enu = Eigen::Vector3f::Zero(); ///< Last position in ENU frame
};

#endif  // SLAM__SLAM_MANAGER_HPP_
