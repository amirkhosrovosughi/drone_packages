#ifndef SLAM__MANAGER_HPP_
#define SLAM__MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "common_utilities/transform_util.hpp"
#include <drone_msgs/msg/point_list.hpp>
#include <drone_msgs/msg/map_summary.hpp> 
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "def_slam.hpp"
#include "slam_logger.hpp"

#include "filter/base_filter.hpp"
#include "filter/extended_kalman_filter.hpp"
#include "filter/unscented_kalman_filter.hpp"
#include "filter/fast_slam.hpp"

#include "association/base_association.hpp"
#include "association/joint_compatibility_association.hpp"
#include "association/nearest_neighbor_association.hpp"

typedef std::shared_ptr<BaseFilter> FilterPtr;
typedef std::shared_ptr<BaseAssociation> AssociationPtr;

/**
 * @class SlamManager
 * @brief Main SLAM management node that handles sensor input, filtering, data association, and map publishing.
 */
class SlamManager : public rclcpp::Node {
public:
  /**
   * @brief Construct a new SlamManager node.
   */
  SlamManager();
private:
  /**
   * @brief Create ROS 2 subscriptions for odometry and feature topics.
   */
  void createSubscribers();

  /**
   * @brief Create ROS 2 publishers for SLAM map output.
   */
  void createPublishers();

  /**
   * @brief Initialize internal state and register callbacks with filter and association modules.
   */
  void initialize();

  /**
   * @brief Callback from filter with updated map data.
   * @param map Current map summary.
   */
  void filterCallback(const MapSummary& map);

  /**
   * @brief Callback from association with new measurement data.
   * @param meas Set of measurements from features.
   */
  void associationCallback(const Measurements& meas);

  /**
   * @brief Callback for drone odometry messages.
   * @param odometry Vehicle odometry from PX4.
   */
  void droneOdometryCallback(const px4_msgs::msg::VehicleOdometry odometry);

  /**
   * @brief Callback for detected 3D features.
   * @param features Point list of detected features.
   */
  void featureDetectionCallback(const drone_msgs::msg::PointList features);

  /**
   * @brief Publish the map to ROS 2.
   * @param map Map summary to be published.
   */
  void publishMap(const MapSummary& map);

  /**
   * @brief Update transform between base and camera frames.
   */
  void updateTransform();

  /**
   * @brief Get current system time in seconds.
   * @return double Current time in seconds.
   */
  double getCurrentTimeInSeconds();

  /**
   * @brief Estimate linear speed from consecutive robot positions.
   * @param position Current position of the robot.
   * @return Eigen::Vector3f Estimated linear speed.
   */
  Eigen::Vector3f estimateLinearSpeed(const Eigen::Vector3f& position);

private:
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _droneOdometrySubscriber; ///< Subscriber for drone odometry.
  rclcpp::Subscription<drone_msgs::msg::PointList>::SharedPtr _feature3DCoordinateSubscriber; ///< Subscriber for detected 3D features.
  rclcpp::Publisher<drone_msgs::msg::MapSummary>::SharedPtr _mapPublisher; ///< Publisher for SLAM map summary.
  FilterPtr _filter; ///< Current filter implementation.
  AssociationPtr _association; ///< Current data association strategy.
  std::unique_ptr<tf2_ros::Buffer> _tfBuffer; ///< TF buffer for transforms.
  std::shared_ptr<tf2_ros::TransformListener> _tfListener; ///< TF listener for transforms.
  LoggerPtr _logger; ///< Logger utility.
  rclcpp::TimerBase::SharedPtr _timer; ///< Timer for periodic tasks.
  bool _cameraTransformLoaded = false; ///< Whether the camera transform has been loaded.

  Eigen::Vector3f _previousRobotPosition; ///< Previous robot position (for velocity estimation).
  double _lastOdomTime; ///< Timestamp of last odometry update.
};

#endif  // SLAM__MANAGER_HPP_