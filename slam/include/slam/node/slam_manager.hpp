#ifndef SLAM__NODE__SLAM_MANAGER_HPP_
#define SLAM__NODE__SLAM_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "pipeline/slam_pipeline.hpp"
#include "common/slam_logger.hpp"
#include "common/def_slam_core.hpp"
#include "measurement/measurement_factory.hpp"
#include "startup/slam_startup_gate.hpp"
#ifdef USE_GPS
#include "gps/gps_local_frame.hpp"
#include "gps/gps_manager.hpp"

#include <px4_msgs/msg/sensor_gps.hpp>
#endif
#include "sensor_msgs/msg/camera_info.hpp"
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
 *  - Forward motion & observation data to pipeline
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
   * @brief Configure startup integration mode and state machine behavior.
   */
  void configureStartupContract();

  /**
   * @brief Create ROS publishers.
   */
  void createPublishers();

  /**
   * @brief Initialize pipeline and internal state.
   */
  void initialize();

  /**
   * @brief Odometry callback.
   * @param msg Vehicle odometry
   */
  void odometryCallback(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

#ifdef USE_GPS
  /**
   * @brief GPS callback — delegates to GpsManager.
   */
  void gpsCallback(
    const px4_msgs::msg::SensorGps::SharedPtr msg);
#endif

  /**
   * @brief Feature observation callback.
   * @param msg Unified feature observation
   */
  void feature3dPointCallback(
    const drone_msgs::msg::PointList::SharedPtr msg);

  /**
   * @brief Feature bbox observation callback.
   * @param msg Unified feature bbox observation
   */
  void featureBboxCallback(
    const vision_msgs::msg::Detection3DArray::SharedPtr msg);

  /**
   * @brief Publish map output.
   * @param map Map summary
   */
  void publishMap(const MapSummary& map);

  /**
   * @brief Update sensor transform from TF.
   */
  void updateCameraExtrinsic();

  /**
   * @brief Camera info callback.
   * @param cameraInfo Camera intrinsic info.
   */
  void cameraIntrinsicCallback(const sensor_msgs::msg::CameraInfo& cameraInfo);

  /**
   * @brief Get current time in seconds.
   * @return double Time in seconds
   */
  double getCurrentTimeInSeconds();

  /**
   * @brief Initialize camera info in ObservationBuilder.
   */
  void initializeCameraInfo();

  /**
   * @brief Startup watchdog callback for degraded fallback handling.
   */
  void startupWatchdogCallback();

private:
  std::shared_ptr<slam::SlamPipeline> _slam; ///< Active SLAM pipeline
  std::shared_ptr<MeasurementFactory> _measurementFactory; ///< Measurement factory for pipeline
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odomSub; ///< Odometry subscriber
#ifdef USE_GPS
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr _gpsSub; ///< GPS subscriber used for startup gating
#endif
  rclcpp::Subscription<drone_msgs::msg::PointList>::SharedPtr _obs3dPointSub; ///< Observation 3D coordinate subscriber
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr _obsBboxSub; ///< Observation bbox subscriber
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _cameraIntrinsicSubscriber; ///< Subscriber for camera intrinsic information
  rclcpp::Publisher<drone_msgs::msg::MapSummary>::SharedPtr _mapPub; ///< Map publisher
  rclcpp::TimerBase::SharedPtr _cameraExtrinsicTimer; ///< Timer for sensor transform update
  rclcpp::TimerBase::SharedPtr _mapPubTimer; ///< Timer for publish map tasks.
  rclcpp::TimerBase::SharedPtr _startupWatchdogTimer; ///< Timer for startup state transitions

  std::unique_ptr<tf2_ros::Buffer> _tfBuffer; ///< TF buffer
  std::shared_ptr<tf2_ros::TransformListener> _tfListener; ///< TF listener
  LoggerPtr _logger; ///< Logger instance
  Eigen::Vector3f _lastPositionEnu = Eigen::Vector3f::Zero(); ///< Last position in ENU frame
  CameraInfo _cameraInfo; ///< Camera intrinsic and extrinsic information
  bool _cameraIntrinsicLoaded = false; ///< Flag indicating if camera intrinsic parameters are loaded
  bool _cameraExtrinsicLoaded = false; ///< Flag indicating if camera extrinsic parameters are loaded

  std::unique_ptr<SlamStartupGate> _startupGate; ///< Startup gate managing GPS init state and SLAM input gating.
#ifdef USE_GPS
  GpsLocalFrame _gpsLocalFrame; ///< Helper storing the GPS anchor and projecting future samples to ENU.
  std::unique_ptr<GpsManager> _gpsManager; ///< GPS signal handler (startup gating + post-init projection).
  bool _gpsFusionEnabled = false; ///< Whether live GPS samples are routed to pipeline fusion after anchor creation.
#endif
};

#endif  // SLAM__NODE__SLAM_MANAGER_HPP_
