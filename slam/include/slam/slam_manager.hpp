#ifndef SLAM__MANAGER_HPP_
#define SLAM__MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>

// #include "common_utilities/transform_util.hpp"
#include <drone_msgs/msg/point_list.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "def_slam.hpp"

#include "filter/base_filter.hpp"
#include "filter/extended_kalman_filter.hpp"
#include "filter/unscented_kalman_filter.hpp"
#include "filter/fast_slam.hpp"

#include "association/base_association.hpp"
#include "association/joint_compatibility_association.hpp"
#include "association/nearest_neighbor_association.hpp"

typedef std::shared_ptr<BaseFilter> FilterPtr;
typedef std::shared_ptr<BaseAssociation> AssociationPtr;

class SlamManager : public rclcpp::Node {
public:
  SlamManager();
private:
  void createSubscribers();
  void createPublishers();
  void initialize();
  void filterCallback(const Map& map);
  void associationCallback(const Measurements& meas);
  void droneOdometryCallback(const px4_msgs::msg::VehicleOdometry odometry);
  void featureDetectionCallback(const drone_msgs::msg::PointList features);
  void publishMap(const Map& map);

private:
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _droneOdometrySubscriber;
  rclcpp::Subscription<drone_msgs::msg::PointList>::SharedPtr _feature3DcoordinatSubscriber;
  FilterPtr _filter;
  AssociationPtr _associantion;
};

#endif  // SLAM__MANAGER_HPP_