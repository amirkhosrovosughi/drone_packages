#ifndef SLAM__MANAGER_HPP_
#define SLAM__MANAGER_HPP_

#include "filter/base_filter.hpp"
#include "filter/extended_kalman_filter.hpp"
#include "filter/unscented_kalman_filter.hpp"
#include "filter/fast_slam.hpp"

#include <rclcpp/rclcpp.hpp>

class SlamManager : public rclcpp::Node {
public:
  SlamManager();
private:
  // Add your class members and methods here
  void createSubscribers();
  void createPublishers();

private:
  std::shared_ptr<BaseFilter> _filter;
};

#endif  // SLAM__MANAGER_HPP_