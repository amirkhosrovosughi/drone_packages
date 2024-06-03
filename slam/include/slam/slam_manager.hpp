#ifndef SLAM__MANAGER_HPP_
#define SLAM__MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>

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

private:
  FilterPtr _filter;
  AssociationPtr _associantion;
};

#endif  // SLAM__MANAGER_HPP_