#ifndef SLAM_VISUALIZATION_HPP_
#define SLAM_VISUALIZATION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <drone_msgs/msg/map_summary.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class SlamVisualization : public rclcpp::Node
{
public:
    SlamVisualization();

private:
    void mapCallback(const drone_msgs::msg::MapSummary::SharedPtr msg);
    void broadcastTransform();

    rclcpp::Subscription<drone_msgs::msg::MapSummary>::SharedPtr _mapSubscription;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _markerPublisher;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tfBroadcaster;
    rclcpp::TimerBase::SharedPtr _timer;
};

#endif // SLAM_VISUALIZATION_HPP_