#include "slam_visualization.hpp"

SlamVisualization::SlamVisualization()
    : Node("slam_visualization")
{
    _mapSubscription = this->create_subscription<drone_msgs::msg::MapSummary>(
        "/slam/map", 10, std::bind(&SlamVisualization::mapCallback, this, std::placeholders::_1));
    _markerPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker", 10);

    // Periodically broadcast the transform
    _timer = this->create_wall_timer(
        std::chrono::milliseconds(1000), std::bind(&SlamVisualization::broadcastTransform, this));
}

void SlamVisualization::broadcastTransform()
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "map2";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    // _tfBroadcaster->sendTransform(t);
}

void SlamVisualization::mapCallback(const drone_msgs::msg::MapSummary::SharedPtr msg)
{
    visualization_msgs::msg::MarkerArray marker_array;

    // Robot Marker
    visualization_msgs::msg::Marker robot_marker;
    robot_marker.header.frame_id = "map";
    robot_marker.header.stamp = this->get_clock()->now();
    robot_marker.ns = "robot";
    robot_marker.id = 0;
    robot_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    robot_marker.action = visualization_msgs::msg::Marker::ADD;
    robot_marker.pose.position.x = msg->robot.pose.position.x;
    robot_marker.pose.position.y = msg->robot.pose.position.y;
    robot_marker.pose.position.z = 0;
    robot_marker.scale.x = 0.5;
    robot_marker.scale.y = 0.5;
    robot_marker.scale.z = 0.1;
    robot_marker.color.a = 1.0;
    robot_marker.color.r = 0.0;
    robot_marker.color.g = 1.0;
    robot_marker.color.b = 0.0;
    marker_array.markers.push_back(robot_marker);

    // Robot Orientation Marker (Yaw)
    double roll, pitch, yaw;
    tf2::Quaternion q(
        msg->robot.pose.orientation.x,
        msg->robot.pose.orientation.y,
        msg->robot.pose.orientation.z,
        msg->robot.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    visualization_msgs::msg::Marker orientation_marker;
    orientation_marker.header.frame_id = "map";
    orientation_marker.header.stamp = this->get_clock()->now();
    orientation_marker.ns = "robot_orientation";
    orientation_marker.id = 1;
    orientation_marker.type = visualization_msgs::msg::Marker::ARROW;
    orientation_marker.action = visualization_msgs::msg::Marker::ADD;
    orientation_marker.pose.position.x = msg->robot.pose.position.x;
    orientation_marker.pose.position.y = msg->robot.pose.position.y;
    orientation_marker.pose.position.z = 0.05;
    orientation_marker.scale.x = 0.5;
    orientation_marker.scale.y = 0.1;
    orientation_marker.scale.z = 0.1;
    orientation_marker.color.a = 1.0;
    orientation_marker.color.r = 1.0;
    orientation_marker.color.g = 0.0;
    orientation_marker.color.b = 0.0;

    tf2::Quaternion orientation_quat;
    orientation_quat.setRPY(0, 0, yaw);
    orientation_marker.pose.orientation.x = orientation_quat.x();
    orientation_marker.pose.orientation.y = orientation_quat.y();
    orientation_marker.pose.orientation.z = orientation_quat.z();
    orientation_marker.pose.orientation.w = orientation_quat.w();
    marker_array.markers.push_back(orientation_marker);

    // Robot Variance Marker
    visualization_msgs::msg::Marker robot_variance_marker;
    robot_variance_marker.header.frame_id = "map";
    robot_variance_marker.header.stamp = this->get_clock()->now();
    robot_variance_marker.ns = "robot_variance";
    robot_variance_marker.id = 2;
    robot_variance_marker.type = visualization_msgs::msg::Marker::SPHERE;
    robot_variance_marker.action = visualization_msgs::msg::Marker::ADD;
    robot_variance_marker.pose.position.x = msg->robot.pose.position.x;
    robot_variance_marker.pose.position.y = msg->robot.pose.position.y;
    robot_variance_marker.pose.position.z = 0;
    robot_variance_marker.scale.x = sqrt(msg->robot.variance.xx);
    robot_variance_marker.scale.y = sqrt(msg->robot.variance.yy);
    robot_variance_marker.scale.z = 0.1;
    robot_variance_marker.color.a = 0.5;
    robot_variance_marker.color.r = 0.0;
    robot_variance_marker.color.g = 0.0;
    robot_variance_marker.color.b = 1.0;
    marker_array.markers.push_back(robot_variance_marker);

    // Landmarks
    int id = 3;
    for (const auto& landmark : msg->landmarks)
    {
        // Landmark Marker
        visualization_msgs::msg::Marker landmark_marker;
        landmark_marker.header.frame_id = "map";
        landmark_marker.header.stamp = this->get_clock()->now();
        landmark_marker.ns = "landmarks";
        landmark_marker.id = id++;
        landmark_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        landmark_marker.action = visualization_msgs::msg::Marker::ADD;
        landmark_marker.pose.position.x = landmark.position.x;
        landmark_marker.pose.position.y = landmark.position.y;
        landmark_marker.pose.position.z = 0;
        landmark_marker.scale.x = 0.2;
        landmark_marker.scale.y = 0.2;
        landmark_marker.scale.z = 0.1;
        landmark_marker.color.a = 1.0;
        landmark_marker.color.r = 1.0;
        landmark_marker.color.g = 0.0;
        landmark_marker.color.b = 0.0;
        marker_array.markers.push_back(landmark_marker);

        // Landmark Variance Marker
        visualization_msgs::msg::Marker landmark_variance_marker;
        landmark_variance_marker.header.frame_id = "map";
        landmark_variance_marker.header.stamp = this->get_clock()->now();
        landmark_variance_marker.ns = "landmark_variance";
        landmark_variance_marker.id = id++;
        landmark_variance_marker.type = visualization_msgs::msg::Marker::SPHERE;
        landmark_variance_marker.action = visualization_msgs::msg::Marker::ADD;
        landmark_variance_marker.pose.position.x = landmark.position.x;
        landmark_variance_marker.pose.position.y = landmark.position.y;
        landmark_variance_marker.pose.position.z = 0;
        landmark_variance_marker.scale.x = sqrt(landmark.variance.xx);
        landmark_variance_marker.scale.y = sqrt(landmark.variance.yy);
        landmark_variance_marker.scale.z = 0.1;
        landmark_variance_marker.color.a = 0.5;
        landmark_variance_marker.color.r = 0.0;
        landmark_variance_marker.color.g = 0.0;
        landmark_variance_marker.color.b = 1.0;
        marker_array.markers.push_back(landmark_variance_marker);

        // Landmark ID Text Marker
        visualization_msgs::msg::Marker landmark_id_marker;
        landmark_id_marker.header.frame_id = "map";
        landmark_id_marker.header.stamp = this->get_clock()->now();
        landmark_id_marker.ns = "landmark_ids";
        landmark_id_marker.id = id++;
        landmark_id_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        landmark_id_marker.action = visualization_msgs::msg::Marker::ADD;
        landmark_id_marker.pose.position.x = landmark.position.x;
        landmark_id_marker.pose.position.y = landmark.position.y;
        landmark_id_marker.pose.position.z = 0.5; // slightly above the landmark
        landmark_id_marker.scale.z = 0.2; // height of the text
        landmark_id_marker.color.a = 1.0;
        landmark_id_marker.color.r = 1.0;
        landmark_id_marker.color.g = 1.0;
        landmark_id_marker.color.b = 1.0;
        landmark_id_marker.text = std::to_string(landmark.id);
        marker_array.markers.push_back(landmark_id_marker);
    }

    _markerPublisher->publish(marker_array);
}
