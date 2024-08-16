#include "slam_visualization.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamVisualization>());
    rclcpp::shutdown();
    return 0;
}