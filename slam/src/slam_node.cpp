#include "slam_manager.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamManager>());
  rclcpp::shutdown();
  return 0;
}