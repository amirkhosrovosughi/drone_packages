// px4_command_handler_node.cpp

#include "px4_command_handler.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4CommandHandlerNode>());
  rclcpp::shutdown();
  return 0;
}
