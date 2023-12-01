// keyboard_control_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "keyboard_control.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardControl>());
    rclcpp::shutdown();
    return 0;
}
