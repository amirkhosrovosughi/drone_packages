// feature_2dto3d_transfer_node.cpp

#include "feature_2dto3d_transfer.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Feature2DTo3DTransfer>());
  rclcpp::shutdown();
  return 0;
}
