// visual_feature_extraction_node.cpp

#include "visual_feature_extraction.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualFeatureExtraction>());
  rclcpp::shutdown();
  return 0;
}