#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "yaml-cpp/yaml.h"

// OBSOLETE: Do not use this node, instead use ros_gz_bridge to provide camera_info from gz simulator directly.

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("camera_info_publisher");

  auto publisher =
    node->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 10);

  // Read camera info from YAML file
  std::string package_path = ament_index_cpp::get_package_prefix("camera_info_publisher");
  const std::string yaml_file_path = package_path + "/share/camera_info_publisher/config/camera_info.yaml";

  std::cout << "yaml file path is: " << yaml_file_path << "\n";

  YAML::Node yaml_config = YAML::LoadFile(yaml_file_path);

  sensor_msgs::msg::CameraInfo camera_info_msg;
  // Set camera_info_msg parameters from the YAML file
  
  // Set header values
  camera_info_msg.header.stamp.sec = 0;
  camera_info_msg.header.stamp.nanosec = 0;
  camera_info_msg.header.frame_id = yaml_config["header"]["frame_id"].as<std::string>();
  

  // Set other parameters
  camera_info_msg.height = yaml_config["height"].as<int>();
  camera_info_msg.width = yaml_config["width"].as<int>();
  camera_info_msg.distortion_model = yaml_config["distortion_model"].as<std::string>();

  // Set D array
  camera_info_msg.d.resize(yaml_config["D"].size());
  for (std::size_t i = 0; i < yaml_config["D"].size(); ++i)
  {
    camera_info_msg.d[i] = yaml_config["D"][i].as<double>();
  }

  // Set K array
  for (std::size_t i = 0; i < yaml_config["K"].size(); ++i)
  {
    camera_info_msg.k[i] = yaml_config["K"][i].as<double>();
  }

  // Set R array
  for (std::size_t i = 0; i < yaml_config["R"].size(); ++i)
  {
    camera_info_msg.r[i] = yaml_config["R"][i].as<double>();
  }

  // Set P array
  for (std::size_t i = 0; i < yaml_config["P"].size(); ++i)
  {
    camera_info_msg.p[i] = yaml_config["P"][i].as<double>();
  }

  rclcpp::WallRate loop_rate(1);  // 1 Hz
  while (rclcpp::ok()) {
    publisher->publish(camera_info_msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
