cmake_minimum_required(VERSION 3.5)
project(drone_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneDirectionCommand.msg"
  "msg/DetectedFeature.msg"
  "msg/DetectedFeatureList.msg"
  "srv/DroneMode.srv"
)

ament_package()
