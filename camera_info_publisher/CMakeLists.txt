cmake_minimum_required(VERSION 3.8)
project(camera_info_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(camera_info_publisher src/camera_info_publisher.cpp)
ament_target_dependencies(camera_info_publisher rclcpp sensor_msgs)

target_link_libraries(camera_info_publisher
  yaml-cpp
)

install(TARGETS
  camera_info_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install the YAML file
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()