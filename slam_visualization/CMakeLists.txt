cmake_minimum_required(VERSION 3.8)
project(slam_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(drone_msgs REQUIRED) 

include_directories(include/${PROJECT_NAME})

set(SOURCES
  src/slam_visualization.cpp
  src/slam_visualization_node.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  visualization_msgs
  drone_msgs
  tf2
  tf2_geometry_msgs)

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
