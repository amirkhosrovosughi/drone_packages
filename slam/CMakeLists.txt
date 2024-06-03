cmake_minimum_required(VERSION 3.8)
project(slam)

# chooose flag to select the type of FILTER
add_definitions(-DEKF)
# add_definitions(-DUKF)
# add_definitions(-DFAST_SLAM)

# chooose flag to select the type of ASSOSSIATION
add_definitions(-DNEAREST_NEIGHBOR)
# add_definitions(-DJOINT_COMPATIBILITY)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(DEBUG_FEATURE "Enable debug features" ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(common_utilities REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

include_directories(include/${PROJECT_NAME})

set(SOURCES
  src/slam_node.cpp
  src/slam_manager.cpp
  src/filter/extended_kalman_filter.cpp
  src/filter/unscented_kalman_filter.cpp
  src/filter/fast_slam.cpp
  src/association/joint_compatibility_association.cpp
  src/association/nearest_neighbor_association.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  drone_msgs
  common_utilities
  px4_msgs
)

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
