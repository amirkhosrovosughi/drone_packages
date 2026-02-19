cmake_minimum_required(VERSION 3.8)
project(slam)

# Flag to select the type of FILTER
add_definitions(-DEKF)
# add_definitions(-DGRAPH)
# add_definitions(-DFAST_SLAM)

# Flag to select the type of ASSOSSIATION
add_definitions(-DNEAREST_NEIGHBOR)
# add_definitions(-DJOINT_COMPATIBILITY)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(POSITION_MOTION_POSITION_MEASUREMENT "position motion model for odometry" ON)
option(POSE_MOTION_POSITION_MEASUREMENT "pose motion model for odometry" OFF)

option(POSITION_MOTION_2D_MEASUREMENT "position measrement model" OFF)
option(POSE_MOTION_2D_MEASUREMENT "2D measrement model" OFF)

if(POSITION_MOTION_POSITION_MEASUREMENT)
    add_definitions(-DPOSITION_MOTION_POSITION_MEASUREMENT)
endif()

if(POSE_MOTION_POSITION_MEASUREMENT)
    add_definitions(-DPOSE_MOTION_POSITION_MEASUREMENT)
endif()

if(POSITION_MOTION_2D_MEASUREMENT)
    add_definitions(-DPOSITION_MOTION_2D_MEASUREMENT)
endif()

if(POSE_MOTION_2D_MEASUREMENT)
    add_definitions(-DPOSE_MOTION_2D_MEASUREMENT)
endif()

option(STORE_DEBUG_DATA "Enable store debug data" OFF)

if(STORE_DEBUG_DATA)
    add_compile_definitions(STORE_DEBUG_DATA)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(common_utilities REQUIRED)
find_package(data_logging_utils REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(include/${PROJECT_NAME})

set(SOURCES
  src/node/slam_node.cpp
  src/node/slam_manager.cpp
  src/backend/ekf_slam_backend.cpp
  src/filter/extended_kalman_filter.cpp
  src/motion/position_only_motion_model.cpp
  src/observation/observation_builder.cpp
  src/measurement/measurement_factory.cpp
  src/measurement/point3d_measurement_model.cpp
  src/measurement/bbox_measurement_model.cpp
  src/measurement/bearing_measurement_model.cpp
  src/association/joint_compatibility_association.cpp
  src/association/nearest_neighbor_association.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  drone_msgs
  common_utilities
  data_logging_utils
  px4_msgs
  tf2
  tf2_ros
  sensor_msgs
  vision_msgs
)

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
