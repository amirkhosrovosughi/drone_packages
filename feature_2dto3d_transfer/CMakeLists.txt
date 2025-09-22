cmake_minimum_required(VERSION 3.8)
project(feature_2dto3d_transfer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(STORE_DEBUG_DATA "Enable store debug data" OFF)

if(STORE_DEBUG_DATA)
    add_compile_definitions(STORE_DEBUG_DATA)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common_utilities REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(data_logging_utils REQUIRED)

include_directories(include/${PROJECT_NAME})

set(SOURCES
  src/feature_2dto3d_transfer.cpp
  src/feature_2dto3d_transfer_node.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  drone_msgs
  geometry_msgs
  sensor_msgs
  common_utilities
  tf2
  tf2_ros
  data_logging_utils
  px4_msgs)

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
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
