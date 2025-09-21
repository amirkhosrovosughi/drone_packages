cmake_minimum_required(VERSION 3.5)
project(px4_command_handler)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(common_utilities REQUIRED)

include_directories(include/${PROJECT_NAME})

set(SOURCES
  src/px4_command_handler.cpp
  src/px4_command_handler_node.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  px4_msgs
  drone_msgs
  geometry_msgs
  sensor_msgs
  common_utilities)

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
