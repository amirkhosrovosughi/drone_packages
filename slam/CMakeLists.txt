cmake_minimum_required(VERSION 3.8)
project(slam)

# Flag to select SLAM pipeline (exactly one must be enabled)
option(SLAM_EKF "Build with EKF SLAM" OFF)
option(SLAM_GRAPH "Build with Graph SLAM" OFF)
option(SLAM_FAST_SLAM "Build with FastSLAM SLAM (future)" OFF)

# Default pipeline used when no explicit SLAM_* option is enabled.
set(SLAM_DEFAULT_PIPELINE "GRAPH")

set(_slam_pipeline_count 0)
if(SLAM_EKF)
  math(EXPR _slam_pipeline_count "${_slam_pipeline_count} + 1")
endif()
if(SLAM_GRAPH)
  math(EXPR _slam_pipeline_count "${_slam_pipeline_count} + 1")
endif()
if(SLAM_FAST_SLAM)
  math(EXPR _slam_pipeline_count "${_slam_pipeline_count} + 1")
endif()

if(_slam_pipeline_count EQUAL 0)
  if(SLAM_DEFAULT_PIPELINE STREQUAL "EKF")
    set(SLAM_EKF ON)
  elseif(SLAM_DEFAULT_PIPELINE STREQUAL "GRAPH")
    set(SLAM_GRAPH ON)
  elseif(SLAM+_DEFAULT_PIPELINE STREQUAL "FAST_SLAM")
    set(SLAM_FAST_SLAM ON)
  else()
    message(FATAL_ERROR
      "Invalid SLAM_DEFAULT_PIPELINE='${SLAM_DEFAULT_PIPELINE}'. "
      "Expected one of: EKF, GRAPH, FAST_SLAM")
  endif()
  set(_slam_pipeline_count 1)
  message(STATUS
    "No SLAM_* option explicitly enabled. Falling back to "
    "SLAM_DEFAULT_PIPELINE='${SLAM_DEFAULT_PIPELINE}'.")
endif()

if(NOT _slam_pipeline_count EQUAL 1)
  message(FATAL_ERROR
    "At most one pipeline can be enabled explicitly: "
    "SLAM_EKF, SLAM_GRAPH, SLAM_FAST_SLAM")
endif()

if(SLAM_EKF)
  add_compile_definitions(EKF)
elseif(SLAM_GRAPH)
  add_compile_definitions(GRAPH)
elseif(SLAM_FAST_SLAM)
  add_compile_definitions(FAST_SLAM)
  message(STATUS "FAST_SLAM pipeline selected. Current runtime path is not implemented yet; this selection is reserved for future work.")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(STORE_DEBUG_DATA "Enable store debug data" ON)

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
find_package(ament_cmake_gtest REQUIRED)

include_directories(include/${PROJECT_NAME})

set(SOURCES
  src/node/slam_node.cpp
  src/node/slam_manager.cpp
  src/pipeline/pipeline_factory.cpp
  src/pipeline/ekf_slam_pipeline.cpp
  src/pipeline/graph_slam_backend.cpp
  src/pipeline/graph_slam_frontend.cpp
  src/pipeline/graph_slam_pipeline.cpp
  src/pipeline/frontend_health_monitor.cpp
  src/pipeline/optimization_scheduler.cpp
  src/pipeline/optimization_watchdog.cpp
  src/filter/extended_kalman_filter.cpp
  src/graph/internal_graph_optimizer.cpp
  src/map/slam_map.cpp
  src/motion/position_only_motion_model.cpp
  src/observation/observation_builder.cpp
  src/measurement/measurement_factory.cpp
  src/measurement/point3d_measurement_model.cpp
  src/measurement/bbox_measurement_model.cpp
  src/measurement/bearing_measurement_model.cpp
  src/association/joint_compatibility_association.cpp
  src/association/nearest_neighbor_association.cpp
  src/association/ekf_nearest_neighbor_association.cpp
  src/association/graph_nearest_neighbor_association.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

if(STORE_DEBUG_DATA)
  target_compile_definitions(${PROJECT_NAME} PRIVATE STORE_DEBUG_DATA)
endif()

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

# -- Unit tests --------------------------------------------------------------
if(BUILD_TESTING)
  include(CMakeParseArguments)

  function(add_slam_gtest target)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs SOURCES DEPS)
    cmake_parse_arguments(SLAM_TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    ament_add_gtest(${target} ${SLAM_TEST_SOURCES})

    if(TARGET ${target})
      target_include_directories(${target} PRIVATE include)
      target_include_directories(${target} PRIVATE ${EIGEN3_INCLUDE_DIR})
      if(SLAM_TEST_DEPS)
        ament_target_dependencies(${target} ${SLAM_TEST_DEPS})
      endif()
    endif()
  endfunction()

  add_slam_gtest(test_position_only_motion_model
    SOURCES
      test/motion/test_position_only_motion_model.cpp
      src/motion/position_only_motion_model.cpp
  )

  add_slam_gtest(test_pose_motion_model
    SOURCES
      test/motion/test_pose_motion_model.cpp
      src/motion/pose_motion_model.cpp
  )

  add_slam_gtest(test_point3d_measurement_model
    SOURCES
      test/measurement/test_point3d_measurement_model.cpp
      src/measurement/point3d_measurement_model.cpp
  )

  add_slam_gtest(test_measurement_factory
    SOURCES
      test/measurement/test_measurement_factory.cpp
      src/measurement/measurement_factory.cpp
      src/measurement/point3d_measurement_model.cpp
      src/measurement/bbox_measurement_model.cpp
      src/measurement/bearing_measurement_model.cpp
  )

  add_slam_gtest(test_observation_builder
    SOURCES
      test/observation/test_observation_builder.cpp
      src/observation/observation_builder.cpp
    DEPS
      drone_msgs
      vision_msgs
      geometry_msgs
  )

  add_slam_gtest(test_bearing_measurement_model
    SOURCES
      test/measurement/test_bearing_measurement_model.cpp
      src/measurement/bearing_measurement_model.cpp
  )

  add_slam_gtest(test_ekf_nearest_neighbor_association
    SOURCES
      test/association/test_ekf_nearest_neighbor_association.cpp
      src/association/nearest_neighbor_association.cpp
      src/association/ekf_nearest_neighbor_association.cpp
      src/measurement/point3d_measurement_model.cpp
      src/measurement/bearing_measurement_model.cpp
  )

  add_slam_gtest(test_graph_nearest_neighbor_association
    SOURCES
      test/association/test_graph_nearest_neighbor_association.cpp
      src/association/nearest_neighbor_association.cpp
      src/association/graph_nearest_neighbor_association.cpp
      src/measurement/point3d_measurement_model.cpp
      src/measurement/bearing_measurement_model.cpp
  )

  add_slam_gtest(test_slam_map
    SOURCES
      test/map/test_slam_map.cpp
      src/map/slam_map.cpp
  )

  add_slam_gtest(test_extended_kalman_filter
    SOURCES
      test/filter/test_extended_kalman_filter.cpp
      src/filter/extended_kalman_filter.cpp
      src/map/slam_map.cpp
      src/motion/position_only_motion_model.cpp
      src/measurement/point3d_measurement_model.cpp
  )

  add_slam_gtest(test_ekf_slam_pipeline
    SOURCES
      test/pipeline/test_ekf_slam_pipeline.cpp
      src/pipeline/ekf_slam_pipeline.cpp
      src/filter/extended_kalman_filter.cpp
      src/association/nearest_neighbor_association.cpp
      src/association/ekf_nearest_neighbor_association.cpp
      src/measurement/measurement_factory.cpp
      src/measurement/point3d_measurement_model.cpp
      src/measurement/bbox_measurement_model.cpp
      src/measurement/bearing_measurement_model.cpp
      src/motion/position_only_motion_model.cpp
      src/map/slam_map.cpp
  )

  add_slam_gtest(test_graph_slam_frontend
    SOURCES
      test/pipeline/test_graph_slam_frontend.cpp
      src/pipeline/graph_slam_frontend.cpp
      src/pipeline/frontend_health_monitor.cpp
      src/measurement/measurement_factory.cpp
      src/measurement/point3d_measurement_model.cpp
      src/measurement/bbox_measurement_model.cpp
      src/measurement/bearing_measurement_model.cpp
  )

  add_slam_gtest(test_graph_slam_backend
    SOURCES
      test/pipeline/test_graph_slam_backend.cpp
      src/pipeline/graph_slam_backend.cpp
  )

  add_slam_gtest(test_graph_slam_pipeline
    SOURCES
      test/pipeline/test_graph_slam_pipeline.cpp
      src/pipeline/graph_slam_pipeline.cpp
      src/pipeline/frontend_health_monitor.cpp
      src/pipeline/graph_slam_backend.cpp
      src/pipeline/graph_slam_frontend.cpp
      src/pipeline/optimization_scheduler.cpp
      src/pipeline/optimization_watchdog.cpp
      src/graph/internal_graph_optimizer.cpp
      src/measurement/measurement_factory.cpp
      src/measurement/point3d_measurement_model.cpp
      src/measurement/bbox_measurement_model.cpp
      src/measurement/bearing_measurement_model.cpp
  )

  add_slam_gtest(test_optimization_scheduler
    SOURCES
      test/pipeline/test_optimization_scheduler.cpp
      src/pipeline/optimization_scheduler.cpp
  )

  add_slam_gtest(test_optimization_watchdog
    SOURCES
      test/pipeline/test_optimization_watchdog.cpp
      src/pipeline/optimization_watchdog.cpp
  )

  add_slam_gtest(test_internal_graph_optimizer
    SOURCES
      test/graph/test_internal_graph_optimizer.cpp
      src/graph/internal_graph_optimizer.cpp
  )
endif()
