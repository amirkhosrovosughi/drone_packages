cmake_minimum_required(VERSION 3.8)
project(visual_feature_extraction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(DEBUG_FEATURE "Enable debug features" ON)
option(DEEP_DETECTION "Enable deep learning based detection" ON)

if(DEBUG_FEATURE)
    add_compile_definitions(DEBUG_FEATURE)
endif()

if(DEEP_DETECTION)
  # Flag to select the type of detection
  add_definitions(-DDEEP_DETECTION)

  set(ONNXRUNTIME_ROOT "$ENV{HOME}/onnxruntime")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(vision_msgs REQUIRED)

set(SOURCES
  src/visual_feature_extraction.cpp
  src/visual_feature_extraction_node.cpp
  src/feature_extract_classic.cpp
)

if(DEEP_DETECTION)
  list(APPEND SOURCES src/feature_extract_deep.cpp)
endif()

if(DEEP_DETECTION)
  include_directories(include/${PROJECT_NAME}
      ${ONNXRUNTIME_ROOT}/include)

  link_directories(
      ${ONNXRUNTIME_ROOT}/lib
  )
endif()

add_executable(${PROJECT_NAME} ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

if(DEEP_DETECTION)
  target_link_libraries(${PROJECT_NAME}
    onnxruntime
    ${OpenCV_LIBS}
  )
endif()

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  drone_msgs
  message_filters
  vision_msgs)

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

if(DEEP_DETECTION)
  install(
    DIRECTORY model/
    DESTINATION share/${PROJECT_NAME}/model
  )
endif()

install(
  DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

