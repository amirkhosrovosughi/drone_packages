cmake_minimum_required(VERSION 3.8)
project(data_logging_utils)

option(DATA_LOGGER_KEEP_HISTORY "Keep per-run history folders under ~/ros_data_logging" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}
  src/data_logger.cpp
)

if(DATA_LOGGER_KEEP_HISTORY)
  target_compile_definitions(${PROJECT_NAME} PUBLIC DATA_LOGGER_KEEP_HISTORY=1)
else()
  target_compile_definitions(${PROJECT_NAME} PUBLIC DATA_LOGGER_KEEP_HISTORY=0)
endif()

include_directories(include)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export targets for find_package
ament_export_targets(${PROJECT_NAME}-targets)
ament_export_dependencies(ament_cmake)
ament_package()
