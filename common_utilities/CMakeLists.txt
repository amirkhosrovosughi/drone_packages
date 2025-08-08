cmake_minimum_required(VERSION 3.8)
project(common_utilities)
enable_testing()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_library(${PROJECT_NAME}
  src/transform_util.cpp
)

include_directories(include)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Eigen3::Eigen       
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

ament_add_gtest(transform_util_test test/transform_util_test.cpp)

target_link_libraries(transform_util_test
  common_utilities  # if you have a library target defined below
)

# Export targets for find_package
ament_export_targets(${PROJECT_NAME}-targets)
ament_export_dependencies(Eigen3)
ament_package()
