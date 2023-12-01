cmake_minimum_required(VERSION 3.5)
project(keyboard_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drone_msgs REQUIRED)

include_directories(include/keyboard_control)

add_library(${PROJECT_NAME} SHARED
    src/keyboard_control.cpp
)

add_executable(${PROJECT_NAME}_node
    src/keyboard_control_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    drone_msgs
)

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
