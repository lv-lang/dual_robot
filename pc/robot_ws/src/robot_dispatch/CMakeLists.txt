cmake_minimum_required(VERSION 3.8)
project(robot_dispatch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

set(ROBOT_DISPATCH_DEPS
  ament_index_cpp
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  robot_interfaces
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

add_library(robot_dispatch_core
  src/command_parser.cpp
  src/dispatch_core.cpp
  src/dispatch_types.cpp
)
target_include_directories(robot_dispatch_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_library(robot_dispatch_config
  src/task_points.cpp
)
target_link_libraries(robot_dispatch_config
  robot_dispatch_core
  yaml-cpp
)
target_include_directories(robot_dispatch_config PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_library(robot_dispatch_markers
  src/marker_builder.cpp
)
target_link_libraries(robot_dispatch_markers
  robot_dispatch_core
  robot_dispatch_config
)
ament_target_dependencies(robot_dispatch_markers
  geometry_msgs
  std_msgs
  visualization_msgs
)
target_include_directories(robot_dispatch_markers PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_library(robot_dispatch_node_lib
  src/robot_dispatch_node.cpp
)
target_link_libraries(robot_dispatch_node_lib
  robot_dispatch_core
  robot_dispatch_config
  robot_dispatch_markers
)
ament_target_dependencies(robot_dispatch_node_lib ${ROBOT_DISPATCH_DEPS})

add_executable(robot_dispatch_node
  src/robot_dispatch_node_main.cpp
)
target_link_libraries(robot_dispatch_node robot_dispatch_node_lib)
ament_target_dependencies(robot_dispatch_node ${ROBOT_DISPATCH_DEPS})

add_executable(terminal_dispatch_console
  src/terminal_dispatch_console.cpp
)
target_link_libraries(terminal_dispatch_console
  robot_dispatch_core
  robot_dispatch_config
)
ament_target_dependencies(terminal_dispatch_console ${ROBOT_DISPATCH_DEPS})

add_executable(rviz_task_point_input_node
  src/rviz_task_point_input_node.cpp
)
ament_target_dependencies(rviz_task_point_input_node ${ROBOT_DISPATCH_DEPS})

add_executable(task_marker_publisher
  src/task_marker_publisher.cpp
)
target_link_libraries(task_marker_publisher
  robot_dispatch_config
  robot_dispatch_markers
)
ament_target_dependencies(task_marker_publisher
  ament_index_cpp
  rclcpp
  visualization_msgs
)

install(TARGETS
  robot_dispatch_core
  robot_dispatch_config
  robot_dispatch_markers
  robot_dispatch_node_lib
  robot_dispatch_node
  terminal_dispatch_console
  rviz_task_point_input_node
  task_marker_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_command_parser test/test_command_parser.cpp)
  target_link_libraries(test_command_parser robot_dispatch_core)

  ament_add_gtest(test_dispatch_core test/test_dispatch_core.cpp)
  target_link_libraries(test_dispatch_core robot_dispatch_core)

  ament_add_gtest(test_task_points_and_markers test/test_task_points_and_markers.cpp)
  target_compile_definitions(test_task_points_and_markers
    PRIVATE ROBOT_DISPATCH_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  target_link_libraries(test_task_points_and_markers
    robot_dispatch_config
    robot_dispatch_markers
  )
  ament_target_dependencies(test_task_points_and_markers
    visualization_msgs
  )

  ament_add_gtest(test_robot_dispatch_node_services test/test_robot_dispatch_node_services.cpp)
  target_compile_definitions(test_robot_dispatch_node_services
    PRIVATE ROBOT_DISPATCH_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  target_link_libraries(test_robot_dispatch_node_services robot_dispatch_node_lib)
  ament_target_dependencies(test_robot_dispatch_node_services ${ROBOT_DISPATCH_DEPS})
endif()

ament_export_include_directories(include)
ament_export_libraries(
  robot_dispatch_core
  robot_dispatch_config
  robot_dispatch_markers
  robot_dispatch_node_lib
)
ament_export_dependencies(
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  robot_interfaces
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
  yaml_cpp_vendor
)

ament_package()
