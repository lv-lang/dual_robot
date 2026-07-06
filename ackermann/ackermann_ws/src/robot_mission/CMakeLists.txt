cmake_minimum_required(VERSION 3.8)
project(robot_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_library(robot_mission_executor_lib
  src/mission_executor_node.cpp
)
ament_target_dependencies(robot_mission_executor_lib
  nav2_msgs
  rclcpp
  rclcpp_action
  robot_interfaces
)
target_include_directories(robot_mission_executor_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_executable(mission_executor_node
  src/mission_executor_node_main.cpp
)
target_link_libraries(mission_executor_node robot_mission_executor_lib)
ament_target_dependencies(mission_executor_node
  nav2_msgs
  rclcpp
  rclcpp_action
  robot_interfaces
)

add_library(robot_health_monitor_lib
  src/robot_health_monitor_node.cpp
)
ament_target_dependencies(robot_health_monitor_lib
  lifecycle_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  sensor_msgs
  tf2
  tf2_ros
)
target_include_directories(robot_health_monitor_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_executable(robot_health_monitor_node
  src/robot_health_monitor_node_main.cpp
)
target_link_libraries(robot_health_monitor_node robot_health_monitor_lib)
ament_target_dependencies(robot_health_monitor_node
  lifecycle_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS
  robot_mission_executor_lib
  robot_health_monitor_lib
  mission_executor_node
  robot_health_monitor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mission_executor_node test/test_mission_executor_node.cpp)
  target_link_libraries(test_mission_executor_node robot_mission_executor_lib)
  ament_target_dependencies(test_mission_executor_node
    nav2_msgs
    rclcpp
    rclcpp_action
    robot_interfaces
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(robot_mission_executor_lib robot_health_monitor_lib)
ament_export_dependencies(
  lifecycle_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  robot_interfaces
  sensor_msgs
  tf2
  tf2_ros
)

ament_package()
