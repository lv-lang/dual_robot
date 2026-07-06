cmake_minimum_required(VERSION 3.8)
project(robot_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

set(ROBOT_PLANNER_DEPS
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

add_library(robot_planner_core
  src/astar_planner.cpp
  src/mpc_path_tracker.cpp
  src/planner_types.cpp
)
ament_target_dependencies(robot_planner_core ${ROBOT_PLANNER_DEPS})

add_executable(astar_global_planner src/astar_global_planner_node.cpp)
target_link_libraries(astar_global_planner robot_planner_core)
ament_target_dependencies(astar_global_planner ${ROBOT_PLANNER_DEPS})

add_executable(mpc_dwb_planner src/mpc_dwb_planner_node.cpp)
target_link_libraries(mpc_dwb_planner robot_planner_core)
ament_target_dependencies(mpc_dwb_planner ${ROBOT_PLANNER_DEPS})

install(TARGETS
  robot_planner_core
  astar_global_planner
  mpc_dwb_planner
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
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_astar_planner test/test_astar_planner.cpp)
  target_link_libraries(test_astar_planner robot_planner_core)
  ament_target_dependencies(test_astar_planner ${ROBOT_PLANNER_DEPS})

  ament_add_gtest(test_mpc_path_tracker test/test_mpc_path_tracker.cpp)
  target_link_libraries(test_mpc_path_tracker robot_planner_core)
  ament_target_dependencies(test_mpc_path_tracker ${ROBOT_PLANNER_DEPS})

  ament_add_pytest_test(test_robot_planner_interfaces test/test_robot_planner_interfaces.py)
endif()

ament_package()
