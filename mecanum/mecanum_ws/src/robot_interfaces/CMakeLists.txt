cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(MSG_FILES
  "msg/MissionStep.msg"
  "msg/MissionCommand.msg"
  "msg/TaskState.msg"
  "msg/Task.msg"
  "msg/RobotState.msg"
  "msg/RobotHeartbeat.msg"
  "msg/RobotHealth.msg"
  "msg/DispatchLease.msg"
  "msg/SystemState.msg"
  "msg/ResourceLock.msg"
  "msg/TaskPoint.msg"
)

set(SRV_FILES
  "srv/CreateTask.srv"
  "srv/CancelTask.srv"
  "srv/PauseTask.srv"
  "srv/ResumeTask.srv"
  "srv/ConfirmTaskStep.srv"
  "srv/EmergencyStop.srv"
  "srv/GetDispatchState.srv"
  "srv/AddTaskPoint.srv"
  "srv/ClearTemporaryTaskPoints.srv"
  "srv/GetTaskPoints.srv"
  "srv/EnableSystem.srv"
  "srv/RecoverSystem.srv"
)

set(ACTION_FILES
  "action/ExecuteMission.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_generated_interfaces test/test_generated_interfaces.cpp)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(test_generated_interfaces "${cpp_typesupport_target}")
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
