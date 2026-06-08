cmake_minimum_required(VERSION 3.8)
project(locomotion_ros2_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AdapterStatus.msg"
  "msg/BodyPoseCommand.msg"
  "msg/Footstep.msg"
  "msg/FootstepPlan.msg"
  "msg/LocomotionCommand.msg"
  "msg/RobotProfile.msg"
  "msg/SafetyState.msg"
  "msg/SemanticAction.msg"
  "msg/WalkingState.msg"
  "srv/ClearFault.srv"
  "srv/EmergencyStop.srv"
  "srv/GetRobotProfile.srv"
  "srv/SetLocomotionMode.srv"
  "action/ExecuteBodyPose.action"
  "action/ExecuteFootstepPlan.action"
  "action/ExecuteSemanticAction.action"
  "action/ExecuteVelocity.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
