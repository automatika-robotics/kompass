cmake_minimum_required(VERSION 3.8)
project(kompass_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


set(msg_files
  "msg/map/Obstacles.msg"

  "msg/motion/PathTrackingError.msg"
  "msg/motion/PathTrackingErrorArray.msg"
  "msg/motion/FollowPathRequest.msg"
  "msg/motion/VectorArray.msg"
  "msg/motion/TwistArray.msg"
  "msg/motion/MotionStatus.msg"

  "msg/prediction/PredictedTrajectories.msg"
  "msg/prediction/PredictedOccupancies.msg"
)

set(action_files
  "action/PlanPath.action"
  "action/FollowPath.action"
  "action/ControlPath.action"
  "action/TrackPath.action"
  "action/TrackVisionTarget.action"
  "action/IntelligentRest.action"
  "action/MotionRecording.action"
)

set(service_files
  "srv/prediction/GetPredictedOccupancies.srv"
  "srv/prediction/GetPredictedTrajectories.srv"

  "srv/motion/StartPathRecording.srv"
  "srv/motion/PathFromToFile.srv"
  "srv/motion/GetCmdList.srv"
  "srv/motion/PlanPath.srv"

  "srv/control/StopVisionTracking.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  ${service_files}
  DEPENDENCIES builtin_interfaces std_msgs nav_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
