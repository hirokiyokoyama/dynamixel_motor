cmake_minimum_required(VERSION 3.5)
project(dynamixel_controllers)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(dynamixel_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
# actionlib
# dynamixel_driver

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RestartController.srv"
  "srv/SetComplianceMargin.srv"
  "srv/SetCompliancePunch.srv"
  "srv/SetComplianceSlope.srv"
  "srv/SetSpeed.srv"
  "srv/SetTorqueLimit.srv"
  "srv/StartController.srv"
  "srv/StopController.srv"
  "srv/TorqueEnable.srv"
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  trajectory_msgs
  diagnostic_msgs
  dynamixel_msgs
  control_msgs
  rcl_interfaces
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  nodes/controller_manager.py
  nodes/controller_spawner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

