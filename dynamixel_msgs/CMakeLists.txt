cmake_minimum_required(VERSION 3.3)
project(dynamixel_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointState.msg"
  "msg/MotorState.msg"
  "msg/MotorStateList.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

#ament_export_dependencies(rosidl_default_runtime)
ament_package()

