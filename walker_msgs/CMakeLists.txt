cmake_minimum_required(VERSION 3.5)
project(walker_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncoderStamped.msg"
  "msg/ForceStamped.msg"
  "msg/Step.msg"
  "msg/StepStamped.msg"
  "msg/StepArray.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
 )
 
 ament_export_dependencies(rosidl_default_runtime)

 # At the bottom of the file:
ament_package()
