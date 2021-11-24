cmake_minimum_required(VERSION 3.5)
project(walker_diff_odom)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()


## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(walker_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
  rclcpp 
  std_msgs
  geometry_msgs
  walker_msgs
  nav_msgs
  tf2_ros
  tf2
  )

include_directories(include)
ament_export_include_directories(include)

## Declare cpp executables
# ..................................................................
add_executable(walker_odom_node 
	src/walker_odom_node.cpp
	src/walker_odom.cpp
)

ament_target_dependencies(walker_odom_node ${dependencies})
# ..................................................................


#############
## Install ##
#############

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS walker_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
