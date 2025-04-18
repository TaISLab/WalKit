cmake_minimum_required(VERSION 3.5)
project(walker_simulation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
endif()
 
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(turtlebot3_gazebo REQUIRED)

ament_export_dependencies(gazebo_ros)
ament_export_dependencies(turtlebot3_gazebo)


install(DIRECTORY launch  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
