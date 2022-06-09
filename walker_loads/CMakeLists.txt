cmake_minimum_required(VERSION 3.5)
project(walker_loads)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(walker_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(partial_loads src/partial_loads_node.cpp src/spline.cpp src/partial_loads.cpp)
ament_target_dependencies(partial_loads rclcpp ament_index_cpp Eigen3 walker_msgs)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

# Install Cpp executables
install(TARGETS
partial_loads
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/gait_monitor_handles.py
  scripts/gait_monitor_speed.py
  scripts/partial_loads.py
  scripts/plot_loads.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional resources
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()