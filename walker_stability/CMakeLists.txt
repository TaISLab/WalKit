cmake_minimum_required(VERSION 3.5)
project(walker_stability)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(walker_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/walker_stability.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional resources
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()