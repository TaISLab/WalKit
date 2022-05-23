cmake_minimum_required(VERSION 3.5)
project(walker_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY urdf rviz2 models launch config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})


# Install Python executables
install(PROGRAMS
  scripts/handle_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
