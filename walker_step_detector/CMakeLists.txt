cmake_minimum_required(VERSION 3.5)

project(walker_step_detector)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (OpenCV_DIR /usr/local/include)
include_directories(include ${OpenCV_INCLUDE_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(walker_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# detect_steps executable
add_executable (detect_steps
                src/detect_steps.cpp
                src/laser_processor.cpp
                src/cluster_features.cpp
                include/walker_step_detector/laser_processor.h
                include/walker_step_detector/cluster_features.h
                )
ament_target_dependencies(detect_steps rclcpp walker_msgs OpenCV sensor_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs) 
  
install (TARGETS
detect_steps
  DESTINATION lib/${PROJECT_NAME}
)

# plot_steps executable
add_executable (plot_steps
                src/plot_steps.cpp
                )
ament_target_dependencies(plot_steps rclcpp walker_msgs visualization_msgs) 
  
install (TARGETS
plot_steps
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
