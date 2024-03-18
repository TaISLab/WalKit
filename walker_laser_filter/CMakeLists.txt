cmake_minimum_required(VERSION 3.5)

project(walker_laser_filter)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_export_include_directories(include)
ament_export_dependencies(
    rclcpp
)

# Include Cpp "include" directory
include_directories(include)

# box filter  ..................
add_executable (box_filter
                src/box_filter.cpp
                )

target_link_libraries(box_filter ${catkin_LIBRARIES})

ament_target_dependencies(box_filter
  rclcpp 
  sensor_msgs 
) 

install (TARGETS
box_filter
  DESTINATION lib/${PROJECT_NAME}
)

# mask filter  ..................
add_executable (mask_filter
                src/mask_filter.cpp
                )

target_link_libraries(mask_filter ${catkin_LIBRARIES})

ament_target_dependencies(mask_filter
  rclcpp 
  sensor_msgs 
) 

install (TARGETS
mask_filter
  DESTINATION lib/${PROJECT_NAME}
)


# Install launchers and configs
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
