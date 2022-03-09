cmake_minimum_required(VERSION 3.5)
project(walker_arduino)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(walker_msgs REQUIRED)

include_directories(include)

# usb serial parser
add_executable(usb_conn src/USBConn_node.cpp src/USBConn.cpp)
ament_target_dependencies(usb_conn rclcpp walker_msgs)
target_link_libraries(usb_conn serial)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS usb_conn
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
 DESTINATION  share/${PROJECT_NAME}/
)


ament_package()
