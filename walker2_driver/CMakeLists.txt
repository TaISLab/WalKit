cmake_minimum_required(VERSION 3.5)
project(walker2_driver)

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
find_package(rclcpp REQUIRED)
find_package(walker2_msgs REQUIRED)

include_directories(include)

# usb serial parser
add_executable(usb_conn src/USBConn_node.cpp src/USBConn.cpp)
ament_target_dependencies(usb_conn rclcpp walker2_msgs)
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
