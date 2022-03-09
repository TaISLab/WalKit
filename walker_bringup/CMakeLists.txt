cmake_minimum_required(VERSION 3.5)
project(walker_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)


install(DIRECTORY launch param script config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/record_service.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
