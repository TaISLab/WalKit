cmake_minimum_required(VERSION 3.5)
project(walker_navigation2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch map param rviz2
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
