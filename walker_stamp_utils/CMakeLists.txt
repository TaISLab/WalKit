cmake_minimum_required(VERSION 3.8)
project(walker_stamp_utils)

find_package(ament_cmake REQUIRED)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stamp_sequencer test/test_stamp_sequencer.cpp)
  target_include_directories(test_stamp_sequencer PRIVATE include)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_package()
