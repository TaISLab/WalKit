cmake_minimum_required(VERSION 3.5.0)
project(stability_grid_map)

## Find ament_cmake macros and libraries
find_package(rclcpp REQUIRED)
find_package(walker_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake REQUIRED)

# find_package(Boost REQUIRED) # random

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC)
endif()

set(dependencies
#  Boost
  grid_map_msgs
  grid_map_ros
  walker_msgs
  tf2 
  tf2_geometry_msgs
  tf2_ros
  rclcpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp executable
add_executable(
  simple_demo
  src/simple_demo_node.cpp
)

add_library(stability_grid_map_lib SHARED
  src/stability_grid_map.cpp
)

target_link_libraries(simple_demo stability_grid_map_lib)

## Specify libraries to link a library or executable target against
ament_target_dependencies(
  stability_grid_map_lib SYSTEM
  ${dependencies} 
)

ament_target_dependencies(
  simple_demo SYSTEM
  ${dependencies} 
)


set(targets_list
  simple_demo
  stability_grid_map_lib
)


foreach(target ${targets_list})
  if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${target} PRIVATE "SHELL:--param ggc-min-expand=1")
    target_compile_options(${target} PRIVATE "SHELL:--param ggc-min-heapsize=32768")
  endif()
endforeach()

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${targets_list}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark other files for installation
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  # Linting is setup this way to add a filter
  # to ament_cpplint to ignore the lack of
  # copyright messages at the top of files.
  # Copyright messages are being checked for by both
  # ament_cmake_cpplint & ament_cmake_copyright.

  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    # exclude copyright checks
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_cpplint
      ament_cmake_copyright
    )
    ament_lint_auto_find_test_dependencies()

    # run cpplint without copyright filter
    find_package(ament_cmake_cpplint)
    ament_cpplint(
      FILTERS -legal/copyright -build/include_order
    )
  endif()
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})
ament_package()
