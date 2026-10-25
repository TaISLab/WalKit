cmake_minimum_required(VERSION 3.5)

project(walker_laser_filter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python module: the self-collision criterion and the launch helper
ament_python_install_package(${PROJECT_NAME})

# Tool that makes config/self_filter.yaml
install(PROGRAMS
  scripts/find_self_collision_boxes.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launchers and the parameter files of the filters
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_self_collision_py test/test_self_collision.py)
  ament_add_pytest_test(test_configs_py test/test_configs.py)
endif()

ament_package()
