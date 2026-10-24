cmake_minimum_required(VERSION 3.5)
project(walker_stability)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(walker_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/user_stability.py
  scripts/arm_posture.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional resources
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_support_polygon test/test_support_polygon.py)
  ament_add_pytest_test(test_stability_model test/test_stability_model.py)
  ament_add_pytest_test(test_arm_posture test/test_arm_posture.py)
endif()

ament_package()