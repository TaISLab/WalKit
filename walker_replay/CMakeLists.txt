cmake_minimum_required(VERSION 3.5)
project(walker_replay)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python modules (walker_replay.test_config, .bag, .bagio, .session, .launch_utils)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/replay_context.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_replay test/test_replay.py)
  ament_add_pytest_test(test_bagio test/test_bagio.py)
  ament_add_pytest_test(test_session test/test_session.py)
endif()

ament_package()
