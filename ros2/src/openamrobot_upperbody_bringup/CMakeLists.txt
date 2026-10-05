cmake_minimum_required(VERSION 3.8)
project(openamrobot_upperbody_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/openamrobot_upperbody_bringup)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # L01 starts a real controller_manager: it picks its own ROS_DOMAIN_ID (see the test), so it cannot see or be seen by
  # another stack on the machine.
  add_launch_test(test/test_l01_mock_controllers.py TIMEOUT 60)
endif()

ament_package()
