cmake_minimum_required(VERSION 3.8)
project(openamrobot_upperbody_gazebo)
find_package(ament_cmake REQUIRED)
install(DIRECTORY worlds models launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # G01 starts a real Gazebo server and a controller manager. It picks its own ROS_DOMAIN_ID and GZ_PARTITION (see the test),
  # so it cannot see or be seen by another stack or simulator on the machine. 90 s is the budget for startup and every assertion.
  add_launch_test(test/test_g01_gazebo_smoke.py TIMEOUT 90)
  # G02 guards the finger-mimic decision (README). It passes the Bullet-Featherstone engine explicitly: the launch defaults to DART, on which it fails.
  add_launch_test(test/test_g02_gripper_mimic.py TIMEOUT 90)
endif()

ament_package()
