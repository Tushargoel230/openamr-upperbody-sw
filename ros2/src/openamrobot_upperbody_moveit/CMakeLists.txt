cmake_minimum_required(VERSION 3.8)
project(openamrobot_upperbody_moveit)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/regenerate_acm.sh
  scripts/merge_acm.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Static checks of the SRDF and the configuration files: no ROS runtime, no simulator.
  ament_add_pytest_test(srdf test/test_srdf.py TIMEOUT 600)
  ament_add_pytest_test(srdf_import test/test_srdf_import.py TIMEOUT 600)
  ament_add_pytest_test(acm_generation test/test_acm_generation.py TIMEOUT 600)
  ament_add_pytest_test(gripper_sign test/test_gripper_sign.py TIMEOUT 600)
  ament_add_pytest_test(config_files test/test_config_files.py TIMEOUT 600)
endif()

ament_package()
