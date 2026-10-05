cmake_minimum_required(VERSION 3.8)
project(openamrobot_upperbody_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes config launch test DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(t01_expansion test/test_t01_expansion.py TIMEOUT 600)
  ament_add_pytest_test(t03_tree test/test_t03_tree.py TIMEOUT 600)
  ament_add_pytest_test(t06_arg_validation test/test_t06_arg_validation.py TIMEOUT 600)
endif()

ament_package()
