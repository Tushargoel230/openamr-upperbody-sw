cmake_minimum_required(VERSION 3.8)
project(openamrobot_upperbody_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes config test DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/datums_report.py
  scripts/print_config_id.py
  scripts/check_urdf_matrix.py
  scripts/build_evidence.py
  scripts/evidence_common.py
  scripts/verify_a6.sh
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(t01_expansion test/test_t01_expansion.py TIMEOUT 600)
  ament_add_pytest_test(t03_tree test/test_t03_tree.py TIMEOUT 600)
  ament_add_pytest_test(t06_arg_validation test/test_t06_arg_validation.py TIMEOUT 600)
  ament_add_pytest_test(t12_datums_schema test/test_t12_datums_schema.py TIMEOUT 600)
  ament_add_pytest_test(t13_datum_references test/test_t13_datum_references.py TIMEOUT 600)
  ament_add_pytest_test(t14_ros2_control_mock test/test_t14_ros2_control_mock.py TIMEOUT 600)
  ament_add_pytest_test(t15_ros2_control_gazebo test/test_t15_ros2_control_gazebo.py TIMEOUT 600)
  ament_add_pytest_test(t16_ros2_control_real test/test_t16_ros2_control_real.py TIMEOUT 600)
  ament_add_pytest_test(t04_frame_contract test/test_t04_frame_contract.py TIMEOUT 600)
  ament_add_pytest_test(t11_planar test/test_t11_planar.py TIMEOUT 600)
  ament_add_pytest_test(s4_sensors test/test_s4_sensors.py TIMEOUT 600)
  ament_add_pytest_test(t05_shoulder_height test/test_t05_shoulder_height.py TIMEOUT 600)
  ament_add_pytest_test(urdf_utils test/test_urdf_utils.py TIMEOUT 600)
  ament_add_pytest_test(datums_revision test/test_datums_revision.py TIMEOUT 600)
  ament_add_pytest_test(datums_report test/test_datums_report.py TIMEOUT 600)
  ament_add_pytest_test(w01_kdl_root_inertia test/test_w01_kdl_root_inertia.py TIMEOUT 600)
  ament_add_pytest_test(t02_check_urdf test/test_t02_check_urdf.py TIMEOUT 600)
  ament_add_pytest_test(t07_collision test/test_t07_collision.py TIMEOUT 600)
  ament_add_pytest_test(t09_inertia test/test_t09_inertia.py TIMEOUT 600)
  ament_add_pytest_test(t10_mass_report test/test_t10_mass_report.py TIMEOUT 600)
  ament_add_pytest_test(t13_visualization_constants test/test_t13_visualization_constants.py TIMEOUT 600)
  ament_add_pytest_test(e01_envelope test/test_e01_envelope.py TIMEOUT 600)
  ament_add_pytest_test(c01_config_id test/test_c01_config_id.py TIMEOUT 600)
  ament_add_pytest_test(c02_upstream_pin test/test_c02_upstream_pin.py TIMEOUT 600)
  ament_add_pytest_test(w02_planar_kdl test/test_w02_planar_kdl.py TIMEOUT 600)
  ament_add_pytest_test(evidence_dir test/test_evidence_dir.py TIMEOUT 600)
  ament_add_pytest_test(build_evidence test/test_build_evidence.py TIMEOUT 600)
endif()

ament_package()
