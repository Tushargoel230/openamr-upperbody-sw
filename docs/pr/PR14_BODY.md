<!-- Body for openAMRobot/openamr-upperbody-sw PR #14 (A6 + A7), from branch followups/a6-a7-49d2368. Replaces docs/pr/PR1_BODY.md.
     Tushar Goel's description at 49d2368 is kept, unchanged apart from heading levels, under "Description at 49d2368" below. -->

## Work package

**A6 + A7**: the canonical combined OpenAMRobot 2.0 description (A6: platform base, option B lift column and upper body, both OpenArm v2.0 arms) and its simulation and MoveIt integration (A7: `ros2_control`, Gazebo Harmonic, MoveIt 2, bringup). Issue #10 tracks it. Author: Tushar Goel. Follow-up commits (`49d2368..8e1aba1`) were prepared for Tushar to adopt, at Alex's decision of 16 Oct.

## Evidence

- Base SHA: `f57e54cfc9e7f9536e94c52749502cc983fb908e` (openAMRobot/openamr-upperbody-sw `main`)
- Head SHA: `8e1aba120f42d645734c57be702daa82d3da571b` (the tested code; the next commit on the branch adds only this file and removes `PR1_BODY.md`)
- Workspace: `openamrobot-manifest` `8111c48e48f93c20f2142534ffc340857234bf7a` (`vcs import`). In that workspace:
  - this repository at the head above;
  - `openamr-platform-sw` `15d0eb5254af4984cc66b2a236fec5a640a42466` (`main`);
  - `openamrobot-manipulation` `095e6c7bbd270db6d21de9664a654bb37919a2eb`;
  - `src/upstream/openarm_description` `14ff67b638ff1c738a1b9a6be8aaa5ce5ed2c831` and `src/upstream/openarm_ros2` `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` (the pins in `config/upstream_pins.yaml`).
- Machine: Ubuntu 24.04 container, ROS 2 Jazzy binaries, `/usr/bin/python3` = 3.12, 4 cores, no GPU (ogre2 renders in software; the simulator runs at about 0.4x real time). controller_manager 4.48.1, MoveIt 2.12.4.
- Run sequentially with nothing else running; every launch under a timeout, and no simulator, bridge, `move_group` or `robot_state_publisher` left running after any step.

```bash
source /opt/ros/jazzy/setup.bash
cd <workspace>
rm -rf build install log
colcon build --symlink-install --packages-skip openarm_hardware openarm
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
timeout 3600 colcon test --executor sequential --packages-select openamrobot_upperbody_description openamrobot_upperbody_control \
    openamrobot_upperbody_gazebo openamrobot_upperbody_moveit openamrobot_upperbody_bringup
colcon test-result --verbose --test-result-base build/<package>        # for each of the five packages
timeout 1800 bash src/openamr-upperbody-sw/ros2/src/openamrobot_upperbody_description/scripts/verify_a6.sh    # twice
timeout 3600 bash src/openamr-upperbody-sw/ros2/src/openamrobot_upperbody_bringup/scripts/run_a7_regression.sh  # twice
```

The evidence reports are `20261016T140434Z_dev.json` and `20261016T140930Z_dev.json` (A6), and `20261016T142206Z_a7.json` and `20261016T143422Z_a7.json` (A7). They were written by the harness from the clean tree at `8e1aba1`. They are not committed in this change.

## Tests

On `8e1aba1`:

| Run | Result |
|---|---|
| `colcon test`, five packages | **825 tests: 806 passed, 0 failed, 19 skipped** |
| `verify_a6.sh`, run 1 | **24 PASS, 4 WARN (T08, E01, W02, V01), 0 FAIL, 0 NOT_RUN** (exit 0) |
| `verify_a6.sh`, run 2 | **24 PASS, 4 WARN (T08, E01, W02, V01), 0 FAIL, 0 NOT_RUN** (exit 0) |
| `run_a7_regression.sh`, run 1 | **13 PASS, 2 WARN (M07, V01), 0 FAIL, 0 NOT_RUN** (exit 0) |
| `run_a7_regression.sh`, run 2 | **13 PASS, 2 WARN (M07, V01), 0 FAIL, 0 NOT_RUN** (exit 0) |

`colcon test` per package (passed / failed / skipped):

| Package | Tests | Passed | Failed | Skipped |
|---|---|---|---|---|
| `openamrobot_upperbody_description` | 635 | 635 | 0 | 0 |
| `openamrobot_upperbody_control` | 8 | 8 | 0 | 0 |
| `openamrobot_upperbody_gazebo` | 28 | 28 | 0 | 0 |
| `openamrobot_upperbody_moveit` | 86 | 83 | 0 | 3 |
| `openamrobot_upperbody_bringup` | 68 | 52 | 0 | 16 |

- **The 19 skips** are the A7-gated cases (which run in A7 and pass there) and the clamp-loop cases left to their other registration. They are listed in #16, and each reason names it. A skip is not a pass.
- **A7, every check:** G01, G02, M01, M02, M03, M04, M05, M06, L01, H01, H02, I01 and AUX PASS; M07 and V01 are WARN.
  - M07 WARNs on the four joints whose rest pose is an upstream limit: `openarm_{left,right}_finger_joint1` and `openarm_{left,right}_joint4`.
  - All 16 expected joints were covered, with at least 600 samples each.
  - V01 is WARN until a person answers `docs/v01_visual_check.md`.
- **`config_id`:** 14 of the 15 recorded ids differ from `49d2368`, because of the column geometry in `b45d8f6`. They are the same in both A6 runs.

**Failing runs before the change, one per behaviour change:**

| Change | What failed before |
|---|---|
| `b45d8f6` (a) | The column boxes with the old contract: A6 FAIL 22 / 4 / 2 (T04: "contract says collision_free"). The old collision matrix with the boxes: A7 M03 FAIL (95 derived pairs vs 83 in the contract). |
| `b45d8f6` (b) | On `49d2368`, M07 was FAIL on the same four joints. In this run it is WARN. |
| `1637743` | `test_sim_launch_spawners.py` fails 3 of 4 on the old `sim.launch.py`. On this machine the old launch left `joint_state_broadcaster` inactive (G01 NOT_RUN, G02 and M02 FAIL in two A7 runs on `49d2368`). |
| `84d80a9` | `test_a7_controller_deadline.py` fails 3 of 4 on the old helper (all-active just after the deadline returned ok). `test_m07_coverage.py` fails 5 of 5: its helpers do not exist in the old code. |
| `20886a6` | T04's pinned list fails until the two frames are listed. |

## Not verified

- No hardware.
- No supplier geometry for the column: the boxes are placeholders, so collision checks against the column body are only as good as the box.
- No F2S check, no reach acceptance, and no physical lift safety (power loss, E-stop holding).
- `tl3_400` is not verified: it is refused until its supplier data exist.
- No human visual review in RViz or Gazebo: V01 is WARN; L1000/L1175/L1350 × bp000/bp050 are still to be looked at.
- Not run on a hosted CI runner or on a machine with a GPU. All runs are from one VM.
- The gz-sim teardown hang (the server ignores the stop request) is not fixed. It did not occur in these runs; the detached stop helper still forces such a server, and the run then FAILs.
- The platform-sw drafts #68 (`imu_link` at the CAD Rev B position) and #69 (Gazebo IMU sensor) are not consumed. These results are on platform-sw `15d0eb5`.
- `license/cla` on PR #14 is still pending.
- The six conditional skips that did not trigger here (C02, C01, datums revision, W02, SRDF import) are listed in #16 but not changed.
- V01's recorded measurement text in `build_evidence.py` still says "0 of 478 enabled pairs" (P16). The matrix now enables 535 of 630; V01 has not been re-measured.
- The message of `ed8c56d` says the header comments change `config_id`. That is not so: the canonical URDF is `ET.canonicalize`d, which drops comments. The changed ids come from `b45d8f6`.

## Dependencies

Tushar's statement about platform-sw is kept under "Description at 49d2368" below. Every `package.xml` change against `main` (`f57e54c`) is listed here: **83 dependencies added** across the five packages, and none removed. The duplicate `<test_depend>moveit_msgs</test_depend>` in `openamrobot_upperbody_moveit` is declared once. Licences are as declared by the installed package (ROS) or by the Ubuntu copyright file (apt).

- OpenAMRobot packages are MIT.
- `openarm_description` is built from source at the pin, Apache-2.0.
- No pip, no new apt source, and no vendor SDK.

| Package | Tag | Dependency | Licence | Source |
|---|---|---|---|---|
| `openamrobot_upperbody_bringup` | exec_depend | `launch` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch) |
| `openamrobot_upperbody_bringup` | exec_depend | `launch_ros` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-ros) |
| `openamrobot_upperbody_bringup` | exec_depend | `xacro` | BSD | ROS 2 Jazzy binary (ros-jazzy-xacro) |
| `openamrobot_upperbody_bringup` | exec_depend | `robot_state_publisher` | BSD | ROS 2 Jazzy binary (ros-jazzy-robot-state-publisher) |
| `openamrobot_upperbody_bringup` | exec_depend | `controller_manager` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-controller-manager) |
| `openamrobot_upperbody_bringup` | exec_depend | `joint_state_publisher_gui` | BSD | ROS 2 Jazzy binary (ros-jazzy-joint-state-publisher-gui) |
| `openamrobot_upperbody_bringup` | exec_depend | `rclpy` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-rclpy) |
| `openamrobot_upperbody_bringup` | exec_depend | `sensor_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-sensor-msgs) |
| `openamrobot_upperbody_bringup` | exec_depend | `rviz2` | BSD | ROS 2 Jazzy binary (ros-jazzy-rviz2) |
| `openamrobot_upperbody_bringup` | test_depend | `launch_testing` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-testing) |
| `openamrobot_upperbody_bringup` | test_depend | `launch_testing_ament_cmake` | Apache-2.0, BSD | ROS 2 Jazzy binary (ros-jazzy-launch-testing-ament-cmake) |
| `openamrobot_upperbody_bringup` | test_depend | `launch_testing_ros` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-testing-ros) |
| `openamrobot_upperbody_bringup` | test_depend | `rclpy` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-rclpy) |
| `openamrobot_upperbody_bringup` | test_depend | `control_msgs` | BSD-3-Clause | ROS 2 Jazzy binary (ros-jazzy-control-msgs) |
| `openamrobot_upperbody_bringup` | test_depend | `controller_manager_msgs` | BSD | ROS 2 Jazzy binary (ros-jazzy-controller-manager-msgs) |
| `openamrobot_upperbody_bringup` | test_depend | `sensor_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-sensor-msgs) |
| `openamrobot_upperbody_bringup` | test_depend | `trajectory_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-trajectory-msgs) |
| `openamrobot_upperbody_bringup` | test_depend | `builtin_interfaces` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-builtin-interfaces) |
| `openamrobot_upperbody_control` | exec_depend | `joint_state_broadcaster` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-joint-state-broadcaster) |
| `openamrobot_upperbody_control` | exec_depend | `joint_trajectory_controller` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-joint-trajectory-controller) |
| `openamrobot_upperbody_control` | exec_depend | `gripper_controllers` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-gripper-controllers) |
| `openamrobot_upperbody_control` | test_depend | `ament_cmake_pytest` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-ament-cmake-pytest) |
| `openamrobot_upperbody_control` | test_depend | `openamrobot_upperbody_description` | MIT | OpenAMRobot workspace package |
| `openamrobot_upperbody_control` | test_depend | `python3-yaml` | MIT (PyYAML licence text) | Ubuntu 24.04 apt (python3-yaml) |
| `openamrobot_upperbody_description` | exec_depend | `openamrobot_description` | MIT | OpenAMRobot workspace package |
| `openamrobot_upperbody_description` | exec_depend | `openarm_description` | Apache-2.0 | enactic/openarm_description @ 14ff67b (source, pinned) |
| `openamrobot_upperbody_description` | test_depend | `gz_ros2_control` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-gz-ros2-control) |
| `openamrobot_upperbody_description` | test_depend | `python3-pytest` | MIT (Expat) | Ubuntu 24.04 apt (python3-pytest) |
| `openamrobot_upperbody_description` | test_depend | `python3-yaml` | MIT (PyYAML licence text) | Ubuntu 24.04 apt (python3-yaml) |
| `openamrobot_upperbody_description` | test_depend | `python3-jsonschema` | MIT | Ubuntu 24.04 apt (python3-jsonschema) |
| `openamrobot_upperbody_description` | test_depend | `python3-numpy` | BSD-3-Clause | Ubuntu 24.04 apt (python3-numpy) |
| `openamrobot_upperbody_description` | test_depend | `liburdfdom-tools` | BSD-3-Clause | Ubuntu 24.04 apt (liburdfdom-tools) |
| `openamrobot_upperbody_gazebo` | exec_depend | `ros_gz_bridge` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-ros-gz-bridge) |
| `openamrobot_upperbody_gazebo` | exec_depend | `gz_ros2_control` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-gz-ros2-control) |
| `openamrobot_upperbody_gazebo` | exec_depend | `controller_manager` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-controller-manager) |
| `openamrobot_upperbody_gazebo` | exec_depend | `robot_state_publisher` | BSD | ROS 2 Jazzy binary (ros-jazzy-robot-state-publisher) |
| `openamrobot_upperbody_gazebo` | exec_depend | `joint_state_publisher` | BSD | ROS 2 Jazzy binary (ros-jazzy-joint-state-publisher) |
| `openamrobot_upperbody_gazebo` | exec_depend | `xacro` | BSD | ROS 2 Jazzy binary (ros-jazzy-xacro) |
| `openamrobot_upperbody_gazebo` | exec_depend | `launch` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch) |
| `openamrobot_upperbody_gazebo` | exec_depend | `launch_ros` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-ros) |
| `openamrobot_upperbody_gazebo` | exec_depend | `rviz2` | BSD | ROS 2 Jazzy binary (ros-jazzy-rviz2) |
| `openamrobot_upperbody_gazebo` | exec_depend | `openamrobot_upperbody_control` | MIT | OpenAMRobot workspace package |
| `openamrobot_upperbody_gazebo` | exec_depend | `openamrobot_gazebo` | MIT | OpenAMRobot workspace package |
| `openamrobot_upperbody_gazebo` | test_depend | `launch_testing` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-testing) |
| `openamrobot_upperbody_gazebo` | test_depend | `ament_cmake_pytest` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-ament-cmake-pytest) |
| `openamrobot_upperbody_gazebo` | test_depend | `launch_testing_ament_cmake` | Apache-2.0, BSD | ROS 2 Jazzy binary (ros-jazzy-launch-testing-ament-cmake) |
| `openamrobot_upperbody_gazebo` | test_depend | `launch_testing_ros` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-testing-ros) |
| `openamrobot_upperbody_gazebo` | test_depend | `rclpy` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-rclpy) |
| `openamrobot_upperbody_gazebo` | test_depend | `control_msgs` | BSD-3-Clause | ROS 2 Jazzy binary (ros-jazzy-control-msgs) |
| `openamrobot_upperbody_gazebo` | test_depend | `controller_manager_msgs` | BSD | ROS 2 Jazzy binary (ros-jazzy-controller-manager-msgs) |
| `openamrobot_upperbody_gazebo` | test_depend | `sensor_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-sensor-msgs) |
| `openamrobot_upperbody_gazebo` | test_depend | `geometry_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-geometry-msgs) |
| `openamrobot_upperbody_gazebo` | test_depend | `nav_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-nav-msgs) |
| `openamrobot_upperbody_gazebo` | test_depend | `tf2_msgs` | BSD | ROS 2 Jazzy binary (ros-jazzy-tf2-msgs) |
| `openamrobot_upperbody_gazebo` | test_depend | `trajectory_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-trajectory-msgs) |
| `openamrobot_upperbody_gazebo` | test_depend | `builtin_interfaces` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-builtin-interfaces) |
| `openamrobot_upperbody_moveit` | buildtool_depend | `ament_cmake_python` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-ament-cmake-python) |
| `openamrobot_upperbody_moveit` | exec_depend | `moveit_configs_utils` | BSD-3-Clause | ROS 2 Jazzy binary (ros-jazzy-moveit-configs-utils) |
| `openamrobot_upperbody_moveit` | exec_depend | `moveit_kinematics` | BSD-3-Clause | ROS 2 Jazzy binary (ros-jazzy-moveit-kinematics) |
| `openamrobot_upperbody_moveit` | exec_depend | `moveit_planners_ompl` | BSD-3-Clause | ROS 2 Jazzy binary (ros-jazzy-moveit-planners-ompl) |
| `openamrobot_upperbody_moveit` | exec_depend | `moveit_simple_controller_manager` | BSD-3-Clause | ROS 2 Jazzy binary (ros-jazzy-moveit-simple-controller-manager) |
| `openamrobot_upperbody_moveit` | exec_depend | `moveit_ros_visualization` | BSD-3-Clause | ROS 2 Jazzy binary (ros-jazzy-moveit-ros-visualization) |
| `openamrobot_upperbody_moveit` | exec_depend | `moveit_py` | BSD | ROS 2 Jazzy binary (ros-jazzy-moveit-py) |
| `openamrobot_upperbody_moveit` | exec_depend | `launch` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch) |
| `openamrobot_upperbody_moveit` | exec_depend | `launch_ros` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-ros) |
| `openamrobot_upperbody_moveit` | exec_depend | `rviz2` | BSD | ROS 2 Jazzy binary (ros-jazzy-rviz2) |
| `openamrobot_upperbody_moveit` | exec_depend | `python3-yaml` | MIT (PyYAML licence text) | Ubuntu 24.04 apt (python3-yaml) |
| `openamrobot_upperbody_moveit` | exec_depend | `rclpy` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-rclpy) |
| `openamrobot_upperbody_moveit` | exec_depend | `sensor_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-sensor-msgs) |
| `openamrobot_upperbody_moveit` | exec_depend | `std_srvs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-std-srvs) |
| `openamrobot_upperbody_moveit` | exec_depend | `openamrobot_upperbody_control` | MIT | OpenAMRobot workspace package |
| `openamrobot_upperbody_moveit` | test_depend | `ament_cmake_pytest` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-ament-cmake-pytest) |
| `openamrobot_upperbody_moveit` | test_depend | `python3-pytest` | MIT (Expat) | Ubuntu 24.04 apt (python3-pytest) |
| `openamrobot_upperbody_moveit` | test_depend | `launch_testing` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-testing) |
| `openamrobot_upperbody_moveit` | test_depend | `launch_testing_ament_cmake` | Apache-2.0, BSD | ROS 2 Jazzy binary (ros-jazzy-launch-testing-ament-cmake) |
| `openamrobot_upperbody_moveit` | test_depend | `launch_testing_ros` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-testing-ros) |
| `openamrobot_upperbody_moveit` | test_depend | `rclpy` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-rclpy) |
| `openamrobot_upperbody_moveit` | test_depend | `moveit_msgs` | BSD | ROS 2 Jazzy binary (ros-jazzy-moveit-msgs) |
| `openamrobot_upperbody_moveit` | test_depend | `python3-numpy` | BSD-3-Clause | Ubuntu 24.04 apt (python3-numpy) |
| `openamrobot_upperbody_moveit` | test_depend | `joint_state_publisher` | BSD | ROS 2 Jazzy binary (ros-jazzy-joint-state-publisher) |
| `openamrobot_upperbody_moveit` | test_depend | `shape_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-shape-msgs) |
| `openamrobot_upperbody_moveit` | test_depend | `geometry_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-geometry-msgs) |
| `openamrobot_upperbody_moveit` | test_depend | `std_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-std-msgs) |

## Integration Gate

- **Overlapping work:**
  - This change sits on PR #14 (Tushar's `feat/a7-gazebo-moveit` @ `49d2368`, draft).
  - platform-sw #68 (draft, `imu_link` pose of 16 Oct) and #69 (draft, Gazebo IMU sensor). Both change the base this model includes: #69 changes 14 of 15 `config_id`s, and #68 moves `imu_link`. This contract lists `imu_link` by name only, so both are compatible.
  - platform-sw #53 (draft, Gemini RGB-D), and the planned base-description PR (platform-sw #49).
- **Reused:**
  - The upstream OpenArm v2.0 description and `openarm_ros2` MoveIt config at the pins, with Apache-2.0 headers kept on the derived files.
  - platform-sw's description and bridge, by reference.
  - The repository's own harness: `verify_a6.sh`, `run_a7_regression.sh`, `regenerate_acm.sh`.
- **Rejected:**
  - Widening the joint-state clamp epsilon.
  - A sampled collision matrix: the matrix stays derived, and `--write` was used only after Alex accepted it.
  - Moving the arm and gripper spawners onto `--activate-as-group`: it changes the controller order; not needed.
  - Skipping or removing any test.
- **Review conflict, for the reviewers to settle.** Mohamed's review asks to resolve M07 "without … turning an unaddressed failure into a pass/warning". Alex accepted follow-up (b), which reports the four upstream-limit joints as WARN. This branch follows Alex's decision.
  - Any other joint within the margin, a listed joint further past its limit than the margin, and incomplete coverage all remain FAIL.
  - Mohamed's coverage finding is fixed in `84d80a9`.
- **STATE.md:** the repository has no STATE.md, so there is none to update.

## Safety

No safety path is touched. No file matches the `safety_paths` globs of `openAMRobot/.github` `maintainers.yaml` (E-stop, brake, contactor, watchdog, motor enable, charge inhibit, safety).

- The lift stays outside control authority: `lift_joint` keeps effort 0 and velocity 0, and no controller drives it.
- The spawner change affects simulation start-up only.
- Simulation and mock results are not safety evidence.

## AI disclosure

- **Tushar's commits** (`f57e54c..49d2368`): developed with Claude Code (Anthropic), as Tushar states below ("AI assistance"). Each commit carries `Assisted-by: Claude Code`.
- **Follow-up commits** (`49d2368..8e1aba1`) and this file: produced by Claude Code at Alex Reznichenko's request, and signed off by him (DCO). That covers the verification runs, the code, test and configuration changes, the regenerated collision matrix, the `decision-allow` markers, the SPDX headers, issue #16 and this description.
- No upstream file is modified.

## Follow-up commits on this branch

| Commit | What it does |
|---|---|
| `b45d8f6` | (a) 240 x 240 mm placeholder boxes, visual and collision, on both lift-column stages (opt-in `column_geometry`), the frame contract line, and the re-derived collision matrix: 95 disabled pairs, was 83; 535 of 630 enabled. (b) M07 WARN for the four upstream-limit joints. |
| `1637743` | `joint_state_broadcaster` spawner: `--switch-timeout 30`, `--service-call-timeout 45`. The arm and gripper spawners start only after it exits 0. |
| `84d80a9` | The controller wait honours one absolute deadline. M07 requires coverage of every expected joint (Mohamed's two findings). |
| `20886a6` | Frame contract: `imu_link` and `base_camera_link` consumed from platform-sw, names only. |
| `405d162` | `decision-allow` markers on the 48 historical or negative-test register lines; the stale `mast_id` launch argument in the clamp-loop test is removed. |
| `ed8c56d` | Duplicate `moveit_msgs` removed; the frame reference points at `frame_contract.yaml`; SPDX MIT headers on 116 source files without a header. |
| `8e1aba1` | Every skip names #16. |

## Description at 49d2368 (Tushar Goel, unchanged)

Where this text and the sections above disagree, the sections above describe the branch head. Specifically:
- M07 is now WARN, not FAIL;
- the column now has placeholder geometry;
- the collision matrix has 95 disabled pairs;
- A7 is 13 PASS / 2 WARN / 0 FAIL.

### Why A6 and A7 are one PR

The change from the fixed mast to the lift column straddles the description and the simulation/MoveIt packages: the description gives up the mast, the control, Gazebo and MoveIt packages follow the lift joint, and the numbers each of them records (collision matrix, planning poses, configuration IDs) change together. On current `main` the description is only canonical if it carries the lift, and the lift work lives in the same commits as the Gazebo and MoveIt work, so the two cannot be split by commit; a cherry-pick of the A6 part on its own does not build green against `main`. A file-level split (description files in one PR, everything else in a second) is possible on request, at the cost of a first PR whose tests need the second. <!-- decision-allow: LIFT historical: the change away from the superseded fixed mast -->

`feat/a6-canonical-description` @ `44d28ea` stays on the fork as the historical A6 delivery (the mast model); this PR supersedes it.

### What is in it

Five packages under `ros2/src/`, plus `docs/`, `evidence/` and the changelog:

- `openamrobot_upperbody_description` — the canonical combined description: `combined_robot.xacro` composes the platform base (unchanged, `base_footprint` root), option B's upper body on its lift column and the upstream OpenArm macros; datums with provenance, frame contract, per-expansion evidence checks (T01-T16, C01/C02, E01, W01/W02, V01, H01/H02) and the harness that runs and reads them.
- `openamrobot_upperbody_control` — `ros2_control` for the two OpenArm v2.0 arms, one file for the mock, gazebo and real profiles (joint state broadcaster, trajectory and gripper controllers); the lift is not in it, we have no authority to command it.
- `openamrobot_upperbody_gazebo` — Gazebo Harmonic simulation of the combined model: the base on its DiffDrive plugin, both arms on `gz_ros2_control`, `odom -> base_footprint`.
- `openamrobot_upperbody_moveit` — MoveIt 2 configuration: arm planning groups with the lift as a planned joint, a collision matrix derived (83 disabled pairs, 478 enabled) rather than sampled, named poses, regression checks M01-M07, `joint_state_clamp.py`.
- `openamrobot_upperbody_bringup` — launch files composing base, lift column and arms in simulation.

### Evidence recorded at 013b524 (Tushar)

Class: `static+mock` for the A6 suite, `simulation` for A7. **No physical evidence.** Every check reports one of five values:

| value | meaning |
|---|---|
| `PASS` | the check ran and its assertion held |
| `WARN` | the check ran and recorded a finding; it is not a pass and every WARN has a check ID |
| `FAIL` | an assertion was violated, or a process guard saw a process fail to start, die or exit badly |
| `SKIP` | a stated, reasoned non-comparison (the one legitimate case: no git checkout to compare an upstream pin with) |
| `NOT_RUN` | a registered check left no result; never a `PASS`, and it fails the exit status |

- **A7 regression** (`run_a7_regression.sh`), on the code of this PR (`013b524`; the pushed head adds report copies, measurement data and documents, and one unit-test file): **no run is green, by design: M07 FAILs** (below). Run 2 and run 3: 13 PASS / 1 WARN (V01) / 2 FAIL (M07, and AUX with it, because the bringup `colcon test` step exits 1) / 0 NOT_RUN. Run 1 additionally failed **G01**: its `gz sim` server ignored SIGINT, the stop request and SIGTERM and was ended by the stop helper's SIGKILL at 9.2 s; nothing was left running, and the run says so (11 PASS / 1 WARN / 3 FAIL). Earlier, on `7daa2ab` (the shutdown-race fix, before the rebase and before option B: these prove that fix) five runs of 13 PASS / 1 WARN / 0 FAIL, plus three on `dada509` and three on `aa94fc3`.
- **One A7 run in six failed on this code earlier** (P16, `9bc327c`: 11 PASS / 1 WARN / 2 FAIL, M02 `CONTROL_FAILED`) under a simulator orphaned by G01's launch that nothing detected. Its cause was identified, it is now detected (and, since P17, stopped by the launch), and the report is kept (`evidence/reports/README.md`, P16). The number to quote is "one failure in six on that commit, cause identified", not "three for three"; and P17's run 1 above is a second, different one.
- **M07 currently FAILs by design**: no joint's rest position may be within `joint_rest_margin_rad` (a datum, 1e-3 rad, `placeholder`) of either limit. It FAILs on `openarm_right_finger_joint1` (rests at +2.6e-8 rad, past its open limit) and on `openarm_left_finger_joint1`, `openarm_left_joint4` and `openarm_right_joint4`. The zero and open poses are limits by construction, in the old body too (`evidence/m02_bisect/RESULTS.md`, last section), so the check is red until the margin, the rest pose, or the claim it makes is decided; that is a question for the reviewers, not something the PR hides.
- **A6 suite** (`verify_a6.sh`): **24 PASS / 4 WARN (T08, E01, W02, V01) / 0 FAIL / 0 NOT_RUN**, three runs on `013b524` (and on `1168f45` before). The first pass of this step, on `a713533`, failed 3 of 3 (AUX: `test_evidence_dir.py`, the stop helper's report was in the system temp directory); the reports are kept and the fix is `013b524`. `config_id`s changed again (the datums revision, for the new `joint_rest_margin_rad`).
- The reports are in `evidence/reports/`, with `README.md` listing commit and result for each, including the failed ones. `evidence/frozen_a6_993b000/` is the frozen expansion set of the earlier mast-model A6, with a verifier.
- **Orphaned simulators.** Two of the eight A7 runs of P15 left a `gz sim` server running after the launch under test was torn down, and one more did after G01 in P16 (the natural rate was about 1 in 13 launches). G01, G02, M02 and M07 now fail when a simulator of their own partition survives their launch, and `sim.launch.py` stops the server through a detached helper (SIGINT, then the service request, then SIGTERM and SIGKILL); a server that has to be sent SIGTERM or SIGKILL is itself a FAIL of the run that needed it. After that fix: **0 orphans seen by the detector in 12 runs each of G01, G02 and M02** (natural rate before: about 1 in 13). That is not a claim that it is fixed: 12 clean runs of a one-in-13 event are not unlikely, and the one live force-stop above shows the hang still occurs (one in 49 launches since, 36 of them in that count). The cause of the hang is inside gz-sim and was not found. The detector was proven through the launch-test path with an injected fault (`canary_gz_orphan.sh`) and on decoy processes.

### Dependencies on platform-sw (Tushar)

- **platform-sw `935efc7` (#55)**, on platform-sw `main` (`174731c` at the time of writing): `base_footprint` root, `base_link` 0.098 m above it, and the near-field sensor frames `us_*` (4 x MB7060) and `tof_*` (2 x TF-Luna) declared. This description requires it (its root and the 0.098 m are asserted against platform-sw's `base_footprint_joint`).
- The six near-field frames that earlier versions of our contract declared (`tof_front_left_link` ... `ultrasonic_rear_link`) are removed from our contract and are `consumed_from` platform-sw, not declared by us. T04 now reports which of them each expansion contains and passes **by deletion**: nothing in our contract can be absent any more.
- No upstream (`openarm_description`) file is modified; the pin is unchanged.

### Open questions for reviewers

- [ ] **Alex and Cuma: the two opt-in parameters on `openamrobot_upperbody.xacro`.** `emit_arm_mounts` (default `true`) and `column_inertial` (default `''`, none). Both default to current `main` behaviour. They exist because the upstream arm macro emits a link and a joint with the same names as option B's mount frames, so the combined robot has to turn those off, and because the combined robot supplies the lift's sourced catalogue mass. Two separate statements, kept apart because the first one is weaker than it sounds. (a) A test (`test_with_the_opt_in_parameters_at_their_defaults_the_macro_expands_as_it_does_on_main`) asserts that the mount frames and joints exist and that no inertial appears anywhere with the defaults; it does **not** compare against `main`. (b) Separately, compared byte for byte against `upstream/main` (`tools/compare_option_b_wrapper.sh`, run at `HEAD`), the standalone wrapper with the defaults expands identically apart from the empty `lift_column_base_link`, which is serialised as `<link name="lift_column_base_link"></link>` (an open/close pair) instead of a self-closing tag, and the path xacro writes in its header comment. That script is not part of the test suite. Needs acceptance, or a different composition.
- [ ] **Supplier drawing for the lift column.** The column has **no visual and no collision geometry**: it exists in the model as an inertia tensor only, so every render committed under `evidence/renders/` shows the arms with no column between them, and the arms can plan through the column body. It also carries one invented lateral dimension: `lift_column_section` y is set equal to the sourced 240 mm x side. Both are marked in `datums.yaml` (`placeholder`/`provisional`); nothing downstream treats them as measured.
- [ ] **V01's interference count was taken on collision geometry only.** The arms' visual meshes were not intersected separately, and the column has neither kind of geometry, so a zero there says nothing about the column.

### Known gaps

- `lift_mass_tl3_400` is `null` (no supplier mass for the fallback TiMOTION TL3 400 column). By design a null datum that is read is a hard-fail that names itself: the `tl3_400` configuration does not expand.
- V01 (manual visual check) is `WARN`. Its eight items were measured by script on the lift model (none reads outside its expected value; two sub-questions that need eyes are CANNOT-MEASURE; `evidence/v01/`, renders in `evidence/renders/`), and it stays `WARN` until `lift_mass_tl3_400` is sourced, however the eight land. No person has looked at the running simulator.
- M02 and the clamp: **the activation count is retired as a criterion.** It scatters 646 to 1054 inside one unchanged configuration (and 1795 / 957 / 1832 on three runs of a tests-only diff), so a rise from about 200 to about 1800 could not be read as a signal. M02 now gates on two numbers: readings beyond the epsilon (must be 0) and the largest overshoot magnitude (must be below the epsilon, 1e-5 rad); the counts are reported as `diagnostic_only`. In the three runs above: 0 excursions and a largest overshoot of 1.5e-6 rad (margin about 6.5x; P16's worst variant run read 4.2e-6, about 2.4x). The epsilon has not been widened. M02 also waits for the five controllers to be `active` before its first pose and FAILs, naming their states, if they never are. The right finger rests on its open limit (see M07); why it sits within 1e-5 rad of it in the new body and 2e-4 to 5e-4 inside in the left one was measured (finger effort, both trees, `evidence/m02_bisect/RESULTS.md`) and **not explained: cause unknown**. The M02 bisect is closed.
- No runtime producer of the configuration identity has an owner: `config_id` is computed offline from files; nothing in a running system publishes it.
- Arms are 62 mm apart at their bases: no contact at home, but the left gripper reaches right-arm links in about 3% of random left-arm states (sampled).
- A `gz sim` server that is asked to stop sometimes never begins its teardown and survives SIGINT, the stop request and SIGTERM (about 1 launch in 13 before the helper; 1 in 49 after). The launch now ends it by SIGKILL and the run FAILs naming it; the underlying hang is in gz-sim and is not fixed here.

### `config_id` inputs changed

`config_id` kept its format and changed its inputs: the canonical URDF, `column`, `base_plate`, `head_pitch`, `base_tilt`, `arm_package`, `hardware_profile`, datums revision and every upstream pin (before: `mast_id` and no column or base-plate inputs). It still validates against the C9 schema, so nothing fails to say that **an id recorded before this change is not comparable with one computed now**. The frozen A6 set keeps the old inputs and says so. Satya (C9 owner): please confirm that consumers do not compare ids across this change. <!-- decision-allow: CONFIG-IDS historical: the config_id inputs before this change -->

### AI assistance (Tushar)

This PR was developed with Claude Code (Anthropic) against a written design (architecture documents and prompts). The work was run and tested by the repository's evidence harness (`verify_a6.sh`, `run_a7_regression.sh`; the reports are in `evidence/reports/`, produced on the author's workstation, not by a hosted CI service) and reviewed by Tushar Goel. Prompts and design documents are available on request.
Per the OpenAMRobot AI Contribution Policy.

### Checklist

- [ ] DCO sign-off on every commit
- [ ] Contributor agreement accepted
- [ ] No upstream files modified
- [ ] No frame names outside ros2/src/openamrobot_upperbody_description/config/frame_contract.yaml
