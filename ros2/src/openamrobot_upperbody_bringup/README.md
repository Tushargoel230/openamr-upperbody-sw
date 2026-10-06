# openamrobot_upperbody_bringup

Launch files for the upper body: the model viewer, the mock bringup, and the MoveIt launch (A7). None of them starts the mobile base on its
own (`sim:=gazebo` below starts it through the simulator).

## View the model

```bash
ros2 launch openamrobot_upperbody_bringup view.launch.py mast_id:=mast_1350
```

`robot_state_publisher`, `joint_state_publisher_gui` and RViz2 on the combined description (it moved here from
`openamrobot_upperbody_description`). Source the workspace overlay first; the meshes are `package://` URIs. Args:
`mast_id`, `arm_package`, `enable_planar_base`.

## Mock bringup

```bash
ros2 launch openamrobot_upperbody_bringup mock.launch.py [mast_id:=mast_1350] [rviz:=true]
```

`robot_state_publisher` on the combined description with `hardware_profile:=mock`, `ros2_control_node` with
`openamrobot_upperbody_control/config/arm_controllers.yaml`, and spawners for `joint_state_broadcaster`,
`left_arm_controller`, `right_arm_controller`, `left_gripper_controller` and `right_gripper_controller`. `rviz` defaults to
`false`. The arms run on `mock_components/GenericSystem`, which reflects commands back as state.

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 action send_goal /left_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [openarm_left_joint1, openarm_left_joint2, openarm_left_joint3, openarm_left_joint4, openarm_left_joint5, openarm_left_joint6, openarm_left_joint7], points: [{positions: [0.2, 0, 0, 0.3, 0, 0, 0], time_from_start: {sec: 2}}]}}"
```

## Test

`test/test_l01_mock_controllers.py` (verification check **L01**, `launch_testing`) starts `mock.launch.py` headless, waits
up to 30 s for all five controllers to be `active`, sends one trajectory to `left_arm_controller` and checks that
`/joint_states` follows it within 0.02 rad while the right arm stays put. It runs in its own random `ROS_DOMAIN_ID`,
localhost only. Not covered: a real launch (the `real` profile is checked by expansion in the description package), and tracking
performance, which mock hardware cannot show. The Gazebo launch and its tests (G01, G02) are in `openamrobot_upperbody_gazebo`.

## MoveIt

```bash
ros2 launch openamrobot_upperbody_bringup moveit.launch.py [mast_id:=mast_1350] [sim:=mock|gazebo] [rviz:=false] \
                                                           [headless:=false] [physics_engine:=<gz-physics plugin>]
```

| Argument | Default | |
|---|---|---|
| `mast_id` | `mast_1350` | `mast_1300` \| `mast_1350` \| `mast_1400` \| `mast_1450` |
| `sim` | `mock` | `mock`: `mock.launch.py`, a stub for the wheel joints, then `move_group`. `gazebo`: the gazebo package's `sim.launch.py`, then `move_group` on the simulator's clock |
| `rviz` | `false` | RViz2 with the MotionPlanning display (`openamrobot_upperbody_moveit/config/moveit.rviz`). With `sim:=gazebo` it replaces the simulator's own RViz |
| `headless` | `false` | `sim:=gazebo` only: `true` runs `gz sim -s`, as the checks do |
| `physics_engine` | empty | `sim:=gazebo` only, passed to `sim.launch.py`. Empty means that launch's own default, **DART**, on which a `closed` gripper pose may not close. Pass `gz-physics-bullet-featherstone-plugin` for the grippers to close (M02 does) |

`base_joint_states_stub.py` publishes the base's two wheel joints at zero on `/joint_states`, for `mock` only: the base is not under
`ros2_control` (ADR-003), so nothing there reports them, and MoveIt would warn every second that the robot's state is incomplete. The base
does not move during manipulation (AD-30), so zero is the true state of the thing modelled. In Gazebo the real wheel states come from the
simulator.

## A7 tests

`test/test_m01_mock_named_poses.py` (**M01**, once per mast ID, 4 runs) and `test/test_m02_gazebo_named_poses.py` (**M02**, `mast_1350`)
send every SRDF named pose to `move_group` (both arms: `hands_up`, `home`, `hands_up`, `park`; both grippers: `closed`, `open`, each with its
own value), which plans and executes through the trajectory execution manager into the real controllers, and require a **final joint error
under 0.01 rad** after a one second settle. The record (a sidecar per run) holds every pose's outcome, planning and wall time and error, and
the number of `START_STATE_INVALID` retries (see the MoveIt package README: MoveIt refuses a start state 1e-7 rad past a limit, and a
simulated joint resting on its limit can read that way). M02 starts a headless Gazebo server in its own ROS domain and Gazebo partition and
passes `bullet-featherstone` explicitly. `simulation` class throughout.

They **skip unless `OPENAMR_A7=1`** (`run_a7_regression.sh` sets it): `verify_a6.sh` runs this package's whole `colcon test`, and it must neither
start Gazebo nor depend on an A7 check. A skipped M01 or M02 is never a result; the A7 report treats a missing sidecar as a `FAIL`.

```bash
ros2 run openamrobot_upperbody_bringup run_a7_regression.sh      # about 15 minutes; G01, G02, M01-M06, writes evidence/<ts>_a7.json
```

`run_a7_regression.sh` runs the gazebo, moveit and bringup tests, then `build_a7_evidence.py` writes `<workspace>/evidence/<UTC timestamp>_a7.json`
(`profile: "a7"`, `evidence_class: "simulation"`). A tree with uncommitted changes prints its results and writes **no** file (P08's rule: a
report records a commit). Exit status: **1** if any check is `FAIL`; **3** if nothing failed but the tree is dirty (nothing recorded); 2 if the
evidence cannot be assembled; 0 otherwise. `A7_M01_MASTS=mast_1350,mast_1400` reduces M01 to those mast IDs and the report says so.

## Not here

- The Gazebo launch alone is `ros2 launch openamrobot_upperbody_gazebo sim.launch.py` (the simulator and the base are started there).
- No lift: OpenAMRobot 2.0 has a fixed mast.

See [`openamrobot_upperbody_description`](../openamrobot_upperbody_description/README.md).
