# openamrobot_upperbody_control

`ros2_control` configuration for the two OpenArm arms. One file, `config/arm_controllers.yaml`, serves every hardware
profile (`mock`, `gazebo`, `real`; ADR-003): the plugin differs, the controllers do not.

| Controller | Type | Joints |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | all |
| `left_arm_controller`, `right_arm_controller` | `joint_trajectory_controller/JointTrajectoryController` | `openarm_{left,right}_joint1..7`, position command, position and velocity state |
| `left_gripper_controller`, `right_gripper_controller` | `position_controllers/GripperActionController` | `openarm_{left,right}_finger_joint1` |

The controller manager runs at 100 Hz. The file is derived from `enactic/openarm_ros2` (`ros2_controllers.yaml` at
`4e837e1`, Apache-2.0); the header lists the three changes. The one that matters: the Jazzy `GripperActionController`
reads `joint:` (a string), and upstream's `joints:` list makes it fail to configure with "Joint name cannot be empty".
`position_controllers/GripperActionController` is deprecated in Jazzy and logs a warning at every configure. **It is kept
on purpose, because its replacement is not a rename:** the deprecated controller serves the `control_msgs/action/GripperCommand`
action (the one upstream's MoveIt configuration expects), while `parallel_gripper_action_controller/GripperActionController`
(the warning misspells it as `parallel_gripper_controllers/...`) serves `control_msgs/action/ParallelGripperCommand`, with a
different goal. Switching would change the gripper interface, which is manipulation#11 and ADR-009 point 4, open with Om and
Adnan. Checked in the Jazzy 4.42 binaries (`libgripper_action_controller.so` and `libparallel_gripper_action_controller.so`)
and live: `ros2 action list -t` shows `ParallelGripperCommand` on the parallel controller.

- **The mobile base is not under `ros2_control`.** The base keeps its existing drive plugin and its `/cmd_vel` and `/odom`
  interface; this package must not touch them.
- **No lift controller.** OpenAMRobot 2.0 has a fixed mast; an actuated lift is OpenAMRobot 3.0 scope.

## Tests

`test/test_arm_controllers_yaml.py` checks, for every profile, that each joint the YAML names is declared under
`<ros2_control>` with the interfaces its controller needs, that every declared joint has exactly one controller, and that
the `finger_joint2` mimic has none. The runtime counterpart is L01 in `openamrobot_upperbody_bringup`.

See [`openamrobot_upperbody_description`](../openamrobot_upperbody_description/README.md) for the `ros2_control` blocks.
