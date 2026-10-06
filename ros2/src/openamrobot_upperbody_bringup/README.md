# openamrobot_upperbody_bringup

Launch files for the upper body. Today there are two, and neither starts the mobile base.

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
localhost only. Not covered: a Gazebo or real launch (the `gazebo` and `real` profiles are checked by expansion in the
description package), and tracking performance, which mock hardware cannot show.

## Not here

- A Gazebo launch, a MoveIt launch and a base launch: later steps. The `package.xml` still lists the Gazebo and MoveIt
  packages, which are scaffolds.
- No lift: OpenAMRobot 2.0 has a fixed mast.

See [`openamrobot_upperbody_description`](../openamrobot_upperbody_description/README.md).
