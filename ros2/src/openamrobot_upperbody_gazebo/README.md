# openamrobot_upperbody_gazebo

**Status: scaffold. Nothing is implemented here yet.**

The package contains a manifest (depending on `ros_gz_sim` and the description) and empty `launch/`, `models/` and `worlds/`
directories. No world, no model and no launch file exists, and the combined description has never been loaded into Gazebo.

## Intended scope

Spawn the combined robot in Gazebo with the arms on `gz_ros2_control` and the base on its existing drive plugin. This is
the second pull request of the description work (A7), not the first.

## Known constraint

`robot_state_publisher` converts the description's `base_planar_joint` to a fixed joint (KDL has no planar joint), so planar
base motion in simulation must come from Gazebo's `odom -> base_link` transform, never from `robot_state_publisher`
(description check W02).

See [`openamrobot_upperbody_description`](../openamrobot_upperbody_description/README.md).
