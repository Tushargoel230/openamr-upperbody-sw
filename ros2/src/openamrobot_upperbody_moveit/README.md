# openamrobot_upperbody_moveit

**Status: scaffold. Nothing is implemented here yet.**

The package contains a manifest (depending on `moveit_ros_move_group` and the description) and empty `config/` and `launch/`
directories. There is no SRDF, no planning group, no named pose, no collision matrix and no launch file.

## Intended scope

MoveIt 2 configuration for the combined fixed-mast model: arm planning groups, a collision matrix generated against the
combined model, named poses and controller mapping. No actuated lift planning group: OpenAMRobot 2.0 has a fixed mast. This is the second pull request of the description work (A7), not the first.

## Known constraint

MoveIt uses KDL, which drops the inertia of a root link. The base's `base_link` carries an inertial from platform-sw, so
MoveIt will not see it (description check W01). This is not fixed here.

See [`openamrobot_upperbody_description`](../openamrobot_upperbody_description/README.md).
