# openamrobot_upperbody_control

**Status: scaffold. Nothing is implemented here yet.**

The package contains a manifest and empty `config/` and `launch/` directories. There are no controller configurations,
no launch files and no tests.

## Intended scope

- `ros2_control` configuration for the two OpenArm arms: joint state broadcaster, a joint trajectory controller per arm and
  a gripper controller per arm, for the `mock`, `gazebo` and `real` hardware profiles.
- **The mobile base is not under `ros2_control`.** The base keeps its existing drive plugin and its `/cmd_vel` and `/odom`
  interface; this package must not touch them.
- **No lift controller.** OpenAMRobot 2.0 has a fixed mast; an actuated lift is OpenAMRobot 3.0 scope. The `description`
  field of `package.xml` still says "lift and arm" because it predates that decision (issue #7) and has not been
  re-worded yet.

## Where this stands

The arm `ros2_control` profiles are the next step after the description PR. Until they land, the description's
`hardware_profile` argument is validated but has no effect, and the verification checks that need controllers (T14, T15,
T16, L01) are reported as `SKIP`, not passed. The description evidence says nothing about controllers.

See [`openamrobot_upperbody_description`](../openamrobot_upperbody_description/README.md).
