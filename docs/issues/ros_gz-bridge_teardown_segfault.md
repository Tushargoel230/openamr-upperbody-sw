<!-- Draft of the issue for https://github.com/gazebosim/ros_gz/issues/new . NOT FILED: the workspace has no GitHub credential (no gh, no token),
     so it is written to be pasted by someone who has one. When it exists, put its URL in docs/pr/PR14_BODY.md ("Findings that are not resolved") and here.
     Companion draft: docs/issues/gz-sim_server_teardown_segfault_and_hang.md (gz-sim; not filed either). -->

# parameter_bridge dies of SIGSEGV (exit -11) when the launch stops while a headless gz-sim server is tearing down (ros_gz_bridge 1.0.24, Jazzy)

## Environment

- Ubuntu 24.04.5 (WSL2), ROS 2 Jazzy: `ros-jazzy-ros-gz-bridge 1.0.24`, `ros-jazzy-ros-gz-sim 1.0.24`, `ros-jazzy-gz-sim-vendor 0.0.13` (gz-sim 8.15.0),
  `ros-jazzy-gz-transport-vendor 0.0.9`, `ros-jazzy-gz-ros2-control 1.2.20`. Software rendering (no GPU), headless server (`gz sim -r -s -v 2`).
- Physics: gz-physics Bullet-Featherstone (both crashes below). The server has `gz_ros2_control` loaded and five controllers active.
- Two `ros_gz_bridge` `parameter_bridge` nodes in the launch: one from a platform package (`platform_bridge`, a YAML of clock, tf, lidar, imu and so on) and
  one added by our package, **`base_joint_states_bridge`**, with a single entry:

  ```yaml
  - ros_topic_name: "/base/joint_states"
    gz_topic_name: "/joint_states"
    ros_type_name: "sensor_msgs/msg/JointState"
    gz_type_name: "gz.msgs.Model"
    direction: GZ_TO_ROS
  ```

## What happens

`launch_testing` (or Ctrl-C) stops the launch. launch sends SIGINT to every process of the launch at the same moment, the bridges and the server's shell wrapper
(the `ros2 launch` log shows the `sending signal 'SIGINT'` lines for `parameter_bridge-4`, `parameter_bridge-3`, `robot_state_publisher-2` and `gazebo-1` within
1 ms of each other). Both bridges log `signal_handler(SIGINT/SIGTERM)`. `platform_bridge` finishes cleanly; `base_joint_states_bridge` then reports

```
[ERROR] [parameter_bridge-4]: process has died [pid 87475, exit code -11, cmd '/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge --ros-args -r __node:=base_joint_states_bridge --params-file /tmp/launch_params_asknaziv'].
```

with nothing printed by the bridge itself between its `signal_handler` line and the death (no trace, no message). In both runs it died about 0.1 s after the signal, at the moment
the gz server's `controller_manager.pal_statistics` teardown lines were being printed. The rest of the shutdown was normal: the server stopped in 1.4 s, nothing was left running.

## How often (our launch tests; each is one headless launch with this stack)

The `-11` column below is OBSERVED-NOT-RECORDED: it was read from the console at the time. The number of launches is four per recorded run.

| set of runs | launches with this bridge | `parameter_bridge` exit -11 | exit -6 (SIGABRT, "corrupted double-linked list", a different, older symptom) |
|---|---|---|---|
| 3 runs on an earlier commit of our branch (`bb39c83`), platform package at `174731c` | 12 | 0 | not counted |
| 3 runs, platform package at `d619012` | 12 | 0 | 0 |
| 3 runs, platform package at `dfd44a5` | 12 | **2** (runs 2 and 3, both in the Bullet-Featherstone gripper test) | 1 (the other bridge, run 2) |
| 3 more runs, platform package at `dfd44a5`, a few hours later, same code | 12 | 0 | 1 |
| 3 more runs, platform package at `dfd44a5`, a few hours later, with test-only changes | 12 | 0 | 1 |

That is **2 of 60 launches in total, 2 of 36 on the one platform package where it was seen**; the three sets of three on that package ran within a day of each other and gave 2, 0 and 0, so we cannot say the
rate depends on anything we changed or on the platform package. Both crashes were in the same launch test (the gripper test, Bullet-Featherstone), in the same bridge instance. We have not
run a larger sample and have no claim about the true rate. **The only observed rate is 2 of 60 launches since P18, OBSERVED-NOT-RECORDED: the exit codes were read from the console, and the reports record only that G02 and AUX failed.** (An earlier figure from this project, about 1 in 7 G02 runs, appeared in a report with no evidence behind it and was never recorded; we searched our earlier reports and logs and found no SIGSEGV of this process in them. Please do not rely on it.)
The exit -6 case, a heap-corruption abort of the same executable at the same moment, was seen before (the project's process guard allows it and says so), which suggests one
underlying teardown problem with two outcomes, not two problems. That is a guess.

## Not known

- No stack trace: no gdb or core dump on the machine that saw it, and the bridge prints nothing. We have not rebuilt `ros_gz_bridge` with symbols.
- Whether it needs the server to be tearing down at the same moment (the bridge is subscribed to a `gz.msgs.Model` topic that the model's JointStatePublisher plugin writes),
  or whether the bridge alone would do it. A bare `parameter_bridge` with this one entry against an empty-world server was **not** tried.
- Whether it is the same defect as the server-side teardown segfault in the companion gz-sim draft. Different process, same period of the shutdown.

## Reproduction (probabilistic; we have no deterministic case)

```bash
git clone https://github.com/Tushargoel230/openamr-upperbody-sw && cd openamr-upperbody-sw
git checkout feat/a7-gazebo-moveit       # the commit that carries this draft
# build the five packages under ros2/src with openamr-platform-sw at dfd44a5 in a Jazzy workspace (colcon build --symlink-install), then
colcon test --packages-select openamrobot_upperbody_gazebo --event-handlers console_direct+ --pytest-args -k g02    # the Bullet-Featherstone gripper launch test
```

Repeat until a launch ends with `Proc parameter_bridge-4 exited with code -11`. The reports that record the two failed runs (they record G02 and AUX failing, **not the exit code**, which was read from the console: OBSERVED-NOT-RECORDED) are `evidence/reports/20261018T200829Z_a7.json` and
`20261018T201608Z_a7.json`. Expect it in a minority of launches.

## Expected

`parameter_bridge` exits 0 (or by its own SIGINT handler) when it receives SIGINT, whatever the state of the simulator it is bridging from.

## Notes

- The workaround in our project is none: the process guard that sees the crash allows SIGABRT for `parameter_bridge` (found earlier, documented as a finding) and **does not allow
  SIGSEGV**, so the test fails when this happens. We would rather it fail than hide a crash.
- Happy to run anything on this stack.
