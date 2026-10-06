# openamrobot_upperbody_moveit

MoveIt 2 configuration for the combined OpenAMRobot 2.0 model (A7): the two OpenArm v2.0 arms on the fixed mast, no lift group
(OpenAMRobot 2.0 has a fixed mast). Everything this package shows is class **`simulation`**: how the model plans and executes under
MoveIt 2.12.4 and **declared OpenAMRobot limits**, on `mock_components` and in Gazebo. It permits no statement about a physical
robot, a payload, a torque, or a validated limit (ADR-009).

```bash
source /opt/ros/jazzy/setup.bash && source ~/openamr_ws/install/setup.bash
ros2 launch openamrobot_upperbody_bringup moveit.launch.py sim:=mock rviz:=true                     # arms on mock_components
ros2 launch openamrobot_upperbody_bringup moveit.launch.py sim:=gazebo rviz:=true \
    physics_engine:=gz-physics-bullet-featherstone-plugin                                           # Gazebo, see "Which engine"
ros2 launch openamrobot_upperbody_moveit move_group.launch.py hardware_profile:=mock                # move_group alone (a stack is running)
```

In RViz's MotionPlanning panel pick a planning group (`left_arm`, `right_arm`, `left_gripper`, `right_gripper`), a named goal state
(`home`, `hands_up`, `park`; `open`, `closed`), Plan, then Execute. The fixed frame is `base_link`, which is the model frame MoveIt
reports for this SRDF. The configuration is built with `moveit_configs_utils`' `MoveItConfigsBuilder` over the combined xacro and
its args (`mast_id`, `hardware_profile`), in `openamrobot_upperbody_moveit/moveit_config.py`; nothing is read from upstream's MoveIt
package (ADR-007).

## The SRDF

`config/openamrobot.srdf` has three sections, labelled in the file.

1. **Imported from upstream**, with attribution: the groups `left_arm`, `right_arm`, `left_gripper`, `right_gripper`, the group
   states `home` and `hands_up`, the end effectors, the passive `*_finger_joint2` joints and the arm-internal `disable_collisions`,
   copied from `enactic/openarm_ros2` `openarm_bimanual_moveit_config/config/openarm_v2.0/openarm_bimanual.srdf` at **`4e837e1`**
   (Apache-2.0). One pair is dropped on purpose: `openarm_body_link0` / `openarm_right_link1`, because this model has no OpenArm body
   (ADR-008). `test/test_srdf_import.py` diffs these elements against the checkout at the pinned commit.
2. **Ours:** a `virtual_joint` that is **fixed**, `world -> base_link`; the group state `park`; the gripper states `open` and
   `closed`. The virtual joint is fixed because the base does not move during manipulation (AD-30), and because of W02: KDL has no
   planar joint, so planning against a planar root would be planning against a joint `robot_state_publisher` has already turned
   into a fixed one. Base motion in simulation comes from Gazebo's `odom -> base_link`.
3. **Generated:** the self-collision matrix for the combined model's own links, between the `BEGIN`/`END GENERATED ACM` markers.

### Regenerating the collision matrix (every setting)

```bash
ros2 run openamrobot_upperbody_moveit regenerate_acm.sh [--trials N] [--mast-id mast_1350]
```

It runs the MoveIt Setup Assistant's `collisions_updater` and `scripts/merge_acm.py` merges its output into the SRDF. Every setting
is also written to `config/acm/acm_generation.yaml`, which `test_acm_generation.py` reads back.

| Setting | Value | Note |
|---|---|---|
| Tool | `moveit_setup_assistant` `collisions_updater`, MoveIt 2.12.4 | the same `ComputeDefaultCollisions` code as the GUI's Self-Collisions page |
| Model | `combined_robot.xacro`, args `mast_id:=mast_1350 arm_package:=openarm hardware_profile:=mock` | the release configuration (rev 18.2) |
| Input SRDF | `config/openamrobot.srdf` as it is; its own `disable_collisions` are **not** kept (`--keep` off) | the tool recomputes every pair |
| Samples | `--trials 10000` (the script refuses fewer) | |
| "Never in collision" | on (always on in the tool) | pairs that collided in no sample |
| "Adjacent" | on (always on in the tool) | pairs adjacent in the kinematic tree |
| "Default" | on (`--default`) | pairs that collide in the default state: the two finger links of each hand |
| "Always in collision" | **off** (no `--always`) | the run found 0 such pairs; one would be a modelling question, not something to disable |
| Min. collision fraction | the tool's default, unused because "always" is off | |

Recorded run (2026-10-06): 46 links, 1035 possible pairs, 0 always, **458 never**, 2 default, 34 adjacent, 541 sometimes in
collision; 494 disabled in total; 20 of them were already imported from upstream, so 474 are in the generated block.

**The GUI was not driven.** The Setup Assistant GUI cannot be operated from a non-interactive session, so the headless tool of the
same package was used. The tool loaded the combined model without trouble (the stop condition "the Setup Assistant cannot load the
combined model" did not occur); opening `moveit_setup_assistant` on the same model and pressing "Generate Collision Matrix" at 10000
samples gives the same kind of matrix. The sampling is random, so the set of `Never` pairs differs by a few pairs from run to run (455,
456 and 458 in the three runs made here): that is the reason for M03 below.

## Named poses

| Group | State | Values | Status |
|---|---|---|---|
| `left_arm`, `right_arm` | `home` | all seven joints 0 | upstream's, imported |
| `left_arm`, `right_arm` | `hands_up` | joint 4 = 1.5 rad, the rest 0 | upstream's, imported |
| `left_arm`, `right_arm` | `park` | **a copy of `home`** | **placeholder**: stays a copy until Adnan's safe-pose definition lands under the manipulation action contract. It was deliberately given no safer-looking shape: a placeholder that looks deliberate gets trusted |
| `left_gripper` | `open` / `closed` | 0 / **+0.7854** | ours; `closed` is the far end of the left finger's own URDF range |
| `right_gripper` | `open` / `closed` | 0 / **-0.7854** | ours; `closed` is the far end of the right finger's own URDF range |

`home` has `openarm_*_joint4 = 0`, which is exactly that joint's lower limit. See "Known limitations".

### The two grippers take opposite-signed commands (ADR-009 point 5)

`GripperCommand` positions are `[0, 0.7854]` on the left hand and `[-0.7854, 0]` on the right. A "close the gripper" written as one
number is valid for one hand and out of range for the other, and nothing fails where it is written. So the values above are written
**per hand**, `named_poses.closed_value(side, limits)` derives each hand's value from its own URDF limit, and
`test/test_gripper_sign.py` (check **M05**, proposed) fails the single-number version in both directions, by hand, plus a state that
is in range but not the far limit, a missing state and an extra joint. The sign is also exercised live: M01 and M02 close and open
each hand with its own value through the real controllers.

## The collision matrix is validated by sampling (M03)

`scripts/validate_acm.py` draws 2000 random joint states inside the limits at each of the four mast IDs (numpy `default_rng(seed)`,
the same states at every mast ID) and checks collision **ignoring** the SRDF's list: the check uses an allowed-collision matrix of its
own that allows everything except the pairs under test (`moveit_py`; the colliding pairs are found by bisection, because the contact
map is not readable from Python). A pair disabled as `Never` that collides in any sample is a FAIL with the pair and a reproducing
sample; the `Never` pairs that never collided are listed, which is the expected outcome. Pairs disabled as `Adjacent` or `Default` touch
by design and are counted, not failed.

**Result at the recorded commit: M03 FAILS, and that is the finding.** At all four mast IDs the same pair collides in sample 1112 of
2000 (seed 20261006): **`openarm_left_ee_link1` <-> `openarm_left_link2`**, the left finger link against the upper-arm link. The pair is
arm-internal, so it cannot depend on the mast, and it does not. The other 457 `Never` pairs did not collide at any mast ID, including every
pair involving the base, the mast, the wheels, the casters, the lidar and the cameras.
The pair is **not** re-enabled here: the ACM is wrong for it, and which pair is wrong is the finding. A reproducing sample, rounded, is in the
evidence file; `python3 scripts/validate_acm.py --mast-id mast_1350 --samples 2000` reproduces it, and the sample can be applied to the model
directly.

How fragile the matrix is (a diagnostic, not part of the check): 20000 samples at `mast_1350` with another seed (7) found **14 of the 458 `Never`
pairs colliding**, each in 1 to 6 samples of 20000. All 14 are arm-against-arm pairs (left against right arm links, plus the finger-link2
pair above); none involves the base, mast, wheels, casters, lidar or cameras. So 10000 samples are too few to certify the rarest arm-arm pairs,
and the "never" classification is statistical: roughly one state in 5000 to 20000 reaches those pairs. The options for the architecture chat
are in the report: regenerate with far more samples, enable the arm-arm pairs that the larger run found, or keep M03's N and accept that a
rare pair is the expected finding.

M03's mutation test relabels a pair that really collides (`openarm_left_link1` / `openarm_left_link2`, adjacent) as `Never` and expects
exactly that pair to be reported, so the check is shown to be able to fail.

## M04: a goal inside a base-side volume is refused

`test/test_m04_collision_goal_rejected.py` plans (plan only) to a left-arm joint state in which a left gripper link collides with
`mast_base_link`, through `move_group`'s `/plan_kinematic_path`:

| Request | What happened |
|---|---|
| a free named pose (`hands_up`), the control | planned |
| the **joint goal** inside the mast volume | **rejected**: answered 0.019 s after the control; `move_group`'s log says OMPL found the contact, "Unable to sample any valid states for goal tree", "Invalid goal state", `Planner 'OMPL' failed with error code GOAL_STATE_INVALID` |
| the **pose goal** at the same place | **timed out**: it searched 4.99 s more than the control (the whole budget) looking for a valid goal sample, then failed with the same code |

"Rejected" and "timed out" are different claims and MoveIt 2.12.4 reports both as `FAILURE (99999)` with a planning time of 0 at the
action and at the service, so the check classifies on time spent beyond the control and asserts the planner's own log line for the joint
goal. Only the first is the robot refusing to hurt itself; the second would be a robot that tried until it gave up. Neither produced a plan.

**The arms cannot reach the platform's base body.** In 6000 random left-arm states at `mast_1350` the left gripper touched `base_link`, the
wheels, the casters, the lidar and the camera body in none of them, and M03's 8000 all-joint states found no collision of any arm link with
`base_link`; the left gripper touched `mast_base_link` in 144 of the 6000. So the refused goal is inside `mast_base_link`'s volume, the nearest
base-side volume a gripper can be put in. That the base body is out of reach is a reach fact for A4, not a defect here.

## The gripper fork (ADR-009 point 4, manipulation#11): what the installed plugins support

`moveit_controllers.yaml` is **option 1**, `position_controllers/GripperActionController` serving `GripperCommand`, and nothing else changed
(`arm_controllers.yaml` is untouched). The question the ADR left open was whether `moveit_simple_controller_manager`, as installed, can drive
a `ParallelGripperCommand` action at all. It was answered by reading the installed artefacts and then by running it, not from documentation:

* `/opt/ros/jazzy/include/moveit_simple_controller_manager/moveit_simple_controller_manager/` holds `gripper_command_controller_handle.hpp`
  **and `parallel_gripper_command_controller_handle.hpp`**. The second sends a `control_msgs/action/ParallelGripperCommand` goal for one
  `command_joint` (position from the last trajectory point; velocity and effort only when `max_velocity` / `max_effort` are set).
* `strings libmoveit_simple_controller_manager.so` (2.12.4) contains the type strings `ParallelGripperCommand`, `FollowJointTrajectory`
  and `GripperCommand`, the keys `command_joint`, `max_effort`, `max_velocity`, `joints`, `action_ns`, and the messages "Unknown controller
  type" and "ParallelGripperCommand controller only supports commanding a single joint": the manager dispatches the type.
* `ros2 action list -t` (live): `parallel_gripper_action_controller/GripperActionController` serves `control_msgs/action/ParallelGripperCommand`
  at `/<hand>_gripper_controller/gripper_cmd`, the same action name as option 1's `GripperCommand`.
* **Run on mock, in a scratch configuration outside the repository:** the two gripper controllers of `arm_controllers.yaml` switched to
  `parallel_gripper_action_controller/GripperActionController`, and MoveIt's entries to `type: ParallelGripperCommand` with
  `command_joint` set. Planning and executing `closed` and `open` through MoveIt drove both hands, each with its own sign, to within 0.001 rad.
  No handle plugin was needed.

| Option | Facts | Not established |
|---|---|---|
| 1. `position_controllers/GripperActionController` + `GripperCommand` | what this package uses; M01 and M02 drive both hands through it, on mock and in Gazebo (Bullet-Featherstone). Deprecated in Jazzy: a warning, not a removal | |
| 2. `parallel_gripper_action_controller/GripperActionController` + `ParallelGripperCommand` | **the installed MoveIt can drive it** through its own `ParallelGripperCommand` handle, no new plugin; works end to end on mock | Gazebo and hardware. The controller also has `max_effort_interface` / `max_velocity_interface` parameters (they name interfaces it would claim); whether `gz_ros2_control` and `OpenArmHW` provide them was not tested |
| 3. `JointTrajectoryController` on the finger joints + `FollowJointTrajectory` | the `FollowJointTrajectory` handle is what drives our arms, so it exists and works | not run for the fingers in this step |

So ADR-009 point 4's premise, "unknown whether `moveit_simple_controller_manager` can drive it; if not, it costs a controller-handle plugin",
resolves to: **it can, and it costs nothing**, at least on mock. The choice between the three stays with Om and Adnan.

## Limits are ours (ADR-009)

Upstream OpenArm v2.0 declares **no acceleration limits** (`openarm_bimanual_moveit_config/config/openarm_v2.0/joint_limits.yaml` @ `4e837e1`
sets `has_acceleration_limits: false` on every joint), and MoveIt 2.12's time parameterisation rejects a plan without them
(openamrobot-manipulation#10). `config/joint_limits.yaml` is this package's own file: every acceleration entry is tagged
`source: engineering_default`, `status: placeholder`, with the rule `max_acceleration = max_velocity / 2 s` (the URDF's own velocity
limit). It is **not vendor data**, and no payload, torque, inertia or controller analysis stands behind it. Velocity and position limits come from the
URDF (upstream's) and are not repeated.

`scripts/limits_sensitivity.py` (check **M06**, proposed) plans all twelve named poses under the configured limits scaled by 1, 0.1, 0.01
and 0.001 and under no acceleration limits at all. Result: **12 of 12 succeed under every scaling** (planned duration 16 s in total at x1,
517 s at x0.001: the effective limits go down to about 0.003 to 0.01 rad/s²) **and 0 of 12 succeed with none** (which reproduces #10). The plans
need *some* acceleration limit to exist and do not depend on how generous ours are, so the A7 stop condition ("planning succeeds only because
the limits we invented are generous") did not occur.

## Which physics engine M02 runs under, and why

`sim:=gazebo` includes the gazebo package's `sim.launch.py`, which **defaults to DART**: the engine platform-sw tuned the base on (decided
6 Oct). On DART the fingers do not follow the position command at the default gain and `finger_joint2` has no mimic constraint, so a
`closed` gripper pose may not close. **M02 passes `physics_engine:=gz-physics-bullet-featherstone-plugin` explicitly**, as G02 does, because
its poses include the grippers and the per-hand sign is part of what it checks; the arm poses do not depend on the engine. The base's contact
behaviour was compared on neither engine. `moveit.launch.py`'s `physics_engine` is empty by default, which means the simulator's default.

## Known limitations

* **W01:** `base_link` carries a 4.18 kg inertial from platform-sw and KDL drops the inertia of a root link, so **MoveIt never sees it**
  (the planning model logs "The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an
  inertia"). It matters for nothing planned here (the base does not move), and it is not fixed: the workaround changes the contract's root
  and the base is platform-sw's (ADR-005).
* **`park` is a placeholder** copy of `home`, until the safe-pose definition lands.
* **The acceleration limits are our engineering defaults**, not vendor data (above).
* **The gripper fork is open** on manipulation#11; this package is option 1 because it is the combination known to work, and the evidence above
  is for Om and Adnan.
* **The mimic needs `bullet-featherstone`**: `finger_joint2` follows `finger_joint1` only on that engine.
* **M03 FAILS** for `openarm_left_ee_link1` / `openarm_left_link2` (above): the matrix is not certified.
* **MoveIt's start-state check has no tolerance.** `home` sets `openarm_*_joint4 = 0`, exactly that joint's lower limit. In Gazebo the joint
  can read 1e-7 rad past the limit (-2.5e-7 and -9e-8 were seen), and MoveIt 2.12.4 answers `START_STATE_INVALID` to any request from such a
  state: the default `CheckStartStateBounds` adapter is strict, there is no `FixStartStateBounds` in the installed adapter set, and removing the
  adapter does not help because OMPL's planning context refuses the same state. Planning from `home` in Gazebo is therefore intermittently
  refused: planning again a moment later works. M01 and M02 retry at most three times per pose and **count every retry** in the evidence. Mock
  reads exact values and is not affected. Nothing was changed to hide this: widening MoveIt's bounds would be an invented limit, moving `home`
  off the limit would invent a pose.
* **`move_group` exits with SIGSEGV when it is stopped** (MoveIt 2.12.4, observed on every stop, with and without a test around it). The
  launch tests allow exit code -11 for that one process and no other.
* **The wheel joints are not under `ros2_control`**, so on `mock` nothing reports them and MoveIt would warn every second that the robot's
  state is incomplete; `moveit.launch.py` starts a stub that reports them at rest. In Gazebo they come from the simulator.
* **`move_group` does not publish `robot_description`** (`robot_state_publisher` does). Two publishers raced, and `ros_gz_sim` spawned the
  model from `move_group`'s copy, which has no `controllers_file`, so the simulator's controller manager never started ("found an empty
  parameters file").
* `moveit_py` segfaults when its objects are torn down; M03 and M06 run it in subprocesses that leave with `os._exit` after writing their result.

## Tests

`colcon test --packages-select openamrobot_upperbody_moveit`, or `ros2 run openamrobot_upperbody_bringup run_a7_regression.sh` for the
whole A7 report.

| Test | Check | Proves |
|---|---|---|
| `test_srdf.py` | AUX | names exist at every mast ID; groups, virtual joint (fixed, not planar), poses inside the limits; `park` is a copy of `home` and says it is a placeholder; each pair disabled once |
| `test_srdf_import.py` | AUX | the imported elements equal upstream's at the pinned commit (SKIP, with the reason, only if there is no checkout) |
| `test_acm_generation.py` | AUX | the recorded settings are the ones asked for (at least 10000 samples at `mast_1350`, never/adjacent/default on, always off) and the SRDF holds exactly what they say |
| `test_config_files.py` | AUX | limit tags and rule, controller names against `arm_controllers.yaml`, option 1 unchanged, OMPL defaults are MoveIt's |
| `test_gripper_sign.py` | M05 | per-hand gripper values; the single-number version fails |
| `test_m03_acm_sampling.py` | M03 | no `Never` pair collides (currently FAILS, see above); the mutation test; the bisection |
| `test_m04_collision_goal_rejected.py` | M04 | a goal in the mast volume is rejected; the pose goal times out |
| `test_m06_limits_sensitivity.py` | M06 | the plans need some acceleration limit and do not depend on its value |

M01 (mock, every mast ID) and M02 (Gazebo, `mast_1350`) are tests of `openamrobot_upperbody_bringup`: they start the stacks, and that package
depends on this one (a test dependency back would be a colcon cycle).

M05 and M06 are checks this step proposes; they are not in `design/VERIFICATION.md`.

See [`openamrobot_upperbody_description`](../openamrobot_upperbody_description/README.md) and
[`openamrobot_upperbody_bringup`](../openamrobot_upperbody_bringup/README.md).
