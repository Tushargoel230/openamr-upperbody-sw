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
3. **Generated:** the self-collision matrix for the combined model's own links, between the `BEGIN`/`END GENERATED ACM` markers. Together
   with the imported arm-internal pairs it is **the collision-matrix contract**: this checked-in file is what M03 validates, and its
   fingerprint is recorded in `config/acm/acm_generation.yaml` (a test fails if the matrix changes without the record changing).

### The collision matrix is a checked-in contract; regenerating it only proposes

```bash
ros2 run openamrobot_upperbody_moveit regenerate_acm.sh [--trials N] [--mast-id mast_1350]         # a PROPOSAL: prints the diff, changes nothing
ros2 run openamrobot_upperbody_moveit regenerate_acm.sh --write                                    # replaces the contract: a decision, read the diff
```

**`collisions_updater` takes no seed.** Checked on the installed tool: `--help` lists `--config-pkg --urdf --srdf --output --xacro-args
--default --always --keep --verbose --trials --min-collision-fraction` and no seed, `ComputeDefaultCollisions` takes the number of trials and
nothing else, and it samples with `RobotState::setToRandomPositions()`, whose generator is seeded from the system. Measured: two runs at the same
settings gave 473 and 468 `Never` pairs at 3000 trials, three runs at 10000 gave 455, 456 and 458, and a fresh 10000-trial run against the contract
below differs from it in 11 pairs (3 only in the run, 8 only in the contract; all of them left-arm against right-arm links). So **running the
procedure again is not a verification step**: it is a proposal for a new contract. The contract is the SRDF in the repository, and what verifies
it is `scripts/validate_acm.py` (M03, below), which is seeded.

The script runs the MoveIt Setup Assistant's `collisions_updater`, `scripts/merge_acm.py` merges its output into a **copy** of the SRDF, and the
diff against the contract is printed. Without `--write` the contract and its record are untouched; with `--write` the SRDF and
`config/acm/acm_generation.yaml` (every setting, and the new fingerprint) are rewritten, and the commit that does it is a contract change.

| Setting | Value | Note |
|---|---|---|
| Tool | `moveit_setup_assistant` `collisions_updater`, MoveIt 2.12.4 | the same `ComputeDefaultCollisions` code as the GUI's Self-Collisions page |
| Model | `combined_robot.xacro`, args `mast_id:=mast_1350 arm_package:=openarm hardware_profile:=mock` | the release configuration (rev 18.2) |
| Input SRDF | `config/openamrobot.srdf` as it is; its own `disable_collisions` are **not** kept (`--keep` off) | the tool recomputes every pair |
| Samples | `--trials 10000` (the script refuses fewer) | |
| Seed | **none: the tool has no such option** | see above |
| "Never in collision" | on (always on in the tool) | pairs that collided in no sample |
| "Adjacent" | on (always on in the tool) | pairs adjacent in the kinematic tree |
| "Default" | on (`--default`) | pairs that collide in the default state: the two finger links of each hand |
| "Always in collision" | **off** (no `--always`) | the run found 0 such pairs; one would be a modelling question, not something to disable |
| Min. collision fraction | the tool's default, unused because "always" is off | |

The contract (recorded 2026-10-06, `contract:` in `acm_generation.yaml`): 46 links, 1035 possible pairs, 494 disabled pairs (458 `Never`, 36
`Adjacent`), of which 20 are imported from upstream and 474 are in the generated block; fingerprint
`sha256:f8254fac4550ef1254034d477e406d69c725a8facd8d7c8402fcd4e371803e45`.

**The GUI was not driven.** The Setup Assistant GUI cannot be operated from a non-interactive session, so the headless tool of the same
package was used. The tool loaded the combined model without trouble (the stop condition "the Setup Assistant cannot load the combined model"
did not occur).

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
`test/test_gripper_sign.py` (check **M05**, adopted 7 Oct) fails the single-number version in both directions, by hand, plus a state that
is in range but not the far limit, a missing state and an extra joint. The sign is also exercised live: M01 and M02 close and open
each hand with its own value through the real controllers.

## The collision matrix, split by who owns the claim (M03)

"Never in collision" in a generated matrix is a **sampling conclusion**, not a certification: the tool's own count of `Never` pairs changes
from run to run (above), and 20000 samples at another seed (7) found 14 of the 458 `Never` pairs colliding (a P10 diagnostic), all of them
arm-against-arm. So M03 says what each part of the matrix can and cannot be held to, and `scripts/validate_acm.py` derives the parts.

**Inherited and ours, derived from link provenance** (`openamrobot_upperbody_moveit/acm_split.py`; nothing is a list somebody keeps up to date).
A link is *upstream's* if the frame contract's `owner` column says so (`upstream: ...`), else if it is named `openarm_*`; a link that is neither
listed nor prefixed is ours, so an unknown link cannot hide a defect behind someone else's name. A pair whose two links are both upstream's is
**inherited** (Enactic's arm: a collision there is a **WARN** carrying the pair, the seed and the sample index, an upstream observation and not a
defect of ours); every other pair is **ours** (base, wheels, casters, lidar, camera, mast, plates, whatever is added next, and ours against
upstream's): a colliding disabled pair there is a **FAIL**. The sets partition the matrix by construction, and `test_acm_split.py` proves it at
every mast ID, proves that a link added later lands on the right side without an edit, and fails on a contract that contradicts the prefix.

**Certified by construction.** Between two links of a kinematic tree there is one path. If no joint on it can move (all `fixed`, none a mimic),
their relative transform is a constant, so the pair collides in every configuration or in none, and **one evaluation settles it**. Of our 344
`Never` pairs, **77 are certified by construction** (55 among the platform's rigid hardware: the base, the casters and their wheels, the lidar, the camera and
the mast base; and 22 between those and the two arms' base links, which are rigid on the mast plates) and are
evaluated once, in the default state, at each mast ID; the other **267 are sampled** (they have an arm joint or a wheel joint on the path).
The reasoning is tested against an independent forward kinematics (`test_acm_split.py`): every one of the 77 has the same relative pose at random
configurations. As a further check on the reasoning (not as a certification) the first 200 sampled states must agree with the one evaluation,
pair for pair; a disagreement is a FAIL that says the tree reasoning is wrong.

**What sampling does.** It falsifies; it does not certify. The sampled part of M03 means exactly this and nothing more: *no counterexample was
found among our 267 sampled pairs in 2000 random joint states at seed 20261006, at each of the four mast IDs.* Not "validated". Another seed or
more samples could find one, which is what the 14-of-458 diagnostic did for the inherited arm-arm pairs. The states are numpy
`default_rng(seed)` draws inside the joint limits (continuous joints in [-pi, pi]), the same states at every mast ID; collision is checked with an
allowed-collision matrix of its own that allows everything except the pairs under test, so nothing the SRDF says is consulted. The contact map is
not readable from Python (the `Contact` type is not bound), so the colliding pairs are found by bisection.

**Result at the recorded commit** (see the A7 section of `docs/verification.md` for the evidence file): **M03 PASS**. No disabled pair of ours
collides: the 77 by-construction pairs are clear in their one configuration at each mast ID, and none of the 267 sampled pairs collided in 2000
samples at seed 20261006. One inherited pair collides, as a **WARN**: `openarm_left_ee_link1` <-> `openarm_left_link2`, disabled as `Never`,
first at sample index 1112 of 2000 at seed 20261006, the same at every mast ID (it is arm-internal, so it cannot depend on the mast). It is
**not** re-enabled: the matrix is the contract and the finding is that it is a sampling conclusion. Note what the WARN's `origin` says: this pair
is not one of the 20 Enactic listed (`imported_from_upstream`); the Setup Assistant wrote it (`generated_for_upstream_links`), so "an upstream
observation" is true of the geometry and not of the matrix entry. And a `left_vs_right` pair is two Enactic links whose relative pose is set by
**our** mast plates (datums), so a collision there is arguably ours; the rule as specified treats it as inherited and the WARN says `arms:
left_vs_right`. Both are raised in the report.

**The mutation tests** (`test_m03_acm_sampling.py`) disable a pair that really collides as `Never` and require M03 to name exactly that pair by
the route that applies: `mast_base_link` <-> `openarm_left_ee_base_link` (ours, sampled: 7 of 300 states), `base_link` <-> `mast_base_link` (ours,
no joint between: by construction), and `openarm_left_link5` <-> `openarm_left_link6` (inherited: a WARN, exit 0). **The P10 version of this test
never worked**: it relabelled `openarm_left_link1` / `openarm_left_link2` and expected more than 10 hits in 100 samples; those links collide in
about 5 of 300 states, so it failed in P10's own run (`expected exit 1, got 0`) behind M03's real FAIL. The pairs above were measured to collide
with nothing ignored before they were used.

## M04: a goal inside the nearest reachable volume is refused

`test/test_m04_collision_goal_rejected.py` (VERIFICATION.md's M04 row used to say "base collision volume"; it is corrected to *reachable*,
because the arms cannot reach the base body) plans (plan only) to a left-arm joint state in which a left gripper link collides with
`mast_base_link`, the nearest structure volume they can reach, through `move_group`'s `/plan_kinematic_path`:

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
base-side volume a gripper can be put in. That the base body is out of reach is a reach fact for A4, not a defect here (and the lift decided on 7 Oct will change it).

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

`scripts/limits_sensitivity.py` (check **M06**, adopted 7 Oct) plans all twelve named poses under the configured limits scaled by 1, 0.1, 0.01
and 0.001 and under no acceleration limits at all. Result: **12 of 12 succeed under every scaling** (planned duration 16 s in total at x1,
517 s at x0.001: the effective limits go down to about 0.003 to 0.01 rad/s²) **and 0 of 12 succeed with none** (which reproduces #10). The plans
need *some* acceleration limit to exist and do not depend on how generous ours are, so the A7 stop condition ("planning succeeds only because
the limits we invented are generous") did not occur. **Both halves of the conclusion, or it is an overclaim:** planning requires that some
acceleration limits exist and is insensitive to our values across three orders of magnitude, which retires that stop condition; **and that does
not make the limits any less of a placeholder**: they stay engineering defaults (`source: engineering_default`, `status: placeholder`), with no
payload, torque or controller analysis behind them. The test fails if either half is missing from the recorded conclusion.

## Which physics engine M02 runs under, and why

`sim:=gazebo` includes the gazebo package's `sim.launch.py`, which **defaults to DART**: the engine platform-sw tuned the base on (decided
6 Oct). On DART the fingers do not follow the position command at the default gain and `finger_joint2` has no mimic constraint, so a
`closed` gripper pose may not close. **M02 passes `physics_engine:=gz-physics-bullet-featherstone-plugin` explicitly**, as G02 does, because
its poses include the grippers and the per-hand sign is part of what it checks; the arm poses do not depend on the engine. The base's contact
behaviour was compared on neither engine. `moveit.launch.py`'s `physics_engine` is empty by default, which means the simulator's default.

## The joint-state clamp: simulator float noise, and nothing else

MoveIt 2.12.4 refuses a request whose start state is any amount outside a joint's limits. The `CheckStartStateBounds` adapter is strict
("Joint 'openarm_left_joint4' from the starting state is outside bounds by: [-2.5e-07] should be in the range [0], [2.4435]"), there is no
`FixStartStateBounds` in the installed adapter set, and removing the adapter does not help because OMPL's planning context refuses the same state.
`home` puts joint 4 exactly on its lower limit, and Gazebo reads a joint resting on a limit back a hair past it. **MoveIt answers the generic
`FAILURE` (99999) to this, to the action and to the service alike** (an earlier version of this README said `START_STATE_INVALID`; that code is
not what the installed MoveIt returns on this path, and the reason is only in `move_group`'s log).

Of the three ways out the one taken is to clamp the joint state, and only that: widening a limit would invent vendor data (what ADR-009 exists to
forbid), and moving `home` would invent a pose (what `park`'s placeholder comment refuses to do). `scripts/joint_state_clamp.py` subscribes to
`/joint_states` and republishes it on `/joint_states_clamped`, which **only** `move_group` is remapped to (`move_group.launch.py`, argument
`clamp_joint_states`, default true; the raw topic everything else reads is untouched). Its logic, and the reasons, are in
`openamrobot_upperbody_moveit/joint_state_clamp.py`:

| | |
|---|---|
| **Epsilon** | `EPSILON = 1e-5` rad, a named constant. Measured in Gazebo here: `openarm_left_joint4` read back up to **9.9e-7** rad past its limit from a cold start (2585 of 3356 messages), 8.0e-7 at idle, `openarm_right_joint4` 2.6e-7 after a pose sequence; earlier sessions saw 2.5e-7 and 9e-8. A 1e-6 epsilon would clear the worst of those by one per cent, so it is ten times the largest value seen |
| **Clamps TO the limit** | a reading past a limit by at most epsilon is published *at* the limit, never beyond it |
| **Beyond epsilon is not noise** | a reading further out is published unchanged, logged as an error, counted as `over_epsilon`, and the request is still refused (`test_joint_state_clamp_loop.py` shows it); the clamp does not hide a real drift |
| **Logs** | every *episode* of clamping (a joint resting on its limit is one episode of many activations) is logged at WARN with the joint and the magnitude; each activation is logged at DEBUG and counted |
| **Counts** | `~/get_stats` returns `activations`, `episodes`, `max_magnitude_rad`, `over_epsilon`, per joint; M01 and M02 put them in their sidecars and the A7 report sums them in `clamp` |

A deterministic reproduction needs no Gazebo: `test_joint_state_clamp_loop.py` holds `openarm_left_joint4` at -2.5e-7 and shows the request
refused without the clamp (`clamp:=false`) and planning with it. The start gate of M01/M02 (wait until the joints are inside their limits for a
second) stays, widened by the epsilon: it waits out spawn transients, which the clamp does not touch.

**The retry loop is gone.** M01 and M02 used to retry a refused start state up to three times and count the retries. It never fired in the P10
record (0 retries at every mast ID and in Gazebo), it was keyed on a code MoveIt does not return here, and with the clamp a retry that never fires
is dead code and one that fires says something else is wrong. A refused request is now a failed pose.

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
* **M03 is a sampling result, not a certification** (above): it passes with one inherited WARN, and the sampled part means "no counterexample in
  2000 samples at seed 20261006", nothing stronger. The matrix is a checked-in contract that cannot be regenerated reproducibly.
* **MoveIt's start-state check has no tolerance, so the simulated joint state is clamped** (next section). `home` sets `openarm_*_joint4 = 0`,
  exactly that joint's lower limit, which is upstream's pose and is itself reportable upstream: it breaks planning from `home` for any
  float-valued state source.
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
| `test_acm_generation.py` | AUX | the recorded settings are the ones asked for (at least 10000 samples at `mast_1350`, never/adjacent/default on, always off), the SRDF holds exactly what they say, **the matrix's fingerprint is the one on record**, and a merge proposes and leaves the contract alone unless `--write` |
| `test_acm_split.py` | AUX | inherited/ours is derived from link provenance and partitions the matrix at every mast ID; a link added later lands on the right side; the by-construction pairs have a constant relative pose in an independent forward kinematics |
| `test_joint_state_clamp_logic.py` | AUX | the clamp: to the limit and never past it, nothing beyond epsilon, the counters, the limits are the URDF's, no named pose is touched |
| `test_joint_state_clamp_loop.py` | AUX | with a real `move_group`: the refusal without the clamp, the plan with it, and a reading beyond epsilon still refused and counted |
| `test_config_files.py` | AUX | limit tags and rule, controller names against `arm_controllers.yaml`, option 1 unchanged, OMPL defaults are MoveIt's |
| `test_gripper_sign.py` | M05 | per-hand gripper values; the single-number version fails, in both directions |
| `test_m03_acm_sampling.py` | M03 | no disabled pair of ours collides (by construction, or in 2000 samples at seed 20261006 at each mast ID); inherited collisions are WARNs; three mutation tests (ours sampled, ours by construction, inherited), and the bisection |
| `test_m04_collision_goal_rejected.py` | M04 | a goal in the nearest reachable volume (`mast_base_link`) is rejected; the pose goal times out |
| `test_m06_limits_sensitivity.py` | M06 | the plans need some acceleration limit and do not depend on its value |

M01 (mock, every mast ID) and M02 (Gazebo, `mast_1350`) are tests of `openamrobot_upperbody_bringup`: they start the stacks, and that package
depends on this one (a test dependency back would be a colcon cycle).

M05 and M06 were proposed by P10 and are adopted in `design/VERIFICATION.md` (7 Oct); the tests follow those rows.

See [`openamrobot_upperbody_description`](../openamrobot_upperbody_description/README.md) and
[`openamrobot_upperbody_bringup`](../openamrobot_upperbody_bringup/README.md).
