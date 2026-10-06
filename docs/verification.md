# Verifying the robot description (A6) and its simulation (A7)

`verify_a6.sh` runs the description, control and bringup tests, then writes one JSON evidence report. This page says how
to run it, what the report contains, what it lets you claim, and why six checks are `WARN`. The Gazebo report and the A7 report
(MoveIt, `run_a7_regression.sh`) follow the A6 one.

## Run it

```bash
source /opt/ros/jazzy/setup.bash
cd ~/openamr_ws
colcon build --symlink-install --packages-skip openarm_hardware openarm   # the tests describe the INSTALLED package (I01)
source install/setup.bash
ros2 run openamrobot_upperbody_description verify_a6.sh                   # about 2 minutes
```

It prints a summary and writes `<workspace>/evidence/<UTC timestamp>_dev.json`, **from a clean tree only**: with uncommitted
changes (untracked files included) it prints the results, says `Evidence: NOT WRITTEN` and names the files, because a report
records a commit and a dirty tree is none. The exit status is 1 if a check is `FAIL`, 3 if everything passed but the tree was
dirty, 2 if the evidence cannot be assembled. `OPENAMR_EVIDENCE_DIR` moves the output; `OPENAMR_WS` names the workspace.

## The evidence report (`openamr.evidence.v1`)

| Field | Meaning |
|---|---|
| `schema`, `repo`, `profile` | `openamr.evidence.v1`, `openamr-upperbody-sw`, `dev` |
| `git_hash`, `dirty` | The commit the report describes. `dirty` is always `false` in a written file: the harness refuses to write evidence from a dirty tree (it prints the results, says `Evidence: NOT WRITTEN` and names the files, and exits 3 if nothing failed), because a report from uncommitted changes describes no commit and a file that exists gets read |
| `timestamp` | UTC, when the report was built |
| `evidence_class` | Always `static+mock`; see below |
| `upstream` | Commits of `openarm_description`, `openarm_ros2` and `openamrobot-manipulation`. Without a git checkout of `openarm_description` its commit is the pin in `config/upstream_pins.yaml`, and `limitations` says so |
| `datums_revision` | `revision` of `datums.yaml`; one input of every config id |
| `checks[]` | `id`, `name`, `result`, and `details` (pytest counts plus whatever the check recorded) |
| `config_ids` | `sha256:` plus 64 hex characters, one per expansion (mast ID x arm package x profile). Two reports with the same id describe the same URDF, datums revision and upstream commit |
| `provenance` | Datum counts by status: `placeholder`, `provisional`, `verified` |
| `limitations` | What the report does not establish, in words. Read it before quoting the report |

## What `static+mock` permits

It permits: the xacro expands for every mast ID, arm package and hardware profile; `check_urdf` accepts the result; frames,
inertials and the planar joint have the properties the checks name; and the five controllers reach `active` on
`mock_components/GenericSystem` and follow one trajectory (L01).

It does not permit any claim that a controller has driven a motor, anything about Gazebo, MoveIt or hardware, tracking
performance (mock hardware reflects commands back as state), or the real mass, envelope or mast geometry (placeholder
datums). Words such as "validated" or "works on hardware" do not belong in anything quoting it. The shared rules say it
directly: *"SKIP/BLOCKED is not PASS; fixtures/fake hardware are not integrated simulation, physical acceptance or
release readiness."*

## The Gazebo report (`verify_gazebo.sh`)

```bash
ros2 run openamrobot_upperbody_description verify_gazebo.sh            # about 1.5 minutes: two headless Gazebo servers
```

It runs the launch tests of `openamrobot_upperbody_gazebo` and writes `<workspace>/evidence/<UTC timestamp>_gazebo.json` with
`profile: "gazebo"` and `evidence_class: "simulation"`, plus a `simulator` block (Gazebo, `gz_ros2_control` and `ros_gz_sim`
versions). It refuses to write from a dirty tree, like the other report. Its checks:

| ID | Proves |
|---|---|
| G01 | in a headless Gazebo on the launch's default engine (DART): the five arm controllers are `active`; a `left_arm_controller` goal completes and tracks while the other arm and the base stay still; every topic of platform-sw's bridge file exists by name and type; `/cmd_vel` moves the base and `/odom` reports it; exactly one publisher of `odom -> base_link` (strictly increasing stamps at the plugin's 50 Hz); `/joint_states` has one publisher carrying the wheels and the arms once each |
| G02 | both grippers close and `finger_joint2` follows `finger_joint1` in the physics (link poses, not `/joint_states`); it passes `physics_engine:=gz-physics-bullet-featherstone-plugin` explicitly, because the launch defaults to DART (platform-sw's engine) and the grippers do not close there: on DART it fails, which is the documented limitation, not a defect |
| I01 | the installed gazebo and description packages match the source |
| V01 | `WARN` until a person has answered `v01_visual_check.md`; "looks fine" does not clear it |

**What `simulation` permits:** the model behaves as G01 and G02 say in that simulator on that machine. **What it does not:** any
statement about a physical robot, about the real arm's controller, gains or latency (the arms' position interface is a velocity
servo in `gz_ros2_control`), about payload, torque or mass (the masses are placeholders), about the base's real wheel or
motor parameters (platform-sw's plugin values, not measured), or about how the base behaves on the physics engine platform-sw
tuned for (DART). The report's last limitation is the A6 report's own sentence: *no physical evidence*.

## The A7 report (`run_a7_regression.sh`)

```bash
ros2 run openamrobot_upperbody_bringup run_a7_regression.sh            # about 15 minutes; headless
```

It runs the gazebo, moveit and bringup tests (`OPENAMR_A7=1`, which un-gates M01 and M02: they skip otherwise so that `verify_a6.sh` never
starts them), then `build_a7_evidence.py` writes `<workspace>/evidence/<UTC timestamp>_a7.json` with `profile: "a7"`, `evidence_class:
"simulation"`, a `simulator` block (Gazebo, `gz_ros2_control`, `ros_gz_sim` and MoveIt versions) and a `moveit_config` block (hashes of the
SRDF, which carries the collision matrix, and of the other configuration files). It inherits P08's rule: **a dirty tree writes no file**. The
exit status is 1 if a check is `FAIL`, 3 if everything passed but the tree is dirty, 2 if the evidence cannot be assembled.

| ID | Proves |
|---|---|
| G01, G02 | as in the Gazebo report above (G01 on the default DART engine, G02 on Bullet-Featherstone passed explicitly) |
| M01 | on `mock`, at **each of the four mast IDs**, every SRDF named pose plans and executes through `move_group` into the real controllers and ends under 0.01 rad of its goal: both arms (`hands_up`, `home`, `hands_up`, `park`) and both grippers (`closed` then `open`, each hand with its own value) |
| M02 | the same in Gazebo at `mast_1350` (the release configuration), on Bullet-Featherstone passed explicitly because the gripper poses do not close on DART |
| M03 | the collision matrix validated by sampling: 2000 random joint states per mast ID, collision checked ignoring the SRDF's list, a pair disabled as `Never` that collides is a FAIL with the pair and a reproducing sample; the `Never` pairs that never collided are listed |
| M04 | a goal that puts the left gripper inside a base-side collision volume is **rejected** by planning (the joint goal: answered at once, the planner's own verdict `GOAL_STATE_INVALID`), not merely unsuccessful; the pose goal at the same place **times out**, and the record says which is which |
| M05 | *(proposed by this step)* the per-hand gripper values: `closed` is each hand's own far limit, the signs are opposite, and the single-number version fails (ADR-009 point 5) |
| M06 | *(proposed by this step)* the named-pose plans need some acceleration limit and do not depend on its value (ADR-009) |
| L01, I01, V01, AUX | as in the A6 report, for the packages this one touches; V01 now has **eight** checklist items (item 8: an arm executes a named pose in RViz and the preview matches what the robot then does) and stays `WARN` until a person has answered them |

**What `simulation` permits here:** every claim reads *"plans and executes in simulation under declared OpenAMRobot limits"*. The limits
are engineering defaults (`source: engineering_default`, `status: placeholder`) because upstream declares no acceleration limits
(manipulation#10, ADR-009); they are not vendor data. **What it does not permit:** any statement about a physical robot, a payload, a
torque, a validated limit, the real arm's controller or latency (the Gazebo position interface is a velocity servo), or the base's
behaviour on the engine platform-sw tuned (DART, on which the grippers do not close).

The A7 WARNs and findings, beyond the A6 ones (W01, W02, E01, T04, T08, V01 above):

| Item | What | What would clear it |
|---|---|---|
| W01 in A7 | MoveIt never sees `base_link`'s 4.18 kg inertial: KDL drops the inertia of a root link | not clearable here; matters for nothing planned (the base does not move) |
| `park` | a placeholder copy of `home` | Adnan's safe-pose definition under the manipulation action contract |
| acceleration limits | engineering defaults, tagged placeholder | vendor data (manipulation#10) |
| gripper interface | option 1 is in use; the choice is open on manipulation#11 | Om and Adnan; the evidence is in the MoveIt package README |
| start-state tolerance | MoveIt 2.12.4 refuses a start state 1e-7 rad past a limit; in Gazebo `home` (joint 4 on its lower limit 0) can read that way, so planning from `home` is intermittently refused. M01/M02 retry up to 3 times per pose and count every retry | a decision on where a tolerance may live (MoveIt bounds, the simulator's joint state, or a `home` off the limit) |
| `move_group` shutdown | exits with SIGSEGV when stopped; allowed for that one process | upstream MoveIt |
| M03 | see the report: when a pair disabled as `Never` collides, the ACM is wrong for it and the pair is not re-enabled by the check | a decision on how the matrix is certified (more samples, or enabling the arm-arm pairs a larger run finds) |

## Result values

| Value | Meaning |
|---|---|
| `PASS` | The check ran and its condition held |
| `WARN` | It ran, and found something a reader must know; not a defect in this package. It is not `PASS` |
| `SKIP` | It did not run, with the reason. Not `PASS`. The only legitimate `SKIP` is C02 with no `src/upstream/` checkout |
| `FAIL` | A condition was violated, or no result was recorded. Always fails the run |

Current result: 20 `PASS`, 6 `WARN`, 0 `SKIP`, 0 `FAIL` with the upstream checkout present.

## The six WARNs

| ID | Why a WARN and not a FAIL | What would clear it |
|---|---|---|
| T04 | Six base sensor frames (`tof_*`, `ultrasonic_*`) expected from platform-sw are missing. The base is not ours (ADR-005) | platform-sw adds them (WP-F5, static TFs into URDF sensor macros) |
| T08 | Six platform-sw base links (`base_link`, four casters, `lidar_link`) use the full-detail visual STL as collision mesh: slower planning, not a wrong model | platform-sw supplies simplified collision meshes |
| E01 | Height, width and depth are lower bounds from link origins and primitives; meshes are not loaded. A figure under the limit proves nothing, so it is never `PASS` by design | A4 measures the envelope with meshes. Until then it stays `WARN` |
| W01 | `base_link` has a 4.18 kg inertial and KDL drops the inertia of a root link, so `robot_state_publisher` and MoveIt never see it. The usual workaround changes the contract's root, and the base is platform-sw's | A base-macro refactor by the WP-F5 owner. It matters in A7 |
| W02 | `robot_state_publisher` converts the planar joint to fixed, because KDL has no planar joint. A consumer limitation; A6's acceptance (joint present, `check_urdf` accepts it) is unaffected | Not clearable here. In A7 planar motion comes from Gazebo's `odom -> base_link` |
| V01 | No person has answered the visual checklist. A numeric check is not a visual one | A person runs the simulator with RViz beside it, looks, and answers each item of [`v01_visual_check.md`](v01_visual_check.md) in their own words |

## Reading a report

Check `git_hash` first (every file in `evidence/` is from a clean commit), then `limitations`, then every non-`PASS` row. T10 prints the modelled mass beside the
120 kg limit and the F2S vehicle estimate (93 kg); the modelled total is what the description contains, not what the robot
weighs. Each mass row carries its provenance state from `config/mass_provenance.yaml`: `placeholder` (no source), `external_nominal`
(another owner's figure, not verified by us: the base CAD export and the upstream arm nominals) or `verified` (measured, with a
document); none is `verified` today.
