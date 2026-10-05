# openamrobot_upperbody_description

The canonical OpenAMRobot 2.0 robot description: the platform base, a fixed MISUMI mast selected by `mast_id`, and two
OpenArm v2.0 arms, expanded from one xacro entry point.

> **Evidence class: static + mock.** Everything here is verified by xacro expansion, `check_urdf`, zero-pose
> forward kinematics and unit tests. Nothing has been launched on hardware, nothing has run in Gazebo, and no `ros2_control`
> is emitted yet. Read [Known limitations](#known-limitations) before relying on any number in the model.

## What it is

`urdf/combined_robot.xacro` composes, without editing any of them:

```
base_link                       platform-sw (openamrobot_description), included unchanged
  -> mast_base_link             mast.xacro, placeholder box + inertial
       -> mast_mount_link       the plate reference frame; height set by mast_id
            -> mast_{left,right}_arm_mount_link    emitted by the upstream OpenArm macro
                 -> openarm_{left,right}_*         two calls of upstream `openarm_v20_robot`, one per arm
                      -> {left,right}_wrist_camera_link -> *_optical_frame     upper_sensors.xacro
            -> head_camera_link -> head_camera_optical_frame                   upper_sensors.xacro
            -> so101_mount_link                    only with arm_package:=so101 (stub)
world -> base_planar_joint -> base_link             only with enable_planar_base:=true
```

The default expansion has 40 links and one root, `base_link`. There is no OpenArm body: the arms bolt to the mast, so
each arm is rooted on its own plate link. Upstream `openarm_description` is a pinned dependency and is never modified.

The mast height is **configuration, not state**. `mast_id` selects one of four shoulder-axis heights; changing it is a
different robot configuration, not a joint to move. Every combination of the args below has its own
configuration id (see [Configuration id](#configuration-id)).

## Arguments

| Arg | Values | Default | Effect |
|---|---|---|---|
| `mast_id` | `mast_1300`, `mast_1350`, `mast_1400`, `mast_1450` | `mast_1350` | Shoulder-axis height above the floor in mm. `mast_1350` is the installed release configuration; the other three are analysis positions, not installed configurations. Mast top is 1500 mm; 1700 mm is the maximum assembled robot height, not a shoulder height. |
| `arm_package` | `openarm`, `so101`, `none` | `openarm` | `openarm`: two upstream OpenArm v2.0 arms. `so101`: a single `so101_mount_link` stub on `mast_mount_link`, nothing below it. `none`: base and mast only. |
| `hardware_profile` | `mock`, `gazebo`, `real` | `mock` | Validated and part of the configuration id, but **has no effect on the model yet**: no `ros2_control` block is emitted. Anything other than `mock` prints a warning saying so. |
| `enable_planar_base` | `true`, `false` | `false` | `true` adds a `world` link and `base_planar_joint` (type `planar`, axis `0 0 1`) above `base_link`. Off by default: base and arm never move together in OpenAMRobot 2.0. |
| `datums_file` | path | `config/datums.yaml` | Alternative datum file, used by the tests. |

An unknown value for any arg stops expansion with an error that lists the accepted values.

```bash
xacro $(ros2 pkg prefix openamrobot_upperbody_description)/share/openamrobot_upperbody_description/urdf/combined_robot.xacro \
      mast_id:=mast_1400 arm_package:=openarm > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

## Build

From a workspace that contains this repo, `openamr-platform-sw` and the pinned upstream `openarm_description`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-skip openarm_hardware openarm
source install/setup.bash
```

`openarm_hardware` and the `openarm` metapackage need the OpenArm CAN library and are not part of the description.
The tests and the evidence report describe the **installed** package, so rebuild before running them.

## View it

```bash
ros2 launch openamrobot_upperbody_description view.launch.py mast_id:=mast_1350
```

This starts `robot_state_publisher`, `joint_state_publisher_gui` and RViz2 (fixed frame `base_link`). Source the workspace
overlay first: the OpenArm and base meshes are `package://` URIs that only resolve through it, and without it RViz loads
the model but draws the arms and base as missing geometry. The launch accepts `mast_id`, `arm_package` and
`enable_planar_base`. With `enable_planar_base:=true` the robot cannot be driven from here: `robot_state_publisher` treats
the planar joint as fixed (check W02).

The launch has started without xacro or mesh errors, but **no person has looked at the RViz window yet** (check V01 is a
WARN until someone does).

## Add a mast ID

A mast ID is one entry in `config/mast_ids.yaml`:

```yaml
mast_ids:
  mast_1500: {shoulder_height: 1.500, class: analysis}     # metres above the floor; class: release | analysis
```

The accepted-value check in `combined_robot.xacro`, the mast offset (`mast.xacro`), the expansion matrix and the tests
all read this file, so nothing else needs a code change. Then:

1. Only one ID may be `class: release`, and it must be `default`; `analysis` IDs are not installed configurations.
2. The shoulder height must be compatible with the mast top and the maximum assembled height in the same file.
3. Update the ID lists in the `view.launch.py` description strings and in this README.
4. Run `verify_a6.sh`: T05 asserts that `openarm_left_base_link` sits at exactly the new shoulder height, and the
   configuration id of the new mast ID will differ from every existing one.

A mast ID that does not follow a recorded decision should not be `class: release`.

## Add an arm package

`arm_package` selects a model subtree and nothing more. It does **not** define what an arm can do: capability semantics,
the Device Package schema and its interfaces are the Manipulation team's I2 work (Om Mangesh Jagtap, Adnan Khalid). This
package only decides which links and joints are attached to the mast.

To add a package:

1. Add the value to the accepted list and a new `xacro:if` block in `urdf/combined_robot.xacro`, attached to
   `mast_mount_link` (or to `mast_left_arm_mount_link` / `mast_right_arm_mount_link` if it uses the OpenArm plates).
2. Add the frames to `config/frame_contract.yaml` with `req: C` and `condition: "arm_package == <value>"`. T04 fails on
   any link in our namespaces that the contract does not list.
3. Add the value to `ARM_PACKAGES` in `test/conftest.py` and to `expansion_matrix` / `planar_extras` in
   `scripts/evidence_common.py`, so T01, T02 and the configuration ids cover it.
4. Every number the new subtree needs goes in `config/datums.yaml` (below), not in the xacro.

The `so101` stub is the model: a link, a fixed joint, and a note in the frame contract that A9 owns everything below it.

## Datums

Every number in the model that is not owned by upstream OpenArm lives in `config/datums.yaml`, with its provenance.
Computed values (for example the mast offset) are not stored; upstream values are read from upstream YAML, not copied.

```yaml
datums:
  <name>:
    value: <number | [x, y, z] | {xyz: [..], rpy: [..]}>
    source: <measured | contractor_m01 | f2s | supplier | plan | upstream | derived | placeholder>
    status: <placeholder | provisional | verified>
    ref: "<document, drawing number, file:line, or person + date>"
    owner: "<who can change it>"
    note: "<optional>"
```

Rules, enforced by tests T12 and T13 (`config/datums.schema.json` holds the schema):

1. `source` and `status` are mandatory and from the lists above.
2. `status: verified` needs a measured or documented source (`measured`, `contractor_m01`, `f2s`, `supplier`, `plan`)
   and a non-empty `ref`.
3. `source: placeholder` means `status: placeholder`. **Never write a supplier or plan figure as verified if it is not.**
4. Every datum is either referenced by the xacro, listed under `report_only` (reported, never feeds the model), or
   listed under `pending_reference` with the step that will consume it. A datum in none of the three is a failure.
5. Change a value, bump `revision`. The revision goes into every configuration id, and `test_datums_revision.py` fails
   if the content changed against `HEAD~` and the revision did not.
6. A number that only affects how the model is *drawn* (the camera marker cube edge, `openamr_sensor_marker_edge`) is a
   named visual-only constant in the xacro, not a datum. It may appear only inside a `<visual>`.

Current state (revision `2026-10-08.d`): **14 datums: 8 placeholder, 5 provisional, 1 verified.**
`ros2 run openamrobot_upperbody_description datums_report.py` lists them with their owners.

## Verify

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run openamrobot_upperbody_description verify_a6.sh
```

`verify_a6.sh` runs this package's `colcon test`, the `check_urdf` matrix, the datum report and the configuration ids, and
writes `<workspace>/evidence/<UTC timestamp>_dev.json` (`openamr.evidence.v1`, evidence class `static+mock`). Set
`OPENAMR_EVIDENCE_DIR` to put it elsewhere. The exit status is non-zero if and only if any check is `FAIL`. It needs
`colcon`, `xacro`, `check_urdf` (`liburdfdom-tools`) and `numpy` on the machine; they are declared as test dependencies, so
`rosdep install --from-paths src` installs them.

Result vocabulary: `PASS`, `FAIL`, `WARN`, `SKIP`. **`SKIP` and `WARN` are not `PASS`.**

| Check | What it proves |
|---|---|
| T01 | all four `mast_id` x three `arm_package` on `mock`, plus `openarm` on `gazebo` and `real` at `mast_1350`, expand |
| T02 | `check_urdf` accepts every T01 expansion and the planar-flag expansions |
| T03 | exactly one root and no duplicate link names |
| T04 | the frame contract: required frames and parents, conditional frames, the expected-from-other-owner frames |
| T05 | `openarm_left_base_link` is at the shoulder height of its `mast_id`, within 1 mm, for all four IDs |
| T06 | an unknown arg value fails with the accepted values named |
| T07 / T08 | collision geometry exists or is declared absent; a collision mesh that is the visual mesh is reported |
| T09 / T10 | every massed link has positive mass and a physically valid inertia; the modelled mass is reported by owner against 120 kg |
| T11 | no planar joint by default; with `enable_planar_base:=true` a `planar` joint, parent `world`, child `base_link` |
| T12 / T13 | datum schema, provenance rules and references |
| C01 | every configuration id is `sha256:` plus 64 hex characters and matches the C9 dataset schema pattern |
| I01 | the installed package matches the source tree (catches evidence generated against a stale install) |
| REV | a datum change carries a revision change |
| T14, T15, T16, L01 | **SKIP**: these are `ros2_control` checks and no controllers exist yet (see `openamrobot_upperbody_control`) |
| E01 | **WARN, never PASS**: height, width and depth as lower bounds, see below |
| W01, W02, V01 | **WARN**: see Known limitations |

### Configuration id

`ros2 run openamrobot_upperbody_description print_config_id.py mast_id:=mast_1350 arm_package:=openarm hardware_profile:=mock` prints the id of one expansion:
`sha256:` followed by the SHA-256 (64 hex characters) of the canonicalised expanded URDF, the three args, the datum revision
and the pinned upstream commit. A 12-character prefix is for log lines only and is labelled as a prefix wherever it is
shown. `hardware_profile` is part of the id because the id names the configuration, not just the geometry.

## Known limitations

Nothing in this package is a measured value or a hardware result. In particular:

- **Base inertials are platform-sw CAD placeholders.** The base comes from `openamr-platform-sw` unchanged; its masses and
  inertias are CAD-export values. The mast mass is a placeholder and the arm masses are upstream nominal values.
- **Mast and plate transforms are placeholders.** `mast_base_pose`, `left/right_arm_mount_pose` and `mast_mass`
  are placeholder datums awaiting the general-arrangement and mast drawings. `mast_section` is provisional: the MISUMI
  HFS6-60120 profile is 60 x 120 mm, but which axis carries the 120 mm is unconfirmed. **T05 asserts that the
  shoulder height equals `shoulder_height(mast_id)`, which holds for any datum values; it does not validate the datums.**
- **The head camera sits above the end of the mast at the two highest IDs.** `head_camera_pose` is a guess, +0.10 m above
  the shoulder plate. At `mast_1400` and `mast_1450` the head-camera marker is the highest point on the robot, and at
  `mast_1450` it is 0.0756 m above the 1.500 m mast top, so the model shows a camera floating beyond the end of the mast.
  This is a placeholder artefact, but it raises a real question for the mechanical owner: the mast top is 1500 mm and the
  shoulder plate is at 1350-1450 mm, so where does the head camera actually mount?
- **The ZED wrapper is deferred.** The head camera and wrist cameras are plain frames with a marker cube, not sensor
  models. The upstream ZED wrapper belongs with camera calibration.
- **E01 is WARN.** Meshes are not loaded, so height, width and depth come from link origins and box, cylinder and sphere
  extents only. They are lower bounds: a figure under the limit proves nothing, and a figure over it is real. The
  envelope is 1700 x 600 x 800 mm (provisional). Width is the one to look at.
- **W01: KDL ignores the root-link inertia.** `base_link` carries an inertial from platform-sw, and KDL drops the inertia
  of a root link, so `robot_state_publisher` and MoveIt never see it. Not fixed here: the usual workaround adds a dummy
  root above `base_link`, which would change the frame contract's root, and the base belongs to platform-sw.
- **W02: `robot_state_publisher` converts the planar joint to fixed**, because KDL has no planar joint. The joint is
  present and `check_urdf` accepts it, which is what A6 asks for, but planar base motion can never come from
  `robot_state_publisher`; in simulation it comes from the Gazebo `odom -> base_link` transform.
- **V01: nobody has looked at the RViz window.** Numeric forward kinematics is not a visual check.
- **No `ros2_control`, no Gazebo, no MoveIt.** T14-T16 and L01 are SKIP.
- **No physical evidence.** Static expansion and zero-pose forward kinematics only.
