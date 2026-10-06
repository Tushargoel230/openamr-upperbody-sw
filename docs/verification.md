# Verifying the robot description (A6)

`verify_a6.sh` runs the description, control and bringup tests, then writes one JSON evidence report. This page says how
to run it, what the report contains, what it lets you claim, and why six checks are `WARN`.

## Run it

```bash
source /opt/ros/jazzy/setup.bash
cd ~/openamr_ws
colcon build --symlink-install --packages-skip openarm_hardware openarm   # the tests describe the INSTALLED package (I01)
source install/setup.bash
ros2 run openamrobot_upperbody_description verify_a6.sh                   # about 2 minutes
```

It prints a summary and writes `<workspace>/evidence/<UTC timestamp>_dev.json`. The exit status is non-zero if and only if
a check is `FAIL`. `OPENAMR_EVIDENCE_DIR` moves the output; `OPENAMR_WS` names the workspace.

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
| V01 | No person has looked at the RViz window. A numeric check is not a visual one | A person runs `ros2 launch openamrobot_upperbody_bringup view.launch.py`, looks, and records it |

## Reading a report

Check `git_hash` first (every file in `evidence/` is from a clean commit), then `limitations`, then every non-`PASS` row. T10 prints the modelled mass beside the
120 kg limit and the F2S vehicle estimate (93 kg); the modelled total is what the description contains, not what the robot
weighs.
