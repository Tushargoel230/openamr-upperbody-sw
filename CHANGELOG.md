# Changelog

All notable changes to this repository. Evidence for the description is static and mock only, and the Gazebo evidence is class
`simulation`; nothing here is a hardware result.

## [Unreleased]

### Added

- `openamrobot_upperbody_description`: the canonical OpenAMRobot 2.0 robot description (A6). One xacro entry point,
  `combined_robot.xacro`, composes the platform base (unchanged), a fixed mast selected by `mast_id` (`mast_1300`,
  `mast_1350` release, `mast_1400`, `mast_1450`) and two upstream OpenArm v2.0 arms rooted on mast plate links, with
  `arm_package` (`openarm`, `so101` stub, `none`), `hardware_profile` (`mock`, `gazebo`, `real`: selects the arm `ros2_control`
  plugin, see below) and `enable_planar_base`
  (a `planar` joint `world -> base_link`, present and disabled by default, axis `0 0 1`).
- Head camera and wrist camera frames as plain frames with a visual marker (the ZED wrapper is deferred).
- `config/datums.yaml` (15 datums with provenance: 8 placeholder, 6 provisional, 1 verified), its JSON schema, and
  `config/frame_contract.yaml`, the machine-readable frame and joint contract that the tests enforce.
- Verification suite and `verify_a6.sh`, which writes an `openamr.evidence.v1` report: expansion matrix, `check_urdf`,
  single root, frame contract, shoulder height per mast ID, argument validation, collision and inertia checks, mass report,
  planar joint, datum rules, configuration ids (`sha256:` plus 64 hex characters, matching the C9 dataset schema),
  installed-versus-source check, and warnings for the envelope (E01), KDL root inertia (W01), the planar joint under KDL
  (W02) and the missing visual inspection (V01). T14-T16 and L01 cover `ros2_control` (next entries).
- `view.launch.py` (in `openamrobot_upperbody_bringup`): robot_state_publisher, joint_state_publisher_gui and RViz2 for any
  mast ID. It started in the description package and moved to bringup with the mock bringup, with no forwarder.
- Arm `ros2_control` profiles (P06): `urdf/ros2_control.xacro`, `openamr_arm_ros2_control(hardware_profile)`, for
  `arm_package:=openarm` only. `mock` (`mock_components/GenericSystem`) and `real` (`openarm_hardware/OpenArmHW`, CAN-FD) call
  the upstream bimanual macro as upstream's own top-level v2.0 file does; `gazebo` is our own block with
  `gz_ros2_control/GazeboSimSystem` for the same joints and interfaces. The mobile base has no `ros2_control` block in any
  profile. Checks T14, T15, T16 (expansion only; nothing loads the Gazebo or real plugin).
- `openamrobot_upperbody_control/config/arm_controllers.yaml`: controller manager at 100 Hz, `joint_state_broadcaster`, a
  `JointTrajectoryController` per arm and a gripper controller per arm, one file for all three profiles. Derived from
  `enactic/openarm_ros2` at `4e837e1` (Apache-2.0); the header lists the changes.
- `openamrobot_upperbody_bringup/launch/mock.launch.py`: robot_state_publisher, `ros2_control_node` and the five spawners on
  mock hardware (args `mast_id`, `rviz`). Check L01 (`launch_testing`): all five controllers `active` within 30 s and one
  trajectory on `left_arm_controller` followed within 0.02 rad. Mock hardware reflects commands as state, so this shows the
  wiring, not tracking performance.
- `verify_a6.sh` also runs the control and bringup tests, and the evidence report reads the bringup results for L01.
- A6 evidence has no SKIP: T14-T16 and L01 are results, under `mock_components/GenericSystem` (19 PASS, 6 WARN, 0 SKIP,
  0 FAIL at `2083c76`). That is weaker than hardware, and `evidence_class` stays `static+mock`: no controller has driven a
  motor.
- `config/upstream_pins.yaml`: the config id falls back to this pinned `openarm_description` commit when there is no git
  checkout (a checkout with local changes is still an error); a test keeps it equal to the checkout and the baseline pin.
- T02 also runs `check_urdf` on `gazebo` and `real` at `mast_1300` (labelled extras).
- `.gitattributes`: `*.sh text eol=lf`, so `verify_a6.sh` cannot pick up CRLF through the `/mnt/c` checkout.
- T10 reports, after the modelled total, the `gross_mass_limit` it compared against and the F2S vehicle estimate (93 kg), each
  with its source, and prints "per-link masses still placeholders" if and only if a reported group draws a mass from a datum
  whose status is `placeholder`. The clause is computed from `datums.yaml` on every run, so it disappears when real masses
  land. T10 is still report-only: it fails only when the modelled total exceeds `gross_mass_limit`.
- `f2s_vehicle_mass_estimate` datum (`report_only`, source `f2s`, status `provisional`): Alex's F2S design proposal of 28 Sep,
  about 93 kg gross against the 120 kg cap. Second-hand: the F2S document itself is not in this repository.
- Check C02: `config/upstream_pins.yaml` must equal the HEAD of a clean `src/upstream/openarm_description`. A difference is a
  FAIL naming both hashes. With no checkout it is a SKIP that says so, the only one this tree is allowed to have.
- `docs/verification.md`: how to run `verify_a6.sh`, the evidence report field by field, what `static+mock` permits, and the
  six WARNs with what would clear each.
- READMEs for the description package and, stating their scaffold status, for the control, bringup, Gazebo and MoveIt
  packages.
- Test dependencies `python3-numpy` and `liburdfdom-tools` declared in the description package, so
  `rosdep install --from-paths src` installs what T02 and T05 use.

- `openamrobot_upperbody_gazebo` (A7, part 1): `sim.launch.py` (args `mast_id`, `world`, `headless`, plus `physics_engine` and
  `rviz`) spawns the combined robot in Gazebo Harmonic with the base on platform-sw's DiffDrive plugin and both arms on
  `gz_ros2_control`; an empty world; platform-sw's `gz_bridge.yaml` used by reference, plus one bridge for the wheels' joint
  states; `joint_state_publisher` merging `/base/joint_states` and `/arm/joint_states` so `/joint_states` has one publisher;
  `odom -> base_link` from the DiffDrive plugin only. The `gazebo` profile loads `gz_ros2_control` into the model (model plugin,
  `gz_ros2_control-system`, with `arm_controllers.yaml` by installed path), names read from the installed library.
- Checks G01 (headless smoke test: five controllers `active`, an arm goal completes, platform-sw's topics unchanged by name and
  type, `/cmd_vel` and `/odom` work, one `odom -> base_link` publisher, `/joint_states` once each) and G02 (grippers close and
  `finger_joint2` follows in the physics). `verify_gazebo.sh` writes a `gazebo` evidence report, `evidence_class: "simulation"`.
- `docs/v01_visual_check.md`: the checklist for V01, to be answered by a person with the simulator and RViz open.
- Check T15 gains the finger-mimic multiplier test (a hard-coded multiplier, including one equal to the yaml's, fails), and the
  plugin tests. T10 gains the mass-provenance tripwire (a verified mass datum with no base/arm provenance entry fails).

### Changed

- The evidence harness writes no report from a dirty tree: it prints the results, says `Evidence: NOT WRITTEN`, and exits 3 if
  nothing failed. Every file in `evidence/` is a clean-commit run.
- C02 checks every pin in `config/upstream_pins.yaml`, not only `openarm_description`, and the file is normalised to quoted
  40-hex hashes (`openarm_ros2` was a 7-character hash nothing compared). A pin with no checkout is listed as not compared; C01
  compares every pin with the manipulation baseline.
- In `ros2_control.xacro` the finger-mimic multiplier property moved from inside the mock/real branch to file scope so it can be
  read and tested; the expansion of every profile (mock, real; gazebo before the plugin was added) is byte-identical.
- Repository README: status updated, and a pointer to the description package. The mast configuration text is unchanged.
- `mast_section` datum: the invented 80 mm square placeholder is now the MISUMI HFS6-60120 profile, 60 x 120 mm (`source: plan`,
  `status: provisional`; which axis carries the 120 mm is unconfirmed). Datums revision `2026-10-08.d`, so every
  configuration id changed.
- `package.xml` descriptions of the bringup, control and MoveIt packages describe the 2.0 fixed-mast scope instead of a lift.
- The control README's note on the gripper controller's deprecation warning now states, in order: what is in use, that it is
  a warning and not a removal, that the replacement serves a different action type, that the warning misspells the
  replacement's name, and that the choice is manipulation#11, ADR-009 point 4 and P10's. The controller type is unchanged.
- Datums revision `2026-10-08.e` (the new F2S datum), so every configuration id changed. C01 no longer compares the pin file
  with a checkout (that is C02); it keeps the format and the baseline-pin comparison.

- `sim.launch.py`'s `physics_engine` defaults to DART again (`gz-physics-dartsim-plugin`), not Bullet-Featherstone: a
  gripper-mimic convenience is not grounds to re-baseline platform-sw's physics. G02 passes `gz-physics-bullet-featherstone-plugin`
  explicitly and its sidecar records which engine each Gazebo check ran on.

### Removed

- `config/lift/`: an empty scaffold directory. An actuated lift is OpenAMRobot 3.0 scope.

### Known limitations

- Base inertials are platform-sw CAD placeholders; the mast mass, mast and plate transforms and all camera poses are
  placeholder datums. No physical evidence.
- `ros2_control` runs on `mock` (L01) and in Gazebo (G01, G02); the `real` block is checked by expansion only,
  `openarm_hardware` cannot be built in the reference workspace (it needs OpenArmCAN). In Gazebo the arms' position interface is
  a velocity servo, so tracking there says nothing about the real arm. `finger_joint2` is not declared under `ros2_control`:
  Gazebo's Bullet-Featherstone engine follows the URDF `<mimic>` on its own, its DART default does not move the fingers at all.
- The Gazebo launch's `physics_engine` defaults to DART, the engine platform-sw tuned the base on, and the grippers do not close
  there. G02 passes Bullet-Featherstone explicitly. The base's contact behaviour was compared on neither engine (G01 measures
  kinematics).
- The gripper controllers stay `position_controllers/GripperActionController` (deprecated in Jazzy) on purpose: the replacement,
  `parallel_gripper_action_controller`, serves `ParallelGripperCommand` instead of `GripperCommand`, so switching decides the
  gripper interface (manipulation#11, ADR-009 point 4), which is open.
- Issue #7: the description strings it names are fixed here; whether that closes it is for the code owners to judge.
