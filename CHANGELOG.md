# Changelog

All notable changes to this repository. Evidence for the description is static and mock only; nothing here is a hardware
or Gazebo result.

## [Unreleased]

### Added

- `openamrobot_upperbody_description`: the canonical OpenAMRobot 2.0 robot description (A6). One xacro entry point,
  `combined_robot.xacro`, composes the platform base (unchanged), a fixed mast selected by `mast_id` (`mast_1300`,
  `mast_1350` release, `mast_1400`, `mast_1450`) and two upstream OpenArm v2.0 arms rooted on mast plate links, with
  `arm_package` (`openarm`, `so101` stub, `none`), `hardware_profile` (`mock`, `gazebo`, `real`: selects the arm `ros2_control`
  plugin, see below) and `enable_planar_base`
  (a `planar` joint `world -> base_link`, present and disabled by default, axis `0 0 1`).
- Head camera and wrist camera frames as plain frames with a visual marker (the ZED wrapper is deferred).
- `config/datums.yaml` (14 datums with provenance: 8 placeholder, 5 provisional, 1 verified), its JSON schema, and
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
- `config/upstream_pins.yaml`: the config id falls back to this pinned `openarm_description` commit when there is no git
  checkout (a checkout with local changes is still an error); a test keeps it equal to the checkout and the baseline pin.
- T02 also runs `check_urdf` on `gazebo` and `real` at `mast_1300` (labelled extras).
- `.gitattributes`: `*.sh text eol=lf`, so `verify_a6.sh` cannot pick up CRLF through the `/mnt/c` checkout.
- READMEs for the description package and, stating their scaffold status, for the control, bringup, Gazebo and MoveIt
  packages.
- Test dependencies `python3-numpy` and `liburdfdom-tools` declared in the description package, so
  `rosdep install --from-paths src` installs what T02 and T05 use.

### Changed

- Repository README: status updated, and a pointer to the description package. The mast configuration text is unchanged.
- `mast_section` datum: the invented 80 mm square placeholder is now the MISUMI HFS6-60120 profile, 60 x 120 mm (`source: plan`,
  `status: provisional`; which axis carries the 120 mm is unconfirmed). Datums revision `2026-10-08.d`, so every
  configuration id changed.
- `package.xml` descriptions of the bringup, control and MoveIt packages describe the 2.0 fixed-mast scope instead of a lift.

### Removed

- `config/lift/`: an empty scaffold directory. An actuated lift is OpenAMRobot 3.0 scope.

### Known limitations

- Base inertials are platform-sw CAD placeholders; the mast mass, mast and plate transforms and all camera poses are
  placeholder datums. No physical evidence.
- `ros2_control` is verified by a runtime launch on `mock` only. The `gazebo` and `real` blocks are checked by expansion;
  `openarm_hardware` cannot be built in the reference workspace (it needs OpenArmCAN). Whether `gz_ros2_control` needs a
  `mimic` parameter for `openarm_*_finger_joint2` is open and is decided with a Gazebo run.
- The gripper controllers stay `position_controllers/GripperActionController` (deprecated in Jazzy) on purpose: the replacement,
  `parallel_gripper_action_controller`, serves `ParallelGripperCommand` instead of `GripperCommand`, so switching decides the
  gripper interface (manipulation#11, ADR-009 point 4), which is open.
- Issue #7: the description strings it names are fixed here; whether that closes it is for the code owners to judge.
