# Changelog

All notable changes to this repository. Evidence for the description is static and mock only; nothing here is a hardware
or Gazebo result.

## [Unreleased]

### Added

- `openamrobot_upperbody_description`: the canonical OpenAMRobot 2.0 robot description (A6). One xacro entry point,
  `combined_robot.xacro`, composes the platform base (unchanged), a fixed mast selected by `mast_id` (`mast_1300`,
  `mast_1350` release, `mast_1400`, `mast_1450`) and two upstream OpenArm v2.0 arms rooted on mast plate links, with
  `arm_package` (`openarm`, `so101` stub, `none`), `hardware_profile` (validated, no effect yet) and `enable_planar_base`
  (a `planar` joint `world -> base_link`, present and disabled by default, axis `0 0 1`).
- Head camera and wrist camera frames as plain frames with a visual marker (the ZED wrapper is deferred).
- `config/datums.yaml` (14 datums with provenance: 9 placeholder, 4 provisional, 1 verified), its JSON schema, and
  `config/frame_contract.yaml`, the machine-readable frame and joint contract that the tests enforce.
- Verification suite and `verify_a6.sh`, which writes an `openamr.evidence.v1` report: expansion matrix, `check_urdf`,
  single root, frame contract, shoulder height per mast ID, argument validation, collision and inertia checks, mass report,
  planar joint, datum rules, configuration ids (`sha256:` plus 64 hex characters, matching the C9 dataset schema),
  installed-versus-source check, and warnings for the envelope (E01), KDL root inertia (W01), the planar joint under KDL
  (W02) and the missing visual inspection (V01). T14-T16 and L01 are SKIP: they need `ros2_control`, which is not
  implemented yet.
- `view.launch.py`: robot_state_publisher, joint_state_publisher_gui and RViz2 for any mast ID.
- READMEs for the description package and, stating their scaffold status, for the control, bringup, Gazebo and MoveIt
  packages.
- Test dependencies `python3-numpy` and `liburdfdom-tools` declared in the description package, so
  `rosdep install --from-paths src` installs what T02 and T05 use.

### Changed

- Repository README: status updated, and a pointer to the description package. The mast configuration text is unchanged.

### Removed

- `config/lift/`: an empty scaffold directory. An actuated lift is OpenAMRobot 3.0 scope.

### Known limitations

- Base inertials are platform-sw CAD placeholders; the mast mass, mast and plate transforms and all camera poses are
  placeholder datums. No physical evidence.
- Issue #7 stays open: the `package.xml` descriptions of the bringup, control and MoveIt packages still describe a lift.
