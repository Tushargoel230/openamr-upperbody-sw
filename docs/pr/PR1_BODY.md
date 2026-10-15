## Why A6 and A7 are one PR

The change from the fixed mast to the lift column straddles the description and the simulation/MoveIt packages: the description gives up the mast, the control, Gazebo and MoveIt packages follow the lift joint, and the numbers each of them records (collision matrix, planning poses, configuration IDs) change together. On current `main` the description is only canonical if it carries the lift, and the lift work lives in the same commits as the Gazebo and MoveIt work, so the two cannot be split by commit; a cherry-pick of the A6 part on its own does not build green against `main`. A file-level split (description files in one PR, everything else in a second) is possible on request, at the cost of a first PR whose tests need the second.

`feat/a6-canonical-description` @ `44d28ea` stays on the fork as the historical A6 delivery (the mast model); this PR supersedes it.

## What is in it

Five packages under `ros2/src/`, plus `docs/`, `evidence/` and the changelog:

- `openamrobot_upperbody_description` — the canonical combined description: `combined_robot.xacro` composes the platform base (unchanged, `base_footprint` root), option B's upper body on its lift column and the upstream OpenArm macros; datums with provenance, frame contract, per-expansion evidence checks (T01-T16, C01/C02, E01, W01/W02, V01, H01/H02) and the harness that runs and reads them.
- `openamrobot_upperbody_control` — `ros2_control` for the two OpenArm v2.0 arms, one file for the mock, gazebo and real profiles (joint state broadcaster, trajectory and gripper controllers); the lift is not in it, we have no authority to command it.
- `openamrobot_upperbody_gazebo` — Gazebo Harmonic simulation of the combined model: the base on its DiffDrive plugin, both arms on `gz_ros2_control`, `odom -> base_footprint`.
- `openamrobot_upperbody_moveit` — MoveIt 2 configuration: arm planning groups with the lift as a planned joint, a collision matrix derived (83 disabled pairs, 478 enabled) rather than sampled, named poses, regression checks M01-M06, `joint_state_clamp.py`.
- `openamrobot_upperbody_bringup` — launch files composing base, lift column and arms in simulation.

## Evidence

Class: `static+mock` for the A6 suite, `simulation` for A7. **No physical evidence.** Every check reports one of five values:

| value | meaning |
|---|---|
| `PASS` | the check ran and its assertion held |
| `WARN` | the check ran and recorded a finding; it is not a pass and every WARN has a check ID |
| `FAIL` | an assertion was violated, or a process guard saw a process fail to start, die or exit badly |
| `SKIP` | a stated, reasoned non-comparison (the one legitimate case: no git checkout to compare an upstream pin with) |
| `NOT_RUN` | a registered check left no result; never a `PASS`, and it fails the exit status |

- **A7 regression** (`run_a7_regression.sh`): **13 PASS / 1 WARN (V01) / 0 FAIL / 0 NOT_RUN**, five consecutive runs on `7daa2ab` (the shutdown-race fix, before the rebase onto `main` and before option B was adopted: these prove the fix) plus three runs on `dada509`, and three more on `aa94fc3` (its code is that of `63ed6d1`, the last commit that changed code). One further run, on `9bc327c`, **failed** (11 PASS / 1 WARN / 2 FAIL) under an orphaned simulator of G01's launch that nothing detected; its report is kept (see `evidence/reports/README.md`, P16).
- **A6 suite** (`verify_a6.sh`): **24 PASS / 4 WARN (T08, E01, W02, V01) / 0 FAIL / 0 NOT_RUN** on `1168f45`, and on `aa94fc3` (three runs, the same numbers; code of `63ed6d1`). The first run of that step failed H01 (reports kept; fixed in `1168f45`).
- The reports are in `evidence/reports/`, with `README.md` listing commit and result for each, including the failed ones. `evidence/frozen_a6_993b000/` is the frozen expansion set of the earlier mast-model A6, with a verifier.
- Two of the eight A7 runs of P15 left a `gz sim` server running after the launch under test was torn down, and one more did after G01 in P16. G02, G01 and M02 now fail when a simulator of their own partition survives their launch; that check is proven through the launch-test path with an injected fault and was **not** observed to catch a natural orphan (0 of 12 live G02 runs produced one).

## Dependencies

- **platform-sw `935efc7` (#55)**, on platform-sw `main` (`174731c` at the time of writing): `base_footprint` root, `base_link` 0.098 m above it, and the near-field sensor frames `us_*` (4 x MB7060) and `tof_*` (2 x TF-Luna) declared. This description requires it (its root and the 0.098 m are asserted against platform-sw's `base_footprint_joint`).
- The six near-field frames that earlier versions of our contract declared (`tof_front_left_link` ... `ultrasonic_rear_link`) are removed from our contract and are `consumed_from` platform-sw, not declared by us. T04 now reports which of them each expansion contains and passes **by deletion**: nothing in our contract can be absent any more.
- No upstream (`openarm_description`) file is modified; the pin is unchanged.

## Open questions for reviewers

- [ ] **Alex and Cuma: the two opt-in parameters on `openamrobot_upperbody.xacro`.** `emit_arm_mounts` (default `true`) and `column_inertial` (default `''`, none). Both default to current `main` behaviour. They exist because the upstream arm macro emits a link and a joint with the same names as option B's mount frames, so the combined robot has to turn those off, and because the combined robot supplies the lift's sourced catalogue mass. Two separate statements, kept apart because the first one is weaker than it sounds. (a) A test (`test_with_the_opt_in_parameters_at_their_defaults_the_macro_expands_as_it_does_on_main`) asserts that the mount frames and joints exist and that no inertial appears anywhere with the defaults; it does **not** compare against `main`. (b) Separately, compared byte for byte against `upstream/main` (`tools/compare_option_b_wrapper.sh`, run at `HEAD`), the standalone wrapper with the defaults expands identically apart from the empty `lift_column_base_link`, which is serialised as `<link name="lift_column_base_link"></link>` (an open/close pair) instead of a self-closing tag, and the path xacro writes in its header comment. That script is not part of the test suite. Needs acceptance, or a different composition.
- [ ] **Supplier drawing for the lift column.** The column has **no visual and no collision geometry**: it exists in the model as an inertia tensor only, so every render committed under `evidence/renders/` shows the arms with no column between them, and the arms can plan through the column body. It also carries one invented lateral dimension: `lift_column_section` y is set equal to the sourced 240 mm x side. Both are marked in `datums.yaml` (`placeholder`/`provisional`); nothing downstream treats them as measured.
- [ ] **V01's interference count was taken on collision geometry only.** The arms' visual meshes were not intersected separately, and the column has neither kind of geometry, so a zero there says nothing about the column.

## Known gaps

- `lift_mass_tl3_400` is `null` (no supplier mass for the fallback TiMOTION TL3 400 column). By design a null datum that is read is a hard-fail that names itself: the `tl3_400` configuration does not expand.
- V01 (manual visual check) is `WARN`. Its eight items were measured by script on the lift model (none reads outside its expected value; two sub-questions that need eyes are CANNOT-MEASURE; `evidence/v01/`, renders in `evidence/renders/`), and it stays `WARN` until `lift_mass_tl3_400` is sourced, however the eight land. No person has looked at the running simulator.
- M02: the joint-state clamp's margin to its epsilon (1e-5 rad) is about 3x, not the 10x it was set with: activations went from 184-277 to about 1750 per run and the largest overshoot from 9.9e-7 to 3.1e-6 rad, almost all `openarm_right_finger_joint1/2` at their open limit. Nothing is beyond the epsilon and the poses reach their goals. Bisected in P16 (`evidence/m02_bisect/RESULTS.md`): the ACM, the modelled mass, the arm separation and position, the open command and platform-sw's new base are each ruled out; the cause is not found. It is `right_finger_joint1` resting on its open limit in the new body (tens of microradians inside it in the old one). Counts scatter 646 to 1054 between runs of one unchanged configuration. The epsilon has not been widened.
- No runtime producer of the configuration identity has an owner: `config_id` is computed offline from files; nothing in a running system publishes it.
- Arms are 62 mm apart at their bases: no contact at home, but the left gripper reaches right-arm links in about 3% of random left-arm states (sampled).
- A `gz sim` server is orphaned after a launch test in some A7 runs (about 2 of 8 in P15, 1 of 8 in P16 plus one natural one in the first G02 run); the process guard does not see it, the new check does.

## `config_id` inputs changed

`config_id` kept its format and changed its inputs: the canonical URDF, `column`, `base_plate`, `head_pitch`, `base_tilt`, `arm_package`, `hardware_profile`, datums revision and every upstream pin (before: `mast_id` and no column or base-plate inputs). It still validates against the C9 schema, so nothing fails to say that **an id recorded before this change is not comparable with one computed now**. The frozen A6 set keeps the old inputs and says so. Satya (C9 owner): please confirm that consumers do not compare ids across this change.

## AI assistance

This PR was developed with Claude Code (Anthropic) against a written design (architecture documents and prompts). The work was run and tested by the repository's evidence harness (`verify_a6.sh`, `run_a7_regression.sh`; the reports are in `evidence/reports/`, produced on the author's workstation, not by a hosted CI service) and reviewed by Tushar Goel. Prompts and design documents are available on request.
Per the OpenAMRobot AI Contribution Policy.

## Checklist

- [ ] DCO sign-off on every commit
- [ ] Contributor agreement accepted
- [ ] No upstream files modified
- [ ] No frame names outside design/FRAMES.md
