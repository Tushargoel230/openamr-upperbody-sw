# Reports cited by the PR descriptions

(P14's four reports first, then P15's: the five runs that prove the shutdown fix, the A6 runs of the rebased tree, and three A7 runs of the final code.)

Machine-written, copied byte for byte from the author's workspace `evidence/`; they contain local checkout paths. Each describes the commit in its
`git_hash` field, on a clean tree (`dirty: false`). `WARN` is not `PASS`.

| file | what | commit | result |
|---|---|---|---|
| `20261006T164342Z_dev.json` | A6 suite (`verify_a6.sh`), the report PR-1 cites | `993b000` (A6 head) | 20 PASS, 6 WARN, 0 FAIL |
| `20261012T164313Z_dev.json` | A6 suite on the A7 branch (adds H01, H02; V01 now carries the manual-look findings) | `93a2a49` | 22 PASS, 6 WARN, 0 FAIL, 0 NOT_RUN |
| `20261012T170225Z_a7.json` | A7 regression (`run_a7_regression.sh`), the report PR-2 cites | `93a2a49` | 13 PASS, 1 WARN (V01), 0 FAIL, 0 NOT_RUN |
| `20261012T165237Z_a7.json` | the first A7 run on the same commit: **FAILED** M01 (and AUX) through the process guard, `joint_state_clamp.py` exited 1 at shutdown in one of four launches; all poses passed. Kept beside the passing one | `93a2a49` | 11 PASS, 1 WARN, 2 FAIL |

The two A7 reports ran on one commit with nothing changed between them: one of two runs failed, and both are here.

## P15

**Task 0, five consecutive A7 runs on one commit** (`7daa2ab`: the `joint_state_clamp.py` fix, still on the old model and the old base, before the
rebase). Every run is 13 PASS, 1 WARN (V01), 0 FAIL, 0 NOT_RUN; all 20 M01 launches (four mast IDs x five runs) passed 12 of 12 poses and every
process exited as the guard expects.

| file | run |
|---|---|
| `20261013T093322Z_a7.json` | 1 |
| `20261013T094251Z_a7.json` | 2 |
| `20261013T095220Z_a7.json` | 3 |
| `20261013T100148Z_a7.json` | 4 |
| `20261013T101116Z_a7.json` | 5 |

A `gz sim` server was left running after run 4 (started during G02's launch, re-parented away from it) and stopped by hand by pid; no test saw it.

**The rebased tree with option B adopted** (platform-sw `174731c`):

| file | what | commit | result |
|---|---|---|---|
| `20261013T101652Z_dev.json` | A6 suite, the **first** run: **FAILED** H01 (and AUX with it): option B registers `test_upperbody_tf.py` with `ament_add_ros_isolated_pytest_test`, which H01's registration expression did not know, so a test that runs was reported as one that never runs. Fixed in `1168f45` | `0f1ca1e` | 22 PASS, 4 WARN, 2 FAIL |
| `20261013T102027Z_dev.json` | A6 suite after the fix | `1168f45` | 24 PASS, 4 WARN (T08, E01, W02, V01), 0 FAIL, 0 NOT_RUN |
| `20261013T102944Z_a7.json` | A7 regression, final code, run 1 | `dada509` | 13 PASS, 1 WARN (V01), 0 FAIL, 0 NOT_RUN |
| `20261013T103837Z_a7.json` | the same, run 2 | `dada509` | 13 PASS, 1 WARN, 0 FAIL, 0 NOT_RUN |
| `20261013T104713Z_a7.json` | the same, run 3 | `dada509` | 13 PASS, 1 WARN, 0 FAIL, 0 NOT_RUN |

`dada509` is the last commit that changed code, so the three A7 reports describe the code of the final branch; every later commit here is
documentation or an evidence copy. A `gz sim` server was left running after run 3 (again from G02's launch) and stopped by pid: **two of the eight
A7 runs of this step left a simulator behind**, the P12A finding that the process guard does not see (the simulator is the `ruby` wrapper's child, and
`sim.launch.py` asks the server to stop through `/server_control` on shutdown, which evidently does not always work).

**A number that moved on purpose and has no cause established.** In M02 the clamp's activations went from 184 to 277 per run (P14 and the five runs above,
16 to 22 episodes) to 1707 to 1800 (61 to 88 episodes) on the final code, and the largest overshoot it clamped from 5.4e-07 to 9.3e-07 rad to 1.95e-06
and 3.1e-06 rad (`openarm_left_joint4`). Reproduced in the development overlay: 1725, 1796, 1796 and one run of 397. Nearly all of the activations are
`openarm_right_finger_joint1` and `_joint2` at their open limit reading 2.4e-07 rad past it (829 to 865 messages each), which is float noise of the
same kind the clamp exists for; nothing is beyond the epsilon (1e-05), and the poses still reach their goals (12 of 12, 0.001 rad). The hypothesis that
the lift's own prismatic joint, now an unactuated joint in the physics, is the cause was **tested and rejected**: with `lift_joint` made `fixed` in a
copy the activations are 3300 to 3500 (the left fingers join in). So it is the physics solution of the new model arrangement and the new base, and
it is not understood. **The consequence that matters:** the epsilon was set at ten times the largest overshoot seen (9.9e-07); the largest seen is now 3.1e-06,
a margin of about 3. Nobody should widen the epsilon on this (it is ADR-009's number and the clamp must not launder a real excursion): it is a finding.

Class: `static+mock` for `_dev`, `simulation` for `_a7`. No physical evidence. The frozen expansion set is in `../frozen_a6_993b000/`.
