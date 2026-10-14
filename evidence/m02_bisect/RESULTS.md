# M02 clamp activations: bisect of the 184-277 -> ~1750 rise (P16 task 3)

**Result: no cause found.** Four candidates and the platform-sw base are ruled out by measurement; the rise is somewhere in the option-B body
composition as a whole and was not bisected below that. The epsilon (1e-5) was not touched.

## What the rise is

Per M02 run (50 Hz `/joint_states`, about 1520 messages, about 30 s of simulated time), per joint, from the clamp's own counters
(`details.clamp.joints` in the run sidecar; the A7 report keeps only the totals):

| tree | activations | where |
|---|---|---|
| old body (mast model, `7daa2ab`) on old platform-sw (`12de3ef`) | 212 | `left_joint4` 44, `right_joint4` 168. **No finger joint ever.** |
| old body on **new** platform-sw (`174731c`) | 188, 216 | the same two joints. **No finger joint ever** |
| new body (`fe5107f`) on new platform-sw | 1796, 1795, 1350 | `right_finger_joint1` and `_joint2` 873, 873, 646 each (the same reading: joint2 is `joint_state_publisher`'s mimic of joint1), `left_finger_*` 8 each at 1.2e-17 rad (rounding), joint4s 8 to 35 |

So it is one physical joint, the **right** finger at its open limit (0; range [-0.7854, 0]), reading 1e-8 to 2.9e-7 rad past it, for about half of
the run; nothing else changed class. The left finger (range [0, 0.7854], same limit value) does not.

## Which candidates are ruled out, and by what

Each variant is the new tree with one change made in a copy of the installed share directory (`scripts/mkovl.py`), two M02 runs each
(the contaminated first attempt is kept beside, see below). The scatter inside one configuration is as large as the whole range between
variants (unchanged tree: 873, 646, 1054 right-finger activations), so only "present or absent" can be read from this table.

| candidate | variant | right finger activations (run 1, run 2) | verdict |
|---|---|---|---|
| (a) SRDF / ACM rebuild (83 pairs, 478 enabled) | **every** link pair disabled (1596 pairs): the ACM cannot be a cause by any value | 162, 882 | **ruled out** (present at both runs) |
| (b) modelled mass 21.96 -> 30.66 kg | `lift_mass_dold_v4_350` 11.7 -> 3.0 kg (modelled mass back to about 21.96) | run 1 invalid (`move_group` SIGSEGV at start, no poses; the idle readings in it: 5654 of 5756 messages), 908 | **ruled out** (present, 98% of idle messages and in the valid run) |
| (c) two plates / 62 mm separation | arm bases 120 mm apart (old mast value); and arm bases at the column axis (x 0.18 -> 0) | 454, 889 ; 1062, 806 | **ruled out** (both present) |
| (d) opposite-signed GripperCommand open positions (ADR-009 point 5) | `open` at +0.001 (left) and -0.001 (right) instead of 0 | 887, 861 | **ruled out as a cause**; also not a change between old and new (the SRDF gripper states and `arm_controllers.yaml` are byte-identical between `7daa2ab` and HEAD) |
| (e) *not on the prompt's list:* platform-sw 935efc7 (wheel radius 0.11 -> 0.100, axle 0.098 above the floor, `base_footprint` root) | old body composed on the new base | 188, 216 (no finger activations) | **ruled out**: the new base does not produce it |
| lift joint (P15) | `lift_joint` made fixed | 3300 to 3500 (P15, in `evidence/reports/README.md`) | ruled out (made worse) |

The old body on the new base is the discriminating run: the rise needs the new body and does not need the new base.

## What was learned about the mechanism (a measurement, not a cause)

`scripts/probe.sh` launches the simulator alone, waits 25 s of simulated time and samples `/joint_states` at rest (600 messages):

| tree | `left_finger_joint1` | `right_finger_joint1` |
|---|---|---|
| old body, old base (`old1`) | +3.6e-5 (inside its range) | -6.0e-5 (inside its range) |
| new body (`head1`) | +2.0e-4 | -4.7e-6 |
| new body (`head2`, same configuration) | +6.4e-4 | +2.7e-8 (**past** the limit) |

In the old body both fingers rest tens of microradians inside their range; in the new body the **right finger rests on its limit** (within
1e-5 rad of it, and in some runs past it), and the left one rests further inside than before. A servo holding a joint at a target that is the
limit, against a static torque that presses it onto the limit, leaves the limit constraint carrying the load, and the soft constraint's
penetration is the 1e-8 to 3e-7 rad that is read. The reading is therefore a resting condition, not a motion effect (it is there with
no poses at all), and it hovers across zero, which is why the count per run scatters from 160 to 1060. **That the torque's sign changed
between the two bodies is a hypothesis consistent with these numbers; it was not measured** (no finger torque or contact force was read),
and nothing here says why the new composition changes it.

## What is left

The candidates that remain are whatever else the option-B composition changed for the arms' static load, none of which was isolated:
the prismatic joint with `effort 0` and the column's 11.7 kg inertial between the base and the arms (the fixed-lift test of P15 and the
3 kg test above say neither alone is it), the carriage and `arm_profile_link` bodies, the head and wrist camera links, and the way the
URDF-to-SDF conversion lumps the fixed joints and orders the multibody (the arms hang off a different parent chain than they did).
The next experiment is a finger-torque or contact-force readout at rest (`gz topic` on the joint wrench or contacts) in both trees.

## Hygiene note on this data

The first overlay attempt (`m02_results_first_attempt_contaminated.txt`) is **not evidence**: the overlay lacked the package's `lib`
directory (the sidecars were never written) and the failed runs left `gz sim` servers and stack processes running, which loaded
the machine for the runs after them. Those runs were discarded; the table above is from runs after a clean-up, with a pre-run check
that nothing of an earlier run is alive. The M02 launch (not only G02's) leaves its simulator behind after a failed run.
