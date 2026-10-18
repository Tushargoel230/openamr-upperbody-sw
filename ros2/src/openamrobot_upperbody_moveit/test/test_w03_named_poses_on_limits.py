"""W03 (P18, WARN only, gating nothing): which named poses of the SRDF put a joint exactly on one of its URDF limits, and how far.

It replaced M07. M07 measured a simulated rest position against a margin nobody could source, and its premise (the new body put the finger on
a limit, the old body did not) was refuted by measurement (evidence/m02_bisect/RESULTS.md). The thing that is true is simpler and is a fact
about the pose definitions: upstream's `home` is joint 4 at its lower limit 0, and a gripper's `open` is the end of its range. So this check
lists them and says nothing about any body being wrong. It must list the same joints for the old mast body as for the lift body (it is a
function of the SRDF and URDF it is given, with no knowledge of either), which `test_w03_*_both_bodies` pins on a synthetic second body, and
the real old and new reports are in evidence/w03/.

Result: WARN when at least one is listed (it is never a FAIL), PASS when none is."""
from conftest import srdf_text, urdf_text, write_sidecar
from openamrobot_upperbody_moveit.named_poses import ON_LIMIT_TOLERANCE, on_limit_report, poses_on_limits, read_group_states, urdf_joint_limits

FOUR = ["openarm_left_finger_joint1", "openarm_left_joint4", "openarm_right_finger_joint1", "openarm_right_joint4"]


def test_w03_named_poses_on_a_joint_limit_are_listed_with_their_distance():
    report = on_limit_report(srdf_text(), urdf_text())
    rows = report["poses_on_a_limit"]
    write_sidecar("W03", "named_poses_on_limits", "named poses that coincide with a joint limit (information; gates nothing)",
                  "WARN" if rows else "PASS", report)
    assert report["joints_on_a_limit"] == FOUR, report["joints_on_a_limit"]
    assert all(abs(r["distance_rad"]) <= ON_LIMIT_TOLERANCE for r in rows)
    by = {(r["group"], r["pose"], r["joint"]): r for r in rows}
    assert by[("left_arm", "home", "openarm_left_joint4")]["limit"] == "lower"
    assert by[("right_gripper", "open", "openarm_right_finger_joint1")]["limit"] == "upper"
    assert by[("left_gripper", "open", "openarm_left_finger_joint1")]["limit"] == "lower"
    assert by[("left_gripper", "closed", "openarm_left_finger_joint1")]["limit"] == "upper"
    assert report["named_pose_joints_without_a_urdf_limit"] == []


SRDF_OTHER_BODY = """<robot name="x">
  <group_state name="home" group="a"><joint name="elbow" value="0"/><joint name="shoulder" value="0.5"/></group_state>
  <group_state name="open" group="g"><joint name="finger" value="0.04"/></group_state>
  <group_state name="odd" group="a"><joint name="mystery" value="1"/></group_state>
</robot>"""
URDF_OTHER_BODY = """<robot name="x">
  <joint name="elbow" type="revolute"><limit lower="0" upper="2.4"/></joint>
  <joint name="shoulder" type="revolute"><limit lower="-1" upper="1"/></joint>
  <joint name="finger" type="prismatic"><limit lower="0" upper="0.04"/></joint>
</robot>"""


def test_w03_lists_the_same_kind_of_thing_for_any_body_and_does_not_invent_or_skip():
    """A second, different body: a pose on a lower limit, one on an upper limit, one comfortably inside, and one named joint with no URDF limit,
    which is reported as such and not dropped."""
    report = on_limit_report(SRDF_OTHER_BODY, URDF_OTHER_BODY)
    assert report["joints_on_a_limit"] == ["elbow", "finger"]
    assert {r["joint"]: r["limit"] for r in report["poses_on_a_limit"]} == {"elbow": "lower", "finger": "upper"}
    assert [u["joint"] for u in report["named_pose_joints_without_a_urdf_limit"]] == ["mystery"]


def test_w03_a_value_past_a_limit_is_listed_with_a_negative_distance_and_one_just_inside_is_not():
    states = {("g", "past"): {"j": -0.0000005}, ("g", "near"): {"j": 0.0001}}
    rows, _ = poses_on_limits(states, {"j": (0.0, 1.0)})
    assert [(r["pose"], r["distance_rad"] < 0) for r in rows] == [("past", True)]
    # 1e-4 rad inside is not "on" the limit: the tolerance is numeric equality of two written numbers, not a margin
    assert ON_LIMIT_TOLERANCE <= 1e-6


def test_w03_it_is_not_a_property_of_the_body_the_real_srdf_and_a_second_urdf_agree():
    """The same SRDF against the real URDF and against a URDF whose joint4 limits differ: the report follows the limits, not the body."""
    states, limits = read_group_states(srdf_text()), dict(urdf_joint_limits(urdf_text()))
    moved = dict(limits, openarm_left_joint4=(-0.5, 2.4435))
    rows, _ = poses_on_limits(states, moved)
    assert "openarm_left_joint4" not in {r["joint"] for r in rows}
    assert "openarm_right_joint4" in {r["joint"] for r in rows}
