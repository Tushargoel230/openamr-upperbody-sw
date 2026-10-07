"""Shared by the M01 and M02 launch tests: the named-pose sequence, the settle-and-measure step, and the per-run sidecar."""
import os
import sys

from openamrobot_upperbody_moveit.joint_state_clamp import EPSILON
from openamrobot_upperbody_moveit.moveit_config import srdf_path
from openamrobot_upperbody_moveit.named_poses import ARM_GROUP, GRIPPER_GROUP, GRIPPER_JOINT, SIDES, read_group_states

# M01 and M02 start whole stacks (M02 a Gazebo server), and `verify_a6.sh` runs this package's whole `colcon test`: so they run only when
# OPENAMR_A7=1, which run_a7_regression.sh sets. Otherwise they SKIP with this reason, and the A6 verification neither starts Gazebo nor
# depends on an A7 check. A skipped M01/M02 is never a result: the A7 report treats a missing sidecar as a FAIL.
A7_ENABLED = os.environ.get("OPENAMR_A7") == "1"
SKIP_REASON = "A7 simulation check: set OPENAMR_A7=1 (scripts/run_a7_regression.sh does) so verify_a6.sh does not start it"
TOLERANCE = 0.01            # rad: the final joint error every named pose must stay under (A7)
VELOCITY_SCALING = 0.5      # explicit in every request: the SRDF-side default (0.1) would only make the tests slower
ACCELERATION_SCALING = 0.5
SETTLE_SECONDS = 1.0        # of the node's clock after a pose is reported done, before the final joint state is read
# Arms: each pose is reached from a DIFFERENT pose (home -> hands_up -> home -> hands_up -> park), so no plan is a trivial
# start-equals-goal one. `park` is a placeholder copy of `home`; it is still a named pose that must plan and execute.
ARM_SEQUENCE = ("hands_up", "home", "hands_up", "park")
# Grippers: closed then open, per hand, each with that hand's own value from the SRDF (ADR-009 point 5).
GRIPPER_SEQUENCE = ("closed", "open")


def description_scripts():
    from ament_index_python.packages import get_package_prefix
    path = os.path.join(get_package_prefix("openamrobot_upperbody_description"), "lib", "openamrobot_upperbody_description")
    if path not in sys.path:
        sys.path.insert(0, path)
    return path


def write_run_sidecar(check_id, slug, name, result, details):
    description_scripts()
    import evidence_common as ev
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("openamrobot_upperbody_description")
    return ev.write_sidecar(check_id, slug, name, result, details, directory=ev.evidence_dir(share))


def named_pose_plan():
    """[(group, pose name, {joint: value})] in the order they are run: both arms, then both grippers."""
    states = read_group_states(open(srdf_path()).read())
    plan = []
    for side in SIDES:
        plan += [(ARM_GROUP[side], pose, states[(ARM_GROUP[side], pose)]) for pose in ARM_SEQUENCE]
    for side in SIDES:
        plan += [(GRIPPER_GROUP[side], pose, states[(GRIPPER_GROUP[side], pose)]) for pose in GRIPPER_SEQUENCE]
    return plan


def controlled_limits():
    """{joint: (lower, upper)} of the sixteen controlled joints, from the same expansion MoveIt's robot_description comes from."""
    import xacro
    from ament_index_python.packages import get_package_share_directory
    from openamrobot_upperbody_moveit.named_poses import GRIPPER_JOINT, urdf_joint_limits
    path = os.path.join(get_package_share_directory("openamrobot_upperbody_description"), "urdf", "combined_robot.xacro")
    limits = urdf_joint_limits(xacro.process_file(path, mappings={"hardware_profile": "mock"}).toxml())
    wanted = [f"openarm_{side}_joint{i}" for side in SIDES for i in range(1, 8)] + [GRIPPER_JOINT[s] for s in SIDES]
    return {j: limits[j] for j in wanted}


def run_sequence(client, plan, result_timeout=120.0, start_wait=60.0):
    """Plan and execute every pose through move_group; return (rows, failures, start). A row is one pose: the outcome, the largest joint
    error after settling, and whether it is under TOLERANCE. `start` records the wait for a valid start state (see
    MoveGroupClient.wait_inside_limits) and what it saw, so the start-up transient is on the record."""
    inside, transient = client.wait_inside_limits(controlled_limits(), start_wait, tolerance=EPSILON)
    start = {"start_state_inside_limits": inside, "out_of_limit_readings_seen_while_waiting":
             [{"joint": j, "value": v, "lower": lo, "upper": hi} for j, v, lo, hi in transient]}
    rows, failures = [], []
    if not inside:
        failures.append(f"the joint states never stayed inside the URDF limits for 1 s within {start_wait:.0f} s: {start}")
        return rows, failures, start
    for group, pose, goal in plan:
        outcome = client.move(group, goal, execute=True, velocity_scaling=VELOCITY_SCALING,
                              acceleration_scaling=ACCELERATION_SCALING, result_timeout=result_timeout)
        client.settle(SETTLE_SECONDS)
        error = client.max_error(goal) if all(j in client.joint_states for j in goal) else float("nan")
        ok = outcome.success and error < TOLERANCE
        rows.append({"group": group, "pose": pose, "goal": {j: round(v, 4) for j, v in goal.items()}, **outcome.as_dict(),
                     "final_max_joint_error_rad": round(error, 5), "under_tolerance": bool(error < TOLERANCE), "ok": ok})
        if not ok:
            failures.append(f"{group}/{pose}: {outcome.name}, final error {error:.4f} rad (limit {TOLERANCE})")
    return rows, failures, start


def clamp_record(client):
    """What the joint-state clamp did during this run, for the sidecar: its counters (activations, episodes, the largest magnitude, readings
    beyond the epsilon), or the statement that no clamp ran. The counters are the point of the clamp: one that fires constantly, or by more
    than its epsilon, says something other than float noise is moving a joint."""
    stats = client.clamp_stats()
    if stats is None:
        return {"present": False, "note": "no /joint_state_clamp/get_stats: move_group planned from the raw /joint_states"}
    return {"present": True, **stats,
            "note": "joint_state_clamp.py: a reading past a joint limit by at most epsilon_rad is published at the limit, to move_group only; "
                    "a reading further out is passed through and counted as over_epsilon. No limit was widened and no pose moved"}


def stack_healthy(proc_info, expected, simulator=False):
    """After the launch: every process in `expected` started and every process exited cleanly, allowing move_group's known SIGSEGV at stop and,
    with the simulator, its documented shutdown exits (launch_guard.py has the list and the reasons)."""
    description_scripts()
    import launch_guard
    launch_guard.assert_launch_healthy(proc_info, expected=expected, allowed=launch_guard.SIM_ALLOWED if simulator else launch_guard.MOVE_GROUP_ALLOWED)
