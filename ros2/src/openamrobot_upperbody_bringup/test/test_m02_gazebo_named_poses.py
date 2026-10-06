"""M02: every named pose of both arms (and both grippers) plans and executes in Gazebo at mast_1350, through move_group, with a final
joint error below 0.01 rad. mast_1350 is the release configuration under rev 18.2 (VERIFICATION.md, corrected 6 Oct).

The stack is `moveit.launch.py sim:=gazebo headless:=true`: Gazebo Harmonic (server only), the base on platform-sw's DiffDrive plugin, the
arms on gz_ros2_control, the five controllers, move_group on the simulator's clock. The poses are the same sequence as M01.

THE PHYSICS ENGINE: this test passes `physics_engine:=gz-physics-bullet-featherstone-plugin` explicitly. sim.launch.py defaults to DART (the
engine platform-sw tuned the base on; decided 6 Oct), and on DART the fingers do not follow the position command at the default gain and
`finger_joint2` has no mimic constraint, so a `closed` gripper pose would not close. The arm poses do not depend on the engine; the gripper
poses do, and they are part of what this check is for (the per-hand sign, live). That is the same reason G02 passes the engine. What was
and was not compared on DART: see the gazebo package's README.

`simulation` class: it says how the model behaves in this simulator on this machine, under the Gazebo position interface (a velocity
servo), and nothing about a physical robot or its limits. The acceleration limits MoveIt plans under are engineering defaults (ADR-009).
"""
import os
import random
import sys
import unittest

os.environ["ROS_DOMAIN_ID"] = str(random.randint(101, 200))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["GZ_PARTITION"] = f"m02_{os.getpid()}"

import launch  # noqa: E402
import launch_testing.actions  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402

from openamrobot_upperbody_moveit.move_group_client import MoveGroupClient  # noqa: E402
from openamrobot_upperbody_moveit.named_poses import GRIPPER_JOINT, SIDES  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a7_support import (ACCELERATION_SCALING, TOLERANCE, VELOCITY_SCALING, named_pose_plan, run_sequence,  # noqa: E402
                        write_run_sidecar)

MAST_ID = "mast_1350"
# The engine this check REQUIRES, passed explicitly on purpose (see the docstring); the launch's own default is DART.
ENGINE_THE_GRIPPER_POSES_NEED = "gz-physics-bullet-featherstone-plugin"
READY_WITHIN_S = 150.0


@pytest.mark.launch_test
def generate_test_description():
    moveit = os.path.join(get_package_share_directory("openamrobot_upperbody_bringup"), "launch", "moveit.launch.py")
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit),
            launch_arguments={"sim": "gazebo", "mast_id": MAST_ID, "rviz": "false", "headless": "true",
                              "physics_engine": ENGINE_THE_GRIPPER_POSES_NEED}.items()),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestNamedPosesInGazebo(unittest.TestCase):
    record = {}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("m02_test")
        cls.client = MoveGroupClient(cls.node)

    @classmethod
    def tearDownClass(cls):
        rows = cls.record.get("poses", [])
        ok = bool(rows) and all(r["ok"] for r in rows)
        details = {"mast_id": MAST_ID, "hardware": "gz_ros2_control/GazeboSimSystem (Gazebo Harmonic, headless)",
                   "physics_engine_used": ENGINE_THE_GRIPPER_POSES_NEED, "physics_engine_passed_explicitly": True,
                   "why_this_engine": "the launch defaults to DART, on which the gripper poses do not close (fingers do not follow the "
                                      "position command at the default gain; no mimic constraint); the arm poses do not depend on it",
                   "tolerance_rad": TOLERANCE, "velocity_scaling": VELOCITY_SCALING, "acceleration_scaling": ACCELERATION_SCALING,
                   "gripper_interface": "GripperCommand via moveit_simple_controller_manager (ADR-009 option 1)",
                   "poses_run": len(rows), "poses_ok": sum(1 for r in rows if r["ok"]),
                   "start_state_invalid_retries_total": sum(r["start_state_invalid_retries"] for r in rows),
                   "start_state_invalid_retries_note": "MoveIt 2.12.4 refuses a start state even 1e-7 rad outside a joint limit "
                   "(START_STATE_INVALID, no tolerance); a simulated joint resting on its limit (joint4 = 0, the home pose) can read past "
                   "it. Each retry is a re-request half a second later; at most 3 per pose are allowed",
                   "worst_final_error_rad": max((r["final_max_joint_error_rad"] for r in rows), default=None),
                   "poses": rows, **{k: v for k, v in cls.record.items() if k != "poses"}}
        write_run_sidecar("M02", f"gazebo_{MAST_ID}", f"named poses plan and execute in Gazebo at {MAST_ID}",
                          "PASS" if ok else "FAIL", details)
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_m02_1_every_named_pose_plans_and_executes_with_a_final_error_under_tolerance(self):
        self.assertTrue(self.client.wait_for_server(READY_WITHIN_S), f"move_group's /move_action did not appear within {READY_WITHIN_S:.0f} s")
        joints = [f"openarm_{side}_joint{i}" for side in SIDES for i in range(1, 8)] + [GRIPPER_JOINT[s] for s in SIDES]
        self.assertTrue(self.client.wait_for_joints(joints, 120.0), "/joint_states never listed all 16 controlled joints")
        self.record["poses"], failures, self.record["start"] = run_sequence(self.client, named_pose_plan(), result_timeout=240.0)
        self.assertFalse(failures, "\n".join(failures))
        self.assertEqual(len(self.record["poses"]), 4 * 2 + 2 * 2)
