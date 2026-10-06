"""The one place the combined robot's MoveIt configuration is assembled, with moveit_configs_utils' MoveItConfigsBuilder.

The builder is handed the combined xacro with its args (`mast_id`, `hardware_profile`), not a pre-expanded URDF, so the robot_description
MoveIt sees is the one the rest of the stack expands from the same file. The SRDF, kinematics, limits and controller mapping are this
package's own files; none is read from upstream's moveit config package (ADR-007).
"""
import os

from ament_index_python.packages import get_package_share_directory

PACKAGE = "openamrobot_upperbody_moveit"
DESCRIPTION = "openamrobot_upperbody_description"
ROBOT_NAME = "openamrobot"          # the <robot name> of combined_robot.xacro and of the SRDF
MAST_IDS = ("mast_1300", "mast_1350", "mast_1400", "mast_1450")
PROFILES = ("mock", "gazebo")       # `real` is never launched in A7


def combined_xacro():
    return os.path.join(get_package_share_directory(DESCRIPTION), "urdf", "combined_robot.xacro")


def moveit_config(mast_id="mast_1350", hardware_profile="mock", srdf_file=None):
    """MoveItConfigs for the combined robot at `mast_id`. `hardware_profile` only selects the ros2_control block of the URDF, which MoveIt
    does not read; it is passed so that this URDF is the same text the controller manager and robot_state_publisher of that profile use.
    `srdf_file` replaces config/openamrobot.srdf (an absolute path); the mutation tests of M03 use it, nothing else does."""
    from moveit_configs_utils import MoveItConfigsBuilder
    if mast_id not in MAST_IDS:
        raise ValueError(f"mast_id {mast_id!r}: accepted values are {', '.join(MAST_IDS)}")
    if hardware_profile not in PROFILES:
        raise ValueError(f"hardware_profile {hardware_profile!r}: accepted values are {', '.join(PROFILES)}")
    return (MoveItConfigsBuilder(ROBOT_NAME, package_name=PACKAGE)
            .robot_description(file_path=combined_xacro(), mappings={"mast_id": mast_id, "hardware_profile": hardware_profile})
            .robot_description_semantic(file_path=srdf_file or "config/openamrobot.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .trajectory_execution(file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False)
            .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
            .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
            # move_group does NOT publish robot_description: robot_state_publisher is the one publisher of that topic. Two publishers race,
            # and ros_gz_sim's `create -topic robot_description` took move_group's copy, which has no controllers_file, so the model's
            # gz_ros2_control plugin found "an empty parameters file" and the simulator's controller manager never started (found in M02).
            .planning_scene_monitor(publish_robot_description=False, publish_robot_description_semantic=True)
            .to_moveit_configs())


def srdf_path():
    return os.path.join(get_package_share_directory(PACKAGE), "config", "openamrobot.srdf")
