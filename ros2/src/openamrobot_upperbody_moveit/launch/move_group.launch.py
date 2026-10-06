"""move_group (and, optionally, RViz's MotionPlanning display) for the combined OpenAMRobot model. A building block: it assumes the robot
is already running (robot_state_publisher, the controller manager, joint states) and starts nothing of that. The user-facing launch is
openamrobot_upperbody_bringup's moveit.launch.py, which starts the stack too.

    ros2 launch openamrobot_upperbody_moveit move_group.launch.py [mast_id:=mast_1350] [hardware_profile:=mock|gazebo] [rviz:=false]

Args
  mast_id           mast_1300 | mast_1350 | mast_1400 | mast_1450   (default mast_1350, the release configuration)
  hardware_profile  mock | gazebo    which stack is running; `gazebo` also sets use_sim_time (the simulator's /clock)
  rviz              true | false     RViz2 with the MotionPlanning display (config/moveit.rviz)

The configuration comes from MoveItConfigsBuilder over the combined xacro and this package's own files (moveit_config.py). Everything is
simulation: nothing here claims a physical limit (ADR-009).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

# Execution monitoring in MoveIt's TrajectoryExecutionManager: the duration allowed for a trajectory is its own duration scaled and padded.
# Simulated time runs at a real-time factor below 1 here, so with use_sim_time everything is on the same (simulated) clock.
TRAJECTORY_EXECUTION = {
    "trajectory_execution.allowed_execution_duration_scaling": 1.2,
    "trajectory_execution.allowed_goal_duration_margin": 0.5,
    "trajectory_execution.allowed_start_tolerance": 0.01,
}


def launch_setup(context, *args, **kwargs):
    from openamrobot_upperbody_moveit.moveit_config import moveit_config

    mast_id = LaunchConfiguration("mast_id").perform(context)
    profile = LaunchConfiguration("hardware_profile").perform(context)
    use_sim_time = profile == "gazebo"
    config = moveit_config(mast_id, profile)
    parameters = [config.to_dict(), TRAJECTORY_EXECUTION, {"use_sim_time": use_sim_time, "publish_robot_description_semantic": True}]

    nodes = [Node(package="moveit_ros_move_group", executable="move_group", output="screen", parameters=parameters)]
    if LaunchConfiguration("rviz").perform(context) == "true":
        rviz_config = get_package_share_directory("openamrobot_upperbody_moveit") + "/config/moveit.rviz"
        nodes.append(Node(package="rviz2", executable="rviz2", output="log", arguments=["-d", rviz_config],
                          parameters=[config.robot_description, config.robot_description_semantic,
                                      config.robot_description_kinematics, config.planning_pipelines, config.joint_limits,
                                      {"use_sim_time": use_sim_time}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mast_id", default_value="mast_1350", description="mast_1300 | mast_1350 | mast_1400 | mast_1450"),
        DeclareLaunchArgument("hardware_profile", default_value="mock", description="mock | gazebo: which stack is running"),
        DeclareLaunchArgument("rviz", default_value="false", description="true | false. RViz2 with the MotionPlanning display"),
        OpaqueFunction(function=launch_setup),
    ])
