"""MoveIt on the combined OpenAMRobot model: the stack (mock or Gazebo), then move_group, optionally RViz's MotionPlanning display.

    ros2 launch openamrobot_upperbody_bringup moveit.launch.py [mast_id:=mast_1350] [sim:=mock|gazebo] [rviz:=false]
                                                              [headless:=false] [physics_engine:=<gz-physics plugin>]

  sim=mock     mock.launch.py (arms on mock_components/GenericSystem, five controllers active), a stub that reports the wheel joints at
               rest (the base is not under ros2_control), then move_group. Nothing else of the base runs.
  sim=gazebo   openamrobot_upperbody_gazebo's sim.launch.py (Gazebo, the base on its DiffDrive plugin, the arms on gz_ros2_control, the
               five controllers), then move_group on the simulator's clock.

Args
  mast_id         mast_1300 | mast_1350 | mast_1400 | mast_1450   (default mast_1350, the release configuration)
  sim             mock | gazebo
  rviz            true | false  RViz2 with the MotionPlanning display (config/moveit.rviz of the moveit package). With sim:=gazebo the
                  simulator's own RViz is not started: this one replaces it.
  headless        sim:=gazebo only. true runs `gz sim -s` (no Gazebo window), as the checks do
  physics_engine  sim:=gazebo only. The gz-physics plugin, passed to sim.launch.py. EMPTY by default, which means sim.launch.py's own
                  default: DART, the engine platform-sw tuned the base on. DART does not close the grippers (its fingers do not follow
                  position commands at the default gain, and it has no mimic constraint), so the arm named poses work there and the
                  gripper ones may not. Pass gz-physics-bullet-featherstone-plugin for gripper poses to close: M02 does, for that reason.

Everything here is simulation: it permits no statement about a physical robot or a limit of one (ADR-009).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def include(package, launch_file, **launch_arguments):
    path = get_package_share_directory(package) + "/launch/" + launch_file
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=launch_arguments.items())


def launch_setup(context, *args, **kwargs):
    mast_id = LaunchConfiguration("mast_id").perform(context)
    sim = LaunchConfiguration("sim").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)
    if sim not in ("mock", "gazebo"):
        raise RuntimeError(f"sim:={sim}: accepted values are mock, gazebo")
    if sim == "mock":
        # The base is not under ros2_control, so on mock nothing reports its wheel joints and MoveIt warns every second that the
        # robot's complete state is unknown. The stub reports them at rest (the base does not move during manipulation, AD-30).
        stack = [include("openamrobot_upperbody_bringup", "mock.launch.py", mast_id=mast_id, rviz="false"),
                 Node(package="openamrobot_upperbody_bringup", executable="base_joint_states_stub.py", output="log")]
    else:
        sim_args = {"mast_id": mast_id, "headless": LaunchConfiguration("headless").perform(context), "rviz": "false"}
        engine = LaunchConfiguration("physics_engine").perform(context)
        if engine:
            sim_args["physics_engine"] = engine
        stack = [include("openamrobot_upperbody_gazebo", "sim.launch.py", **sim_args)]
    move_group = include("openamrobot_upperbody_moveit", "move_group.launch.py", mast_id=mast_id, hardware_profile=sim, rviz=rviz)
    return [*stack, move_group]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mast_id", default_value="mast_1350", description="mast_1300 | mast_1350 | mast_1400 | mast_1450"),
        DeclareLaunchArgument("sim", default_value="mock", description="mock | gazebo"),
        DeclareLaunchArgument("rviz", default_value="false", description="true | false. RViz2 with the MotionPlanning display"),
        DeclareLaunchArgument("headless", default_value="false", description="sim:=gazebo only: true runs gz sim -s (no window)"),
        DeclareLaunchArgument("physics_engine", default_value="",
                              description="sim:=gazebo only: gz-physics plugin; empty = sim.launch.py's default (DART)"),
        OpaqueFunction(function=launch_setup),
    ])
