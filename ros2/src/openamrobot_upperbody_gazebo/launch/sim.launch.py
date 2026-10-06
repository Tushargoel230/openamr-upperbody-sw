"""The combined OpenAMRobot 2.0 robot in Gazebo Harmonic: the base on its DiffDrive plugin, both arms on gz_ros2_control.

    ros2 launch openamrobot_upperbody_gazebo sim.launch.py [mast_id:=mast_1350] [headless:=true] [world:=/path/x.sdf]
                                                           [rviz:=true]

What it starts, in this order of dependence
  gz sim                 -r (run), plus -s (server only, no window) when headless:=true
  robot_state_publisher  the combined description at hardware_profile:=gazebo, use_sim_time
  ros_gz_sim create      spawns the model from the robot_description topic; the model carries the base's DiffDrive and
                         JointStatePublisher plugins (platform-sw's, unchanged) and our gz_ros2_control plugin, which
                         starts the controller manager inside the simulator
  ros_gz_bridge          platform-sw's own gz_bridge.yaml, BY REFERENCE (clock, /cmd_vel, /odom, /tf, sensors; every topic
                         Nav2 and docking use, under the names they use), plus one bridge of ours for the base's joint
                         states onto a separate topic (see below)
  spawners               joint_state_broadcaster, then left_arm, right_arm, left_gripper and right_gripper controllers
  joint_state_publisher  the ONLY publisher of /joint_states: it merges the base's wheel joints and the arms' joints

/joint_states: platform-sw's JointStatePublisher (scoped by its own <joint_name> list to the two wheels) and our
joint_state_broadcaster (the sixteen arm joints) must not both write to /joint_states. The wheels come in on
/base/joint_states, the arms on /arm/joint_states, and joint_state_publisher merges them. See the package README for what
was observed before this was decided.

odom -> base_link: from the DiffDrive plugin through the /tf bridge, and from nowhere else. robot_state_publisher cannot
publish it, because `odom` is not a link of the description (and with the planar joint enabled KDL turns it into a fixed
joint, check W02). The default here is enable_planar_base:=false.

Args
  mast_id   mast_1300 | mast_1350 | mast_1400 | mast_1450   (default mast_1350, the release configuration)
  world     path to a Gazebo world .sdf                       (default: worlds/empty.sdf in this package; platform-sw's is
            $(ros2 pkg prefix --share openamrobot_gazebo)/worlds/walled_world.sdf)
  headless  true | false     (default false) true runs `gz sim -s`: no Gazebo window, as the smoke test does
  physics_engine  the gz-physics plugin for whichever world is loaded (default gz-physics-bullet-featherstone-plugin).
            The Harmonic default, DART, cannot run this model's grippers: its velocity servo does not move the fingers and
            it has no mimic constraint, so finger_joint2 never follows finger_joint1. See the package README.
  rviz      true | false     (default false) RViz2 with the fixed frame `odom`, beside the simulator, for a person
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

DESCRIPTION = "openamrobot_upperbody_description"
CONTROL = "openamrobot_upperbody_control"
GAZEBO = "openamrobot_upperbody_gazebo"
PLATFORM_GAZEBO = "openamrobot_gazebo"
ARM_AND_GRIPPER_CONTROLLERS = ("left_arm_controller", "right_arm_controller",
                               "left_gripper_controller", "right_gripper_controller")
# Packages whose meshes the model references as package://<name>/...; Gazebo resolves that against the parent of the
# package's share directory (the same trick platform-sw's gz_simulator.launch.py uses for its own description).
MESH_PACKAGES = ("openamrobot_description", "openarm_description", DESCRIPTION)
DEFAULT_PHYSICS_ENGINE = "gz-physics-bullet-featherstone-plugin"      # the tests record it; see the README for why not DART
SPAWN_HEIGHT = "0.15"          # m above the ground plane; the model settles onto its wheels and casters in the first second
CONTROLLER_MANAGER_TIMEOUT = "120"      # s the spawners wait for the controller manager that the simulator hosts


def spawner(controller, *extra):
    return Node(package="controller_manager", executable="spawner", output="screen",
                arguments=[controller, "--controller-manager", "/controller_manager",
                           "--controller-manager-timeout", CONTROLLER_MANAGER_TIMEOUT, *extra],
                parameters=[{"use_sim_time": True}])


def generate_launch_description():
    mast_id = LaunchConfiguration("mast_id")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    physics_engine = LaunchConfiguration("physics_engine")
    rviz = LaunchConfiguration("rviz")
    gazebo_share = get_package_share_directory(GAZEBO)
    platform_gazebo_share = get_package_share_directory(PLATFORM_GAZEBO)

    resource_paths = [AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(gazebo_share, "worlds"))]
    for package in MESH_PACKAGES:
        resource_paths.append(AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", os.path.dirname(get_package_share_directory(package))))

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([FindPackageShare(DESCRIPTION), "urdf", "combined_robot.xacro"]),
            " mast_id:=", mast_id,
            " hardware_profile:=gazebo",
        ]),
        value_type=str,
    )

    gz_sim = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    gz_sim_window = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim),
        launch_arguments={"gz_args": ["-r -v 2 --physics-engine ", physics_engine, " ", world],
                          "on_exit_shutdown": "true"}.items(),
        condition=UnlessCondition(headless))
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim),
        launch_arguments={"gz_args": ["-r -s -v 2 --physics-engine ", physics_engine, " ", world],
                          "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(headless))

    robot_state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher",
                                 name="robot_state_publisher", output="screen",
                                 parameters=[{"use_sim_time": True, "robot_description": robot_description}])

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-name", "openamrobot", "-topic", "robot_description",
                            "-x", "0", "-y", "0", "-z", SPAWN_HEIGHT])

    # Platform-sw's bridge, by reference: it bridges exactly the topics Nav2 and docking consume, under their names.
    platform_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="platform_bridge", output="screen",
        parameters=[{"config_file": os.path.join(platform_gazebo_share, "config", "gz_bridge.yaml"),
                     "qos_overrides./tf_static.publisher.durability": "transient_local"}])
    # Ours: only what platform-sw's file leaves out, the base's joint states, onto a topic of their own.
    base_joint_states_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="base_joint_states_bridge", output="screen",
        parameters=[{"config_file": os.path.join(gazebo_share, "config", "gz_bridge_base_joint_states.yaml")}])

    joint_state_broadcaster = spawner(
        "joint_state_broadcaster", "--controller-ros-args", "-r joint_states:=/arm/joint_states")
    then_the_rest = RegisterEventHandler(OnProcessExit(
        target_action=joint_state_broadcaster,
        on_exit=[spawner(name) for name in ARM_AND_GRIPPER_CONTROLLERS]))

    joint_state_merger = Node(
        package="joint_state_publisher", executable="joint_state_publisher", name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "source_list": ["/base/joint_states", "/arm/joint_states"], "rate": 50}])

    # `gz sim` is started by a ruby wrapper, and a SIGINT sent to the wrapper alone (which is what launch_testing, a parent
    # launch file or `kill` do) ends the wrapper and leaves the simulator running as an orphan, holding its transport
    # partition. Asking the server to stop through its own service ends both. It does nothing when the server is gone.
    stop_server = RegisterEventHandler(OnShutdown(on_shutdown=[ExecuteProcess(
        cmd=["gz", "service", "-s", "/server_control", "--reqtype", "gz.msgs.ServerControl", "--reptype", "gz.msgs.Boolean",
             "--timeout", "3000", "--req", "stop: true"], output="log")]))

    rviz_node = Node(package="rviz2", executable="rviz2", output="screen", condition=IfCondition(rviz),
                     parameters=[{"use_sim_time": True}],
                     arguments=["-d", os.path.join(gazebo_share, "config", "sim.rviz")])

    return LaunchDescription([
        DeclareLaunchArgument("mast_id", default_value="mast_1350",
                              description="mast_1300 | mast_1350 | mast_1400 | mast_1450"),
        DeclareLaunchArgument("world", default_value=os.path.join(gazebo_share, "worlds", "empty.sdf"),
                              description="Path to the Gazebo world .sdf"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="true | false. gz sim -s: server only, no Gazebo window"),
        DeclareLaunchArgument("physics_engine", default_value=DEFAULT_PHYSICS_ENGINE,
                              description="gz-physics plugin; the DART default cannot drive the grippers (README)"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="true | false. RViz2 beside the simulator, fixed frame odom"),
        *resource_paths,
        gz_sim_window,
        gz_sim_server,
        robot_state_publisher,
        platform_bridge,
        base_joint_states_bridge,
        spawn,
        joint_state_broadcaster,
        then_the_rest,
        joint_state_merger,
        rviz_node,
        stop_server,
    ])
