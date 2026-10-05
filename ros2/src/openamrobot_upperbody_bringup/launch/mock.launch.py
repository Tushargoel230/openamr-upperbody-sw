"""Mock bringup: the arms on mock_components/GenericSystem, all five controllers active, nothing else.

    ros2 launch openamrobot_upperbody_bringup mock.launch.py [mast_id:=mast_1350] [rviz:=true]

Starts robot_state_publisher on the combined description with hardware_profile:=mock, ros2_control_node with
openamrobot_upperbody_control/config/arm_controllers.yaml, and spawners for joint_state_broadcaster,
left_arm_controller, right_arm_controller, left_gripper_controller and right_gripper_controller. The mobile base is not
under ros2_control (ADR-003) and is not started here.

Args
  mast_id  mast_1300 | mast_1350 | mast_1400 | mast_1450   (default mast_1350, the release configuration)
  rviz     true | false                                     (default false: headless)

Overlay: the OpenArm and base meshes resolve through the workspace overlay; source install/setup.bash first (RViz only).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

DESCRIPTION = "openamrobot_upperbody_description"
CONTROL = "openamrobot_upperbody_control"
ARM_AND_GRIPPER_CONTROLLERS = ("left_arm_controller", "right_arm_controller",
                               "left_gripper_controller", "right_gripper_controller")


def spawner(controller):
    return Node(package="controller_manager", executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager"], output="screen")


def generate_launch_description():
    mast_id = LaunchConfiguration("mast_id")
    rviz = LaunchConfiguration("rviz")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([FindPackageShare(DESCRIPTION), "urdf", "combined_robot.xacro"]),
            " mast_id:=", mast_id,
            " hardware_profile:=mock",
        ]),
        value_type=str,
    )
    controllers = PathJoinSubstitution([FindPackageShare(CONTROL), "config", "arm_controllers.yaml"])

    robot_state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher",
                                 parameters=[{"robot_description": robot_description}], output="screen")
    ros2_control_node = Node(package="controller_manager", executable="ros2_control_node",
                             parameters=[{"robot_description": robot_description}, controllers], output="screen")
    rviz_node = Node(package="rviz2", executable="rviz2", output="screen", condition=IfCondition(rviz),
                     arguments=["-d", PathJoinSubstitution([FindPackageShare(DESCRIPTION), "config", "view.rviz"])])

    # joint_state_broadcaster first: the other four are spawned when it has exited (its spawner exits once it is active).
    joint_state_broadcaster = spawner("joint_state_broadcaster")
    then_the_rest = RegisterEventHandler(OnProcessExit(
        target_action=joint_state_broadcaster,
        on_exit=[spawner(name) for name in ARM_AND_GRIPPER_CONTROLLERS]))

    return LaunchDescription([
        DeclareLaunchArgument("mast_id", default_value="mast_1350",
                              description="mast_1300 | mast_1350 | mast_1400 | mast_1450"),
        DeclareLaunchArgument("rviz", default_value="false", description="true | false. Start RViz2 (default headless)"),
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster,
        then_the_rest,
        rviz_node,
    ])
