"""View the combined OpenAMRobot description in RViz.

Starts robot_state_publisher, joint_state_publisher_gui and RViz2 for combined_robot.xacro.

    ros2 launch openamrobot_upperbody_description view.launch.py mast_id:=mast_1350 arm_package:=openarm

Overlay: the OpenArm meshes are referenced as package://openarm_description/..., and the base meshes as
package://openamrobot_description/.... They only resolve once the workspace overlay that contains our build is
sourced (`source ~/openamr_ws/install/setup.bash`). Without it RViz loads the model but logs mesh errors and draws
the arms and base as missing geometry.

Args
  mast_id      mast_1300 | mast_1350 | mast_1400 | mast_1450   (default mast_1350, the release configuration)
  arm_package  openarm | so101 | none                           (default openarm)
An unknown value stops xacro with the list of accepted values.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = "openamrobot_upperbody_description"


def generate_launch_description():
    mast_id = LaunchConfiguration("mast_id")
    arm_package = LaunchConfiguration("arm_package")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([FindPackageShare(PKG), "urdf", "combined_robot.xacro"]),
            " mast_id:=", mast_id,
            " arm_package:=", arm_package,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("mast_id", default_value="mast_1350",
                              description="mast_1300 | mast_1350 | mast_1400 | mast_1450"),
        DeclareLaunchArgument("arm_package", default_value="openarm",
                              description="openarm | so101 | none"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], output="screen"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui", output="screen"),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", PathJoinSubstitution([FindPackageShare(PKG), "config", "view.rviz"])]),
    ])
