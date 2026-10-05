"""T15: the `gazebo` profile drives the arms with gz_ros2_control/GazeboSimSystem, same joints and interfaces as `mock`.

Expansion only: nothing launches Gazebo here. The block must not need a hardware plugin of the other profiles, and
the base still has no ros2_control (its DiffDrive plugin is platform-sw's, ADR-003).
"""
from conftest import expand
from ros2_control_utils import arm_joints_needing_an_interface, blocks, controlled_joints, element_text, plugins

GAZEBO = "gz_ros2_control/GazeboSimSystem"


def test_t15_gazebo_arms_use_gazebo_sim_system():
    assert plugins(expand(hardware_profile="gazebo")) == [("openamr_arms_gazebo", GAZEBO)]
    assert blocks(expand(hardware_profile="gazebo"))[0].get("type") == "system"


def test_t15_gazebo_has_neither_mock_nor_hardware_plugin():
    text = element_text(expand(hardware_profile="gazebo"))
    for forbidden in ("mock_components", "openarm_hardware", "OpenArmHW"):
        assert forbidden not in text, f"{forbidden} appears in the gazebo expansion"


def test_t15_gazebo_controls_exactly_the_mock_joints_with_the_same_interfaces():
    gazebo, mock = expand(hardware_profile="gazebo"), expand(hardware_profile="mock")
    assert controlled_joints(gazebo) == controlled_joints(mock)
    assert sorted(controlled_joints(gazebo)) == arm_joints_needing_an_interface(gazebo)


def test_t15_the_base_has_no_ros2_control():
    assert all(n.startswith("openarm_") for n in controlled_joints(expand(hardware_profile="gazebo")))


def test_t15_finger_joint2_is_not_declared_and_stays_a_urdf_mimic():
    """Upstream leaves finger_joint2 out of ros2_control; the URDF <mimic> carries it. Gazebo handling is a TODO."""
    root = expand(hardware_profile="gazebo")
    declared = controlled_joints(root)
    for side in ("left", "right"):
        assert f"openarm_{side}_finger_joint2" not in declared
        assert root.find(f"joint[@name='openarm_{side}_finger_joint2']/mimic") is not None
