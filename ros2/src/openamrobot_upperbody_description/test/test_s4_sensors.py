"""S4 sensor frames and the SO-101 stub: where they attach, what they carry, and when they exist."""
import numpy as np
import pytest

from conftest import ARM_PACKAGES, MASTS, expand, load_datums, tree_facts
from urdf_utils import fixed_fk, pose_to_matrix, position

CAMERAS = {
    "head_camera": ("mast_mount_link", "head_camera_pose"),
    "left_wrist_camera": ("openarm_left_ee_base_link", "left_wrist_camera_pose"),
    "right_wrist_camera": ("openarm_right_ee_base_link", "right_wrist_camera_pose"),
}


@pytest.mark.parametrize("mast_id", MASTS)
@pytest.mark.parametrize("name", CAMERAS)
def test_camera_frames_attach_where_the_contract_says_at_the_datum_pose(name, mast_id):
    parent, datum = CAMERAS[name]
    root = expand(mast_id=mast_id, arm_package="openarm")
    _, _, child_of = tree_facts(root)
    assert child_of[f"{name}_link"] == parent
    assert child_of[f"{name}_optical_frame"] == f"{name}_link"
    pose = load_datums()["datums"][datum]["value"]
    assert np.allclose(fixed_fk(root, parent, f"{name}_link"), pose_to_matrix(pose["xyz"], pose["rpy"]))


@pytest.mark.parametrize("name", CAMERAS)
def test_optical_frame_is_rep103(name):
    """z forward, x right, y down, with the link frame x forward, y left, z up."""
    root = expand(arm_package="openarm")
    rot = fixed_fk(root, f"{name}_link", f"{name}_optical_frame")
    assert np.allclose(position(rot), 0)
    assert np.allclose(rot[:3, 2], [1, 0, 0])      # optical z = link x (forward)
    assert np.allclose(rot[:3, 0], [0, -1, 0])     # optical x = -link y (right)
    assert np.allclose(rot[:3, 1], [0, 0, -1])     # optical y = -link z (down)


@pytest.mark.parametrize("name", CAMERAS)
def test_camera_links_are_visual_only(name):
    link = expand(arm_package="openarm").find(f"link[@name='{name}_link']")
    assert link.find("visual") is not None
    assert link.find("collision") is None and link.find("inertial") is None


def test_head_camera_is_on_the_mast_not_on_an_arm():
    root = expand(arm_package="openarm")
    chain, link = [], "head_camera_link"
    _, _, child_of = tree_facts(root)
    while link in child_of:
        link = child_of[link]
        chain.append(link)
    assert chain[:3] == ["mast_mount_link", "mast_base_link", "base_link"]
    assert not [n for n in chain if n.startswith("openarm_")]


@pytest.mark.parametrize("arm_package", [a for a in ARM_PACKAGES if a != "openarm"])
def test_no_camera_frames_without_openarm(arm_package):
    links, joints, _ = tree_facts(expand(arm_package=arm_package))
    assert not [n for n in links + joints if "camera" in n and not n.startswith("camera_")]


def test_so101_stub_is_one_link_at_identity_on_the_mast_mount():
    root = expand(arm_package="so101")
    links, joints, child_of = tree_facts(root)
    assert child_of["so101_mount_link"] == "mast_mount_link"
    assert np.allclose(fixed_fk(root, "mast_mount_link", "so101_mount_link"), np.eye(4))
    assert [n for n in links if n.startswith("so101_")] == ["so101_mount_link"]
    assert [n for n in joints if n.startswith("so101_")] == ["so101_mount_joint"]
    assert root.find("joint[@name='so101_mount_joint']").get("type") == "fixed"
    assert not [n for n in links if n.startswith("openarm_")]


@pytest.mark.parametrize("arm_package", ["openarm", "none"])
def test_no_so101_frames_unless_so101(arm_package):
    links, joints, _ = tree_facts(expand(arm_package=arm_package))
    assert not [n for n in links + joints if n.startswith("so101_")]
