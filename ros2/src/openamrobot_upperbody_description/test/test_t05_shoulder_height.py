"""T05: shoulder height. The floor-relative height of openarm_{left,right}_base_link equals mast_ids[mast_id].shoulder_height.

Mast math is ADR-006 as corrected 5 Oct:
    mast_offset = shoulder_height - base_link_height_above_floor - mast_base_pose.z - left_arm_mount_pose.z
The identity asserted here does not depend on the datum values, so it survives the placeholders being replaced.
Floor = -base_link_height_above_floor, in base_link.
"""
import copy
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

from conftest import MAST_IDS, MASTS, expand, expand_xml, load_datums, xacro_files
from urdf_utils import fixed_fk, position

TOLERANCE = 0.001  # m
SIDES = ("left", "right")


def mast_offset(datums, mast_id):
    """ADR-006, evaluated independently of the xacro."""
    dt = datums["datums"]
    return (MAST_IDS["mast_ids"][mast_id]["shoulder_height"]
            - dt["base_link_height_above_floor"]["value"]
            - dt["mast_base_pose"]["value"]["xyz"][2]
            - dt["left_arm_mount_pose"]["value"]["xyz"][2])


def shoulder_heights(root, datums):
    """{side: height of openarm_<side>_base_link above the floor}, by fixed-joint FK from base_link."""
    floor_z = -datums["datums"]["base_link_height_above_floor"]["value"]
    return {side: position(fixed_fk(root, "base_link", f"openarm_{side}_base_link"))[2] - floor_z for side in SIDES}


@pytest.mark.parametrize("mast_id", MASTS)
def test_t05_shoulder_height_matches_mast_id(mast_id):
    heights = shoulder_heights(expand(mast_id=mast_id, arm_package="openarm"), load_datums())
    expected = MAST_IDS["mast_ids"][mast_id]["shoulder_height"]
    for side, height in heights.items():
        assert abs(height - expected) <= TOLERANCE, f"{mast_id} {side}: {height:.4f} m, expected {expected:.4f} m"


@pytest.mark.parametrize("mast_id", MASTS)
def test_t05_mast_mount_joint_carries_the_adr006_offset(mast_id):
    root = expand(mast_id=mast_id, arm_package="openarm")
    joint = next(j for j in root.findall("joint") if j.get("name") == "mast_mount_joint")
    z = float(joint.find("origin").get("xyz").split()[2])
    assert z == pytest.approx(mast_offset(load_datums(), mast_id), abs=1e-9)


def test_t05_identity_survives_replacing_the_placeholders(tmp_path):
    """Move every datum the formula reads: the identity still holds, which is why T05 asserts it and not 1.0865."""
    datums = load_datums()
    dt = datums["datums"]
    dt["base_link_height_above_floor"]["value"] = 0.0535          # the 200 mm hub wheel case
    dt["mast_base_pose"]["value"]["xyz"][2] = 0.25
    dt["left_arm_mount_pose"]["value"]["xyz"][2] = 0.03
    dt["right_arm_mount_pose"]["value"]["xyz"][2] = 0.03
    path = tmp_path / "datums_moved.yaml"
    path.write_text(yaml.safe_dump(datums))
    for mast_id in MASTS:
        root = ET.fromstring(expand_xml(mast_id=mast_id, arm_package="openarm", datums_file=str(path)))
        for side, height in shoulder_heights(root, datums).items():
            expected = MAST_IDS["mast_ids"][mast_id]["shoulder_height"]
            assert abs(height - expected) <= TOLERANCE, f"{mast_id} {side}: {height:.4f}, expected {expected:.4f}"


def test_t05_left_and_right_plate_z_are_equal():
    """One mast, one shoulder height. A mismatch is a FAIL, not two datums."""
    dt = load_datums()["datums"]
    left, right = (dt[f"{s}_arm_mount_pose"]["value"]["xyz"][2] for s in SIDES)
    assert left == right, f"left_arm_mount_pose.z = {left} but right_arm_mount_pose.z = {right}"


def test_t05_left_right_mismatch_is_detected_by_the_height_check(tmp_path):
    """The equality assertion above is necessary: the offset reads the left plate only."""
    datums = load_datums()
    datums["datums"]["right_arm_mount_pose"]["value"]["xyz"][2] = 0.02
    path = tmp_path / "datums_mismatch.yaml"
    path.write_text(yaml.safe_dump(datums))
    root = ET.fromstring(expand_xml(mast_id="mast_1350", arm_package="openarm", datums_file=str(path)))
    heights = shoulder_heights(root, datums)
    assert abs(heights["left"] - 1.350) <= TOLERANCE
    assert abs(heights["right"] - 1.350) > TOLERANCE


# --- nothing here may read the OpenArm *body* reference points (ADR-008 does not install the body) ----------------

FORBIDDEN = [
    ("body/struct/reference_points", re.compile(r"body/struct/reference_points")),
    ("0.698 (the OpenArm body's arm-mount height)", re.compile(r"(?<![\d.])0?\.698")),
]


def forbidden_hits(text):
    return [label for label, pattern in FORBIDDEN if pattern.search(text)]


def test_t05_xacro_does_not_read_upstream_body_reference_points():
    hits = {}
    for path in xacro_files():
        with open(path) as f:
            found = forbidden_hits(f.read())     # raw text, comments included: the strictest reading
        if found:
            hits[path] = found
    assert not hits, f"upstream body reference point used: {hits}"


@pytest.mark.parametrize("text,expected", [
    ('<xacro:property name="z" value="0.698"/>', 1),
    ("value=.698", 1),
    ("$(find openarm_description)/config/body/struct/reference_points.yaml", 1),
    ("0.698 and body/struct/reference_points", 2),
    ("10.698 0.6 1.0865 0.0698", 0),
])
def test_t05_forbidden_pattern_matcher(text, expected):
    assert len(forbidden_hits(text)) == expected
