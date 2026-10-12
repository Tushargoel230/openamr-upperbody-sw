"""T01: every mast_id x arm_package combination expands (hardware_profile and the planar flag are accepted too)."""
import xml.etree.ElementTree as ET

import pytest

from conftest import HARDWARE_PROFILES, MAST_IDS, MASTS, expand, expand_xml


def test_t01_matrix_is_four_by_three():
    assert len(MASTS) == 4


def test_t01_expands(combo):
    mast_id, arm_package = combo
    root = expand(mast_id=mast_id, arm_package=arm_package)
    assert root.tag == "robot" and root.get("name") == "openamrobot"
    assert root.find("link[@name='base_link']") is not None
    assert root.find("link[@name='mast_mount_link']") is not None


@pytest.mark.parametrize("hardware_profile", HARDWARE_PROFILES)
def test_t01_hardware_profiles_expand(hardware_profile):
    assert expand(hardware_profile=hardware_profile).find("link[@name='base_link']") is not None


def test_t01_planar_flag_accepted():
    assert expand(enable_planar_base="true").find("link[@name='base_link']") is not None


def test_t01_default_is_the_release_configuration():
    assert MAST_IDS["default"] == "mast_1350"
    assert MAST_IDS["mast_ids"]["mast_1350"]["class"] == "release"
    assert expand_xml() == expand_xml(mast_id="mast_1350", arm_package="openarm", hardware_profile="mock")
