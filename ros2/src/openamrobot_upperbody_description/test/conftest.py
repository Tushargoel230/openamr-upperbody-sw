"""Shared helpers for the description tests (xacro Python API; no ROS runtime, only the ament index for $(find)).

The expansion runs on the *installed* package so that what is tested is what a user gets from `ros2`.
"""
import functools
import itertools
import os
import xml.etree.ElementTree as ET
from collections import Counter

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

PKG = "openamrobot_upperbody_description"
SHARE = get_package_share_directory(PKG)
COMBINED = os.path.join(SHARE, "urdf", "combined_robot.xacro")

with open(os.path.join(SHARE, "config", "mast_ids.yaml")) as _f:
    MAST_IDS = yaml.safe_load(_f)

MASTS = list(MAST_IDS["mast_ids"])
ARM_PACKAGES = ["openarm", "so101", "none"]
HARDWARE_PROFILES = ["mock", "gazebo", "real"]
MATRIX = list(itertools.product(MASTS, ARM_PACKAGES))


def expand_xml(**mappings):
    """Expand combined_robot.xacro with the given args; return the URDF text. Raises xacro.XacroException."""
    return xacro.process_file(COMBINED, mappings={k: str(v) for k, v in mappings.items()}).toxml()


@functools.lru_cache(maxsize=None)
def _cached(items):
    return expand_xml(**dict(items))


def expand(**mappings):
    """Cached expansion -> ElementTree root."""
    return ET.fromstring(_cached(tuple(sorted(mappings.items()))))


def tree_facts(root):
    links = [e.get("name") for e in root.findall("link")]
    joints = root.findall("joint")
    child_of = {j.find("child").get("link"): j.find("parent").get("link") for j in joints}
    return links, [j.get("name") for j in joints], child_of


def duplicates(names):
    return sorted(n for n, c in Counter(names).items() if c > 1)


@pytest.fixture(params=MATRIX, ids=[f"{m}-{a}" for m, a in MATRIX])
def combo(request):
    return request.param
