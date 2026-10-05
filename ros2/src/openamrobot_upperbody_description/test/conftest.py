"""Shared helpers for the description tests (xacro Python API; no ROS runtime, only the ament index for $(find)).

The expansion runs on the *installed* package so that what is tested is what a user gets from `ros2`.
"""
import copy
import functools
import itertools
import json
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

DATUMS_FILE = os.path.join(SHARE, "config", "datums.yaml")
SCHEMA_FILE = os.path.join(SHARE, "config", "datums.schema.json")
FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def load_datums():
    """Fresh parsed datums.yaml (a deep copy: tests may mutate it)."""
    return copy.deepcopy(load_yaml(DATUMS_FILE))


def load_schema():
    with open(SCHEMA_FILE) as f:
        return json.load(f)


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


def xacro_files():
    """Every .xacro installed with the package (the sources of truth for what the model references)."""
    out = []
    for dirpath, _, names in os.walk(os.path.join(SHARE, "urdf")):
        out += [os.path.join(dirpath, n) for n in names if n.endswith(".xacro")]
    return sorted(out)
