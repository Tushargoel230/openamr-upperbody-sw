"""M03's two derivations: the inherited/ours split of the collision matrix from link provenance, and the set of our pairs that the kinematic
tree settles without sampling. Both are computed, never listed, so these tests are what keeps them honest when a link is added.

No ROS runtime and no simulator: the xacro Python API and a small forward-kinematics of the URDF's own joints."""
import itertools
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from conftest import DESCRIPTION_SHARE, MAST_IDS, SRDF_FILE, srdf_root, urdf_root
from openamrobot_upperbody_moveit import acm_split as split

BEGIN = "<!-- BEGIN GENERATED ACM -->"


def srdf_pairs():
    return {split.pair_key(d.get("link1"), d.get("link2")): d.get("reason") for d in srdf_root().findall("disable_collisions")}


def real_split(mast_id="mast_1350"):
    root = urdf_root(mast_id)
    links = [l.get("name") for l in root.findall("link")]
    provenance = split.link_provenance(links, split.load_contract_owners(DESCRIPTION_SHARE))
    inherited, ours = split.split_pairs(list(srdf_pairs()), provenance)
    return root, provenance, inherited, ours


@pytest.mark.parametrize("mast_id", MAST_IDS)
def test_every_pair_of_the_acm_is_inherited_or_ours_and_never_both(mast_id):
    _, _, inherited, ours = real_split(mast_id)
    pairs = set(srdf_pairs())
    assert set(inherited).isdisjoint(ours)
    assert set(inherited) | set(ours) == pairs
    assert len(inherited) + len(ours) == len(pairs), "a pair listed twice would count twice"
    assert inherited and ours, "an empty side means the provenance rule has stopped seeing one of the two owners"


def test_a_pair_is_inherited_exactly_when_both_links_are_upstreams():
    _, provenance, inherited, ours = real_split()
    upstream = {l for l, (side, _) in provenance.items() if side == "upstream"}
    assert all(set(p) <= upstream for p in inherited)
    assert all(not set(p) <= upstream for p in ours)
    assert all(l.startswith("openarm_") for l in upstream)
    assert not [l for l in provenance if l.startswith("openarm_") and l not in upstream]


def test_the_frame_contract_and_the_openarm_prefix_agree_on_every_link_of_the_model():
    """link_provenance raises on a contradiction, so reaching the assertions at every mast ID is the check; this also says which source decided."""
    for mast_id in MAST_IDS:
        _, provenance, *_ = real_split(mast_id)
        hows = {how for _, how in provenance.values()}
        assert hows <= {"contract", "prefix", "unlisted"}
    unlisted = sorted(l for l, (_, how) in real_split()[1].items() if how == "unlisted")
    assert unlisted == [], f"links the frame contract does not list (ours by default, but say so in the contract): {unlisted}"


def test_a_link_added_later_lands_on_the_right_side_without_a_list_being_updated():
    owners = {"base_link": "platform-sw", "openarm_left_link1": "upstream: enactic/openarm_description"}
    links = ["base_link", "openarm_left_link1", "lift_carriage_link", "openarm_left_new_link", "zed_mini_link"]
    prov = split.link_provenance(links, owners)
    assert prov["lift_carriage_link"] == ("ours", "unlisted") and prov["zed_mini_link"] == ("ours", "unlisted")
    assert prov["openarm_left_new_link"] == ("upstream", "prefix")
    inherited, ours = split.split_pairs(list(itertools.combinations(links, 2)), prov)
    assert inherited == [("openarm_left_link1", "openarm_left_new_link")]
    assert len(ours) == 9, "every pair with a link of ours in it is ours, including ours-against-upstream"


def test_contract_and_prefix_disagreeing_is_an_error_not_a_guess():
    with pytest.raises(split.ProvenanceError):
        split.link_provenance(["openarm_x"], {"openarm_x": "platform-sw"})
    with pytest.raises(split.ProvenanceError):
        split.link_provenance(["lift_link"], {"lift_link": "upstream: enactic/openarm_description"})


def tree(xml):
    return split.urdf_tree(ET.fromstring(xml))


def test_a_path_of_fixed_joints_has_no_movable_joint_and_one_movable_joint_anywhere_on_the_path_is_enough():
    t = tree("""<robot>
      <joint name="j_a" type="fixed"><parent link="root"/><child link="a"/></joint>
      <joint name="j_b" type="fixed"><parent link="a"/><child link="b"/></joint>
      <joint name="j_c" type="revolute"><parent link="a"/><child link="c"/></joint>
      <joint name="j_d" type="fixed"><parent link="c"/><child link="d"/></joint>
      <joint name="j_m" type="fixed"><parent link="root"/><child link="m"/><mimic joint="j_c"/></joint></robot>""")
    assert split.movable_joints_between(t, "a", "b") == []
    assert split.movable_joints_between(t, "root", "b") == []
    assert split.movable_joints_between(t, "b", "d") == ["j_c"]
    assert split.movable_joints_between(t, "d", "root") == ["j_c"]
    assert split.movable_joints_between(t, "m", "a") == ["j_m"], "a mimic joint moves its link whatever type it is declared"
    with pytest.raises(ValueError):
        split.movable_joints_between(t, "a", "elsewhere")


# --- an independent forward kinematics, to test the reasoning rather than repeat it -----------------------------------------------------
def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, angle):
    axis = np.array(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def forward_kinematics(root, values):
    """{link: 4x4 pose in the root link's frame} for joint values {joint: q} (a missing joint is 0; a mimic follows its leader)."""
    joints = {j.get("name"): j for j in root.findall("joint")}

    def q(name):
        j = joints[name]
        m = j.find("mimic")
        return values.get(name, 0.0) if m is None else values.get(m.get("joint"), 0.0) * float(m.get("multiplier", 1)) + float(m.get("offset", 0))

    poses = {}
    children = {j.find("child").get("link"): j for j in joints.values()}
    parents = {j.find("parent").get("link") for j in joints.values()}
    roots = [p for p in parents if p not in children]
    assert len(roots) == 1
    poses[roots[0]] = np.eye(4)

    def pose(link):
        if link not in poses:
            j = children[link]
            origin = j.find("origin")
            xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
            t = np.eye(4)
            t[:3, :3], t[:3, 3] = rpy_matrix(*rpy), xyz
            kind = j.get("type")
            if kind in ("revolute", "continuous"):
                a = j.find("axis")
                r = np.eye(4)
                r[:3, :3] = axis_angle([float(v) for v in (a.get("xyz") if a is not None else "1 0 0").split()], q(j.get("name")))
                t = t @ r
            elif kind == "prismatic":
                a = j.find("axis")
                d = np.array([float(v) for v in (a.get("xyz") if a is not None else "1 0 0").split()]) * q(j.get("name"))
                s = np.eye(4)
                s[:3, 3] = d
                t = t @ s
            else:
                assert kind == "fixed", f"{j.get('name')}: {kind}"
            poses[link] = pose(j.find("parent").get("link")) @ t
        return poses[link]

    for link in [l.get("name") for l in root.findall("link")]:
        pose(link)
    return poses


def random_configuration(root, rng):
    values = {}
    for j in root.findall("joint"):
        if j.get("type") in ("revolute", "continuous", "prismatic") and j.find("mimic") is None:
            lim = j.find("limit")
            lo, hi = (float(lim.get("lower")), float(lim.get("upper"))) if j.get("type") != "continuous" else (-math.pi, math.pi)
            values[j.get("name")] = rng.uniform(lo, hi)
    return values


@pytest.mark.parametrize("mast_id", MAST_IDS)
def test_the_pairs_certified_by_construction_have_the_same_relative_transform_in_every_configuration(mast_id):
    """The reasoning, tested against a forward kinematics that shares no code with it: for every pair the tree calls constant, the relative pose
    is identical at several random configurations; and pairs it sends to sampling are not all constant (the rule is not vacuous)."""
    root, _, _, ours = real_split(mast_id)
    ours_never = [p for p in ours if srdf_pairs()[p] == "Never"]
    certified, sampled = split.split_by_kinematics(ours_never, root)
    rng = np.random.default_rng(7)
    configs = [forward_kinematics(root, random_configuration(root, rng)) for _ in range(4)]
    for a, b in certified:
        relative = [np.linalg.inv(c[a]) @ c[b] for c in configs]
        for other in relative[1:]:
            assert np.allclose(relative[0], other, atol=1e-9), f"{a} <-> {b} is called constant but moves"
    moves = [any(not np.allclose(np.linalg.inv(configs[0][a]) @ configs[0][b], np.linalg.inv(c[a]) @ c[b], atol=1e-9) for c in configs[1:])
             for a, b in sampled]
    assert all(moves), [p for p, m in zip(sampled, moves) if not m]


def test_the_certified_set_is_neither_empty_nor_most_of_the_matrix():
    """The stop condition of P11 as a tripwire: an empty set or an implausibly large one means the tree reasoning is wrong."""
    root, _, inherited, ours = real_split()
    ours_never = [p for p in ours if srdf_pairs()[p] == "Never"]
    certified, sampled = split.split_by_kinematics(ours_never, root)
    assert certified and sampled
    assert 0.05 < len(certified) / len(ours_never) < 0.5, (len(certified), len(ours_never))
    # Nothing that has an arm joint or a wheel joint on its path is certified; the platform's fixed hardware is.
    assert ("base_link", "left_wheel_link") not in set(certified)
    assert ("base_link", "openarm_left_base_link") in set(certified), "the arm's base link is rigidly on the mast: its pairs with the base are constant"
    assert all("openarm_left_joint" not in " ".join(split.movable_joints_between(split.urdf_tree(root), *p)) for p in certified)


def test_origin_and_arm_relation_say_what_a_warn_is_about():
    imported = {("openarm_left_link1", "openarm_left_link2")}
    assert split.origin_of(("openarm_left_link2", "openarm_left_link1"), imported) == "imported_from_upstream"
    assert split.origin_of(("openarm_left_ee_link1", "openarm_left_link2"), imported) == "generated_for_upstream_links"
    assert split.arm_relation(("openarm_left_ee_link1", "openarm_left_link2")) == "same_arm"
    assert split.arm_relation(("openarm_left_link3", "openarm_right_link4")) == "left_vs_right"
    assert split.arm_relation(("base_link", "openarm_left_link4")) is None
    assert split.arm_relation(("base_link", "mast_base_link")) is None
