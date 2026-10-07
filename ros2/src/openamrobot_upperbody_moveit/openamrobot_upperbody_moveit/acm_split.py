"""Which side of the collision matrix a pair is on, and which of our pairs the kinematic tree settles without sampling.

Two derivations, both from the model and never from a list somebody keeps up to date (M03, VERIFICATION.md):

1. **Provenance.** A link is *upstream's* (Enactic's) or *ours* (A6's or platform-sw's: base, wheels, casters, lidar, camera, mast, carriage,
   whatever is added next). The frame contract's `owner` column decides where the link is listed; a link the contract does not list falls
   back to the `openarm_` prefix, which is how upstream names every link of its own, and a link that is neither listed nor prefixed is OURS:
   an unknown link must never be able to hide a defect behind someone else's name. A pair whose two links are both upstream's is
   **inherited**; every other pair is **ours**. The two sets partition the matrix by construction: a pair is inherited or it is not.

2. **Construction.** Between two links of a kinematic tree there is exactly one path. If no joint on it can move (every one is `fixed`
   and none is a mimic), the relative transform of the two links is a constant, so their collision status is the same in every
   configuration and one evaluation settles it for all of them. Such a pair is *certified by construction*. If a joint on the path can
   move, the pair is *sampled*, and sampling can only ever falsify.

Pure Python, no ROS runtime: `python3 -m pytest test/test_acm_split.py` runs it.
"""
import hashlib
import os
import re
import xml.etree.ElementTree as ET

INHERITED, OURS = "inherited", "ours"
UPSTREAM_PREFIX = "openarm_"
UPSTREAM_OWNER_PREFIX = "upstream"          # the frame contract writes "upstream: enactic/openarm_description"
ARM_LINK = re.compile(r"^openarm_(left|right)_")


class ProvenanceError(RuntimeError):
    """The frame contract and the link names disagree about who owns a link. Raised instead of picking one."""


def pair_key(a, b):
    return tuple(sorted((a, b)))


def contract_owners(entries):
    """{link: owner string} for the links of the frame contract (`entries` is the YAML's list)."""
    return {e["name"]: e.get("owner", "") for e in entries if e.get("kind") == "link"}


def link_provenance(links, owners):
    """{link: ("upstream" | "ours", how)} for every link in `links`. `how` is "contract" (listed there), "prefix" (not listed, named like an
    upstream link) or "unlisted" (neither: ours, because nothing says otherwise).

    A link listed in the contract as upstream's but not named `openarm_*`, or named `openarm_*` and listed as someone else's, is a
    contradiction between two sources of the same fact: ProvenanceError, so it is looked at instead of guessed."""
    out = {}
    for link in links:
        prefixed = link.startswith(UPSTREAM_PREFIX)
        if link in owners:
            upstream = owners[link].startswith(UPSTREAM_OWNER_PREFIX)
            if upstream != prefixed:
                raise ProvenanceError(f"{link}: the frame contract's owner is {owners[link]!r} but the name "
                                      f"{'has' if prefixed else 'does not have'} the {UPSTREAM_PREFIX!r} prefix")
            out[link] = ("upstream" if upstream else "ours", "contract")
        elif prefixed:
            out[link] = ("upstream", "prefix")
        else:
            out[link] = ("ours", "unlisted")
    return out


def split_pairs(pairs, provenance):
    """(inherited, ours): two lists of pair keys. Inherited = both links upstream's; ours = every other pair. Every pair lands in exactly one."""
    inherited, ours = [], []
    for a, b in sorted(pairs):
        (inherited if provenance[a][0] == "upstream" and provenance[b][0] == "upstream" else ours).append((a, b))
    return inherited, ours


def urdf_tree(urdf_root):
    """{child link: (parent link, joint name, movable)}. A joint is movable unless it is `fixed` and has no <mimic>: a mimic follows another
    joint, so the link it moves is not at a constant pose, however the joint is typed."""
    tree = {}
    for joint in urdf_root.findall("joint"):
        movable = joint.get("type") != "fixed" or joint.find("mimic") is not None
        tree[joint.find("child").get("link")] = (joint.find("parent").get("link"), joint.get("name"), movable)
    return tree


def path_to_root(tree, link):
    path = [link]
    while path[-1] in tree:
        path.append(tree[path[-1]][0])
    return path


def movable_joints_between(tree, a, b):
    """Names of the movable joints on the one path between links `a` and `b` ([] means their relative transform is constant)."""
    up_a, up_b = path_to_root(tree, a), path_to_root(tree, b)
    common = next((l for l in up_a if l in up_b), None)
    if common is None:
        raise ValueError(f"{a} and {b} are not in the same kinematic tree")
    joints = []
    for up in (up_a, up_b):
        for link in up[:up.index(common)]:
            joints.append(tree[link])
    return sorted(name for _, name, movable in joints if movable)


def split_by_kinematics(pairs, urdf_root):
    """(by_construction, sampled): the pairs with no movable joint between their links, and the rest."""
    tree = urdf_tree(urdf_root)
    by_construction, sampled = [], []
    for a, b in pairs:
        (sampled if movable_joints_between(tree, a, b) else by_construction).append((a, b))
    return by_construction, sampled


def origin_of(pair, imported):
    """Where an inherited pair came from: `imported` (listed in section 1 of the SRDF: Enactic's own assertion about their arm) or
    `generated` (written by the collision-matrix tool for two upstream links; nobody at Enactic asserted it)."""
    return "imported_from_upstream" if pair_key(*pair) in imported else "generated_for_upstream_links"


def arm_relation(pair):
    """`same_arm`, `left_vs_right`, or None, for a pair of upstream arm links. A left-vs-right pair is two Enactic links whose relative pose is
    set by OUR mast plates (datums), so a collision there is not obviously upstream's to answer for: the report says so."""
    matches = [ARM_LINK.match(l) for l in pair]
    if not all(matches):
        return None
    return "left_vs_right" if len({m.group(1) for m in matches}) == 2 else "same_arm"


def load_contract_owners(description_share):
    import yaml
    with open(os.path.join(description_share, "config", "frame_contract.yaml")) as f:
        return contract_owners(yaml.safe_load(f)["entries"])


def parse_urdf(text):
    return ET.fromstring(text)


def acm_pairs(srdf_root):
    """{pair key: reason} for every disable_collisions of an SRDF (parsed). A pair listed twice is an error: each pair is disabled once."""
    pairs = {}
    for d in srdf_root.findall("disable_collisions"):
        key = pair_key(d.get("link1"), d.get("link2"))
        if key in pairs:
            raise ValueError(f"{key[0]} <-> {key[1]} is disabled twice")
        pairs[key] = d.get("reason")
    return pairs


def acm_fingerprint(pairs):
    """sha256 of the matrix as a set of (pair, reason) lines, sorted: the same matrix gives the same hash whatever the file's order, comments
    or whitespace, and any added, removed or relabelled pair changes it. This is what `the checked-in ACM is the contract` is checked against."""
    text = "".join(f"{a} {b} {reason}\n" for (a, b), reason in sorted(pairs.items()))
    return "sha256:" + hashlib.sha256(text.encode()).hexdigest()


def diff_acm(old, new):
    """What differs between two matrices ({pair: reason}): ({pair: reason} only in new, {pair: reason} only in old, {pair: (old, new)})."""
    added = {p: r for p, r in new.items() if p not in old}
    removed = {p: r for p, r in old.items() if p not in new}
    relabelled = {p: (old[p], new[p]) for p in old.keys() & new.keys() if old[p] != new[p]}
    return added, removed, relabelled
