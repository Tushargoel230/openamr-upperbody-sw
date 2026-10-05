"""T03: one root `base_link`, no duplicate link or joint names (and no `world`, no dangling joint ends)."""
from conftest import ARM_PACKAGES, MASTS, duplicates, expand, tree_facts


def test_t03_single_root_and_unique_names(combo):
    mast_id, arm_package = combo
    links, joints, child_of = tree_facts(expand(mast_id=mast_id, arm_package=arm_package))

    roots = sorted(set(links) - set(child_of))
    assert roots == ["base_link"], f"roots: {roots}"
    assert not duplicates(links), f"duplicate links: {duplicates(links)}"
    assert not duplicates(joints), f"duplicate joints: {duplicates(joints)}"

    # every link has at most one parent joint, and every joint end names a declared link
    assert len(child_of) == len(joints), "a link is the child of more than one joint"
    declared = set(links)
    dangling = sorted((set(child_of) | set(child_of.values())) - declared)
    assert not dangling, f"joints reference undeclared links: {dangling}"

    # the upstream macro must not inject a world root (stop condition of the step)
    assert "world" not in declared


def test_t03_openarm_attaches_to_our_plates():
    root = expand(arm_package="openarm")
    _, _, child_of = tree_facts(root)
    assert child_of["mast_left_arm_mount_link"] == "mast_mount_link"
    assert child_of["mast_right_arm_mount_link"] == "mast_mount_link"
    assert child_of["openarm_left_base_link"] == "mast_left_arm_mount_link"
    assert child_of["openarm_right_base_link"] == "mast_right_arm_mount_link"
    assert not [n for n in tree_facts(root)[0] if "body" in n], "OpenArm body must not be instantiated (ADR-008)"


def test_t03_no_arm_subtree_without_openarm():
    for arm_package in ARM_PACKAGES:
        links, joints, _ = tree_facts(expand(mast_id=MASTS[0], arm_package=arm_package))
        has_arm = any(n.startswith("openarm_") or n.startswith("mast_left_") for n in links)
        assert has_arm == (arm_package == "openarm")
        assert ("left_arm_mount_joint" in joints) == (arm_package == "openarm")
