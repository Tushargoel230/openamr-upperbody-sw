"""W01 (WARN): base_link carries an inertial, and KDL does not support a root link with an inertia.

`robot_state_publisher` logs: "The root link base_link has an inertia specified in the URDF, but KDL does not support
a root link with an inertia." The inertial comes from platform-sw's base. KDL silently drops it, and MoveIt uses KDL,
so it matters in A7. NOT FIXED here: the published workaround is a dummy root link above base_link, which would change
the root of the frame contract, and the base belongs to platform-sw (ADR-005). Coordination item for the WP-F5 owner.

This test passes while recording the WARN; it fails only if the condition disappears (then W01 can be retired).
"""
import warnings

from conftest import expand, tree_facts, write_sidecar

W01 = ("W01 WARN: frame base_link (root) has an <inertial> in the URDF; KDL does not support a root-link inertia, so "
       "KDL consumers (robot_state_publisher, MoveIt in A7) do not see it: base_link inertia not seen by KDL consumers. "
       "Not fixed: a dummy root above base_link would change the frame contract (ADR-005).")


def test_w01_base_link_inertia_not_seen_by_kdl(record_property):
    root = expand(mast_id="mast_1350", arm_package="openarm")
    links, _, child_of = tree_facts(root)
    assert sorted(set(links) - set(child_of)) == ["base_link"]
    base_link = next(link for link in root.findall("link") if link.get("name") == "base_link")
    inertial = base_link.find("inertial")
    assert inertial is not None, "base_link no longer has an inertial: retire check W01"
    record_property("check_id", "W01")
    record_property("base_link_mass_kg", inertial.find("mass").get("value"))
    write_sidecar("W01", "kdl_root_inertia", "base_link inertia not seen by KDL consumers", "WARN", {
        "frame": "base_link (root, platform-sw)",
        "base_link_mass_kg": float(inertial.find("mass").get("value")),
        "consequence": "robot_state_publisher logs that KDL does not support a root link with an inertia; KDL drops it "
                       "silently and MoveIt uses KDL, so it matters in A7",
        "not_fixed": "the published workaround is a dummy root above base_link, which would change the frame contract's "
                     "root; the base belongs to platform-sw (ADR-005)",
        "action": "coordination item for the WP-F5 owner with the base-macro refactor proposal; PR known-limitation; "
                  "listed in the T10 mass report",
    })
    warnings.warn(W01, UserWarning, stacklevel=1)
