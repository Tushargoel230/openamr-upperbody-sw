"""T10: modelled mass, reported by owner. FAIL only if the modelled total exceeds `gross_mass_limit`.

It REPORTS what the description models; it does not claim the robot weighs that. The battery, hub motors, electronics
and the mast structure are not modelled (all F2S), so the total is a lower bound on nothing in particular and says
nothing about the gross mass of the machine. The report says so in words.

Owner of a link = the `owner` of its frame_contract.yaml entry, with our own links split into mast and sensors by name.
A massed link the contract does not list (or an owner this file does not know) is `unclassified`: still counted in the
total, and a WARN, so a new link cannot hide from the report.
"""
import fnmatch
import os
import warnings
import xml.etree.ElementTree as ET

from conftest import MATRIX, SHARE, expand, load_datums, load_yaml, write_sidecar

CONTRACT = load_yaml(os.path.join(SHARE, "config", "frame_contract.yaml"))
CONTRACT_OWNER = {e["name"]: e["owner"] for e in CONTRACT["entries"] if e["kind"] == "link"}

GROUPS = [  # key, label, what the number is
    ("base", "base (platform-sw)", "CAD export, placeholder"),
    ("mast", "mast (ours)", "placeholder box"),
    ("openarm", "openarm_* (upstream)", "nominal, arms + grippers, no body (ADR-008)"),
    ("sensors", "sensors (ours)", "upper-sensor frames carry no <inertial> by design"),
    ("unclassified", "unclassified", "massed links not in the frame contract"),
]
NOT_MODELLED = ["battery", "hub motors", "electronics", "the mast structure itself"]
SENSOR_LINKS = ["head_camera_*", "*_wrist_camera_*"]


def owner_group(link_name, contract_owner=None):
    owner = CONTRACT_OWNER.get(link_name) if contract_owner is None else contract_owner
    if owner is None:
        return "unclassified"
    if owner == "platform-sw":
        return "base"
    if owner.startswith("upstream:"):
        return "openarm"
    if owner.startswith("A6"):
        if fnmatch.fnmatchcase(link_name, "mast_*"):
            return "mast"
        if any(fnmatch.fnmatchcase(link_name, pat) for pat in SENSOR_LINKS):
            return "sensors"
    return "unclassified"


def mass_by_group(root):
    """({group: kg}, {group: [link names]}) over every link with an <inertial>."""
    kg = {key: 0.0 for key, _, _ in GROUPS}
    links = {key: [] for key, _, _ in GROUPS}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        group = owner_group(link.get("name"))
        kg[group] += float(inertial.find("mass").get("value"))
        links[group].append(link.get("name"))
    return kg, links


def report_lines(label, kg, limit, base_link_mass):
    total = sum(kg.values())
    lines = [f"Modelled mass at {label}"]
    for key, name, what in GROUPS:
        if key == "unclassified" and not kg[key]:
            continue
        lines.append(f"  {name:<22} {kg[key]:>7.2f} kg   {what}")
    lines += [f"  {'modelled total':<22} {total:>7.2f} kg   vs gross_mass_limit {limit:g} kg",
              "Not modelled (all F2S): " + ", ".join(NOT_MODELLED) + ".",
              "  The mast row above is a placeholder box, not the structure. The total is therefore what the",
              "  description contains, not what the robot weighs.",
              f"  W01: base_link's inertia ({base_link_mass:.2f} kg of the base figure) is not seen by KDL consumers."]
    return lines


def _limit():
    datum = load_datums()["datums"]["gross_mass_limit"]
    return float(datum["value"])


def test_t10_mass_report_and_gross_mass_limit():
    limit = _limit()
    release = expand()                                             # mast_1350 / openarm / mock
    kg, links = mass_by_group(release)
    base_link = float(release.find("link[@name='base_link']/inertial/mass").get("value"))
    lines = report_lines("mast_1350 / openarm", kg, limit, base_link)

    totals, exceeded = {}, []
    for mast_id, arm_package in MATRIX:
        group_kg, _ = mass_by_group(expand(mast_id=mast_id, arm_package=arm_package))
        totals[f"{mast_id}/{arm_package}"] = round(sum(group_kg.values()), 3)
        if sum(group_kg.values()) > limit:
            exceeded.append(f"{mast_id}/{arm_package}: {sum(group_kg.values()):.2f} kg > {limit:g} kg")

    unclassified = links["unclassified"]
    result = "FAIL" if exceeded else ("WARN" if unclassified else "PASS")
    write_sidecar("T10", "mass_report", "modelled mass by owner vs gross_mass_limit", result, {
        "report_expansion": "mast_1350/openarm/mock",
        "kg_by_owner": {key: round(value, 3) for key, value in kg.items() if value or key != "unclassified"},
        "links_by_owner": {key: sorted(names) for key, names in links.items() if names},
        "modelled_total_kg": round(sum(kg.values()), 3),
        "gross_mass_limit_kg": limit,
        "modelled_total_kg_by_expansion": totals,
        "not_modelled": NOT_MODELLED,
        "base_link_inertia_not_seen_by_kdl": {"id": "W01", "base_link_mass_kg": round(base_link, 3)},
        "report": lines,
    })
    print("\n" + "\n".join(lines))
    if unclassified:
        warnings.warn(f"T10 WARN: massed links with no owner in frame_contract.yaml: {', '.join(unclassified)}",
                      UserWarning, stacklevel=1)
    assert not exceeded, "\n" + "\n".join(exceeded)


def test_t10_every_massed_link_in_the_matrix_has_an_owner_group():
    unclassified = set()
    for mast_id, arm_package in MATRIX:
        unclassified |= set(mass_by_group(expand(mast_id=mast_id, arm_package=arm_package))[1]["unclassified"])
    assert not unclassified, f"massed links with no owner in frame_contract.yaml: {sorted(unclassified)}"


def test_t10_the_groups_partition_the_total():
    root = expand()
    kg, links = mass_by_group(root)
    every = [l.get("name") for l in root.findall("link") if l.find("inertial") is not None]
    assert sorted(n for names in links.values() for n in names) == sorted(every)
    assert abs(sum(kg.values()) - sum(float(m.get("value")) for m in root.iter("mass"))) < 1e-9


def test_t10_the_upstream_arms_are_the_known_nominal_mass():
    # ADR-008 / S1: arms + grippers 12.26 kg from the upstream xacro; this is what "upstream nominal" means
    kg, _ = mass_by_group(expand())
    assert abs(kg["openarm"] - 12.26) < 0.01


# --- the limit and the grouping on synthetic input: they must be able to fail ------------------------------------------

def _urdf(*masses):
    return ET.fromstring("<robot>" + "".join(
        f"<link name='{n}'><inertial><mass value='{m}'/></inertial></link>" for n, m in masses) + "</robot>")


def test_t10_grouping_follows_the_contract_owner_and_splits_our_links_by_name():
    assert owner_group("base_link") == "base"
    assert owner_group("openarm_left_link1") == "openarm"
    assert owner_group("mast_base_link") == "mast"
    assert owner_group("head_camera_link") == "sensors"
    assert owner_group("left_wrist_camera_link") == "sensors"
    assert owner_group("some_new_link") == "unclassified"                        # not in the contract
    assert owner_group("odd_link", contract_owner="A6 (openamr-upperbody-sw)") == "unclassified"
    assert owner_group("odd_link", contract_owner="somebody else") == "unclassified"


def test_t10_a_total_over_the_limit_is_what_fails():
    kg, _ = mass_by_group(_urdf(("base_link", 100.0), ("mast_base_link", 25.0)))
    assert sum(kg.values()) > 120.0
    kg, _ = mass_by_group(_urdf(("base_link", 100.0), ("mast_base_link", 19.0)))
    assert not sum(kg.values()) > 120.0
    kg, links = mass_by_group(_urdf(("mystery", 1.0)))
    assert kg["unclassified"] == 1.0 and links["unclassified"] == ["mystery"]
