"""T10: modelled mass, reported by owner. FAIL only if the modelled total exceeds `gross_mass_limit`.

It REPORTS what the description models; it does not claim the robot weighs that. The battery, hub motors, electronics
and the mast structure are not modelled (all F2S), so the total is a lower bound on nothing in particular and says
nothing about the gross mass of the machine. The report says so in words.

Owner of a link = the `owner` of its frame_contract.yaml entry, with our own links split into mast and sensors by name.
A massed link the contract does not list (or an owner this file does not know) is `unclassified`: still counted in the
total, and a WARN, so a new link cannot hide from the report.

After the total the report also prints the `gross_mass_limit` it compared against, the F2S vehicle estimate, and
"per-link masses still placeholders" if and only if a reported group draws a mass from a datum whose `status` is
`placeholder` (MASS_DATUMS says which datums those are). The clause is computed from datums.yaml on every run, so it
disappears by itself when real masses land. It is a statement about datum status only: the base and arm masses are not
datums (platform-sw CAD export, upstream nominal) and their rows say what they are in words.
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
# Group -> the datums whose value is a link mass in that group (mast.xacro: mast_m). A group draws a mass from a datum
# only if it is listed here; test_t10_mass_datums_are_the_ones_the_xacro_reads keeps the list honest.
MASS_DATUMS = {"mast": ["mast_mass"]}
F2S_ESTIMATE = "f2s_vehicle_mass_estimate"
PLACEHOLDER_CLAUSE = "per-link masses still placeholders"


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


def placeholder_mass_datums(datums, links):
    """Sorted names of the mass datums, among the groups that have massed links, whose status is `placeholder`."""
    return sorted({name for group, names in MASS_DATUMS.items() if links.get(group) for name in names
                   if datums[name]["status"] == "placeholder"})


def report_lines(label, kg, links, datums, base_link_mass):
    """The T10 report. `datums` is the parsed datums.yaml `datums` map; the limit, the F2S estimate, their sources and
    the placeholder clause all come from it."""
    total = sum(kg.values())
    limit, f2s = datums["gross_mass_limit"], datums[F2S_ESTIMATE]
    placeholders = placeholder_mass_datums(datums, links)
    lines = [f"Modelled mass at {label}"]
    for key, name, what in GROUPS:
        if key == "unclassified" and not kg[key]:
            continue
        lines.append(f"  {name:<22} {kg[key]:>7.2f} kg   {what}")
    lines += [f"  {'modelled total':<22} {total:>7.2f} kg   vs gross_mass_limit {limit['value']:g} kg",
              f"  gross_mass_limit {limit['value']:g} kg [{limit['source']}, {limit['status']}]: {limit['ref']}",
              "  (F2S vehicle estimate " + f"{f2s['value']:g} kg" + (f" — {PLACEHOLDER_CLAUSE}" if placeholders else "") + ")",
              f"  F2S vehicle estimate [{f2s['source']}, {f2s['status']}]: {f2s['ref']}"]
    if placeholders:
        lines.append(f"  mass datums with status placeholder: {', '.join(placeholders)}")
    lines += ["Not modelled (all F2S): " + ", ".join(NOT_MODELLED) + ".",
              "  The mast row above is a placeholder box, not the structure. The total is therefore what the",
              "  description contains, not what the robot weighs.",
              f"  W01: base_link's inertia ({base_link_mass:.2f} kg of the base figure) is not seen by KDL consumers."]
    return lines


def test_t10_mass_report_and_gross_mass_limit():
    datums = load_datums()["datums"]
    limit = float(datums["gross_mass_limit"]["value"])
    release = expand()                                             # mast_1350 / openarm / mock
    kg, links = mass_by_group(release)
    base_link = float(release.find("link[@name='base_link']/inertial/mass").get("value"))
    lines = report_lines("mast_1350 / openarm", kg, links, datums, base_link)
    placeholders = placeholder_mass_datums(datums, links)

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
        "gross_mass_limit_source": {key: datums["gross_mass_limit"][key] for key in ("source", "status", "ref")},
        "f2s_vehicle_estimate_kg": float(datums[F2S_ESTIMATE]["value"]),
        "f2s_vehicle_estimate_source": {key: datums[F2S_ESTIMATE][key] for key in ("source", "status", "ref")},
        "per_link_masses_still_placeholders": bool(placeholders),
        "placeholder_mass_datums": placeholders,
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


# --- the limit, the F2S estimate and the placeholder clause --------------------------------------------------------

def _report(datums=None):
    datums = datums or load_datums()["datums"]
    kg, links = mass_by_group(expand())
    return report_lines("mast_1350 / openarm", kg, links, datums, 4.18), datums


def _all_mass_datums_verified():
    datums = load_datums()["datums"]
    for names in MASS_DATUMS.values():
        for name in names:
            datums[name].update(source="f2s", status="verified", ref="drawing no. X, rev 2")
    return datums


def test_t10_prints_the_limit_with_its_source_and_the_f2s_estimate_with_its_source():
    lines, datums = _report()
    text = "\n".join(lines)
    assert "vs gross_mass_limit 120 kg" in text
    limit = datums["gross_mass_limit"]
    assert f"gross_mass_limit 120 kg [{limit['source']}, {limit['status']}]: {limit['ref']}" in text
    f2s = datums[F2S_ESTIMATE]
    assert f2s["value"] == 93.0 and f2s["source"] == "f2s"          # the datum, not a constant in the reporter
    assert f"(F2S vehicle estimate 93 kg" in text
    assert f"F2S vehicle estimate [{f2s['source']}, {f2s['status']}]: {f2s['ref']}" in text


def test_t10_the_f2s_estimate_is_what_datums_yaml_says_not_a_constant_in_the_reporter():
    datums = load_datums()["datums"]
    datums[F2S_ESTIMATE].update(value=87.5, ref="a different document")
    text = "\n".join(_report(datums)[0])
    assert "(F2S vehicle estimate 87.5 kg" in text and "a different document" in text and "93 kg" not in text


def test_t10_with_the_current_tree_the_placeholder_clause_appears():
    lines, datums = _report()
    assert datums["mast_mass"]["status"] == "placeholder"
    assert "  (F2S vehicle estimate 93 kg \u2014 per-link masses still placeholders)" in lines
    assert any("mass datums with status placeholder: mast_mass" in line for line in lines)


def test_t10_with_every_mass_datum_verified_the_placeholder_clause_does_not_appear():
    lines, datums = _report(_all_mass_datums_verified())
    text = "\n".join(lines)
    assert PLACEHOLDER_CLAUSE not in text and "mass datums with status placeholder" not in text
    assert "  (F2S vehicle estimate 93 kg)" in lines                  # the estimate itself is still printed


def test_t10_the_clause_follows_the_datum_status_back_and_forth():
    datums = _all_mass_datums_verified()
    assert PLACEHOLDER_CLAUSE not in "\n".join(_report(datums)[0])
    datums["mast_mass"]["status"] = "placeholder"
    assert PLACEHOLDER_CLAUSE in "\n".join(_report(datums)[0])
    datums["mast_mass"]["status"] = "provisional"                    # only `placeholder` triggers it
    assert PLACEHOLDER_CLAUSE not in "\n".join(_report(datums)[0])


def test_t10_a_placeholder_datum_in_a_group_with_no_massed_links_does_not_trigger_the_clause():
    datums = load_datums()["datums"]
    assert placeholder_mass_datums(datums, {"mast": []}) == []
    assert placeholder_mass_datums(datums, {"mast": ["mast_base_link"]}) == ["mast_mass"]


def test_t10_mass_datums_are_the_ones_the_xacro_reads():
    # MASS_DATUMS is the one hand-kept list the clause depends on: every entry must be a real datum that the xacro reads
    # for a link mass. A datum renamed or dropped from mast.xacro fails here instead of silently turning the clause off.
    from test_t13_datum_references import referenced_in_package
    datums, referenced = load_datums()["datums"], referenced_in_package()
    for group, names in MASS_DATUMS.items():
        assert group in {key for key, _, _ in GROUPS}
        for name in names:
            assert name in datums and name in referenced, f"{group}: {name} is not a datum the xacro reads"
    root = expand()
    kg, links = mass_by_group(root)
    mast_mass = float(datums["mast_mass"]["value"])
    assert any(abs(float(l.find("inertial/mass").get("value")) - mast_mass) < 1e-9
               for l in root.findall("link") if l.get("name") in links["mast"]), "no mast link carries mast_mass"
