"""T10: modelled mass, reported by owner. FAIL only if the modelled total exceeds `gross_mass_limit`.

It REPORTS what the description models; it does not claim the robot weighs that. The battery, hub motors, electronics
and the mast structure are not modelled (all F2S), so the total is a lower bound on nothing in particular and says
nothing about the gross mass of the machine. The report says so in words.

Owner of a link = the `owner` of its frame_contract.yaml entry, with our own links split into mast and sensors by name.
A massed link the contract does not list (or an owner this file does not know) is `unclassified`: still counted in the
total, and a WARN, so a new link cannot hide from the report.

After the total the report also prints the `gross_mass_limit` it compared against, the F2S vehicle estimate, and
"per-link masses still placeholders" if and only if some group is `placeholder` in config/mass_provenance.yaml, plus a
separate line naming the `external_nominal` groups. The map has three states over all four groups (placeholder,
external_nominal, verified); it replaces P09's tripwire, which only guarded the absence of such a map. Where a group draws its
mass from a datum (the mast, `mast_mass`) the map must agree with the datum's status, so the clause cannot go quiet over a
group that is still a placeholder, and disappears by itself on the day every group is real.
"""
import fnmatch
import os
import warnings
import xml.etree.ElementTree as ET

from conftest import MATRIX, SHARE, expand, load_datums, load_yaml, write_sidecar

CONTRACT = load_yaml(os.path.join(SHARE, "config", "frame_contract.yaml"))
CONTRACT_OWNER = {e["name"]: e["owner"] for e in CONTRACT["entries"] if e["kind"] == "link"}

GROUPS = [  # key, label, what the number is (its provenance STATE is read from config/mass_provenance.yaml, not written here)
    ("base", "base (platform-sw)", "platform-sw CAD export"),
    ("mast", "mast (ours)", "box from the mast_mass datum"),
    ("openarm", "openarm_* (upstream)", "upstream nominal, arms + grippers, no body (ADR-008)"),
    ("sensors", "sensors (ours)", "upper-sensor frames carry no <inertial> by design"),
    ("unclassified", "unclassified", "massed links not in the frame contract"),
]
MASS_GROUPS = ("base", "mast", "openarm", "sensors")          # the four groups the provenance map must cover
NOT_MODELLED = ["battery", "hub motors", "electronics", "the mast structure itself"]
SENSOR_LINKS = ["head_camera_*", "*_wrist_camera_*"]
# Group -> the datums whose value is a link mass in that group (mast.xacro: mast_m). A group draws a mass from a datum
# only if it is listed here and its provenance entry names the datum; test_t10_mass_datums_are_the_ones_the_xacro_reads
# keeps the list honest.
MASS_DATUMS = {"mast": ["mast_mass"]}
PROVENANCE_FILE = os.path.join(SHARE, "config", "mass_provenance.yaml")
PROVENANCE_STATES = ("placeholder", "external_nominal", "verified")
F2S_ESTIMATE = "f2s_vehicle_mass_estimate"
PLACEHOLDER_CLAUSE = "per-link masses still placeholders"


def load_provenance():
    """{group: entry} from config/mass_provenance.yaml (a fresh parse; tests may mutate it)."""
    return load_yaml(PROVENANCE_FILE)["groups"]


def groups_in_state(provenance, state):
    return [group for group in MASS_GROUPS if provenance[group]["state"] == state]


def provenance_problems(provenance, datums):
    """What is wrong with a provenance map, as messages (empty: nothing). Three states over exactly the four mass groups; a
    `verified` group names its document and a group in another state does not claim one; and a group that draws its mass from a
    datum agrees with that datum's status, in both directions that matter: a `placeholder` datum is a placeholder group (the
    caveat may not go quiet over it) and a `verified` datum is a verified group (a stale map may not keep a caveat that is
    no longer true), while a `provisional` datum can only be an external figure."""
    problems = []
    if set(provenance) != set(MASS_GROUPS):
        problems.append(f"groups {sorted(provenance)} are not exactly {sorted(MASS_GROUPS)}")
    for group, entry in provenance.items():
        state = entry.get("state")
        if state not in PROVENANCE_STATES:
            problems.append(f"{group}: state {state!r} is not one of {', '.join(PROVENANCE_STATES)}")
        if not str(entry.get("source") or "").strip():
            problems.append(f"{group}: no `source` saying where the figure comes from")
        if state == "verified" and not str(entry.get("document") or "").strip():
            problems.append(f"{group}: `verified` needs a `document` (measured, with a document)")
        if state in ("placeholder", "external_nominal") and entry.get("document"):
            problems.append(f"{group}: a {state} group cannot name a `document`; that is what `verified` means")
        datum = entry.get("datum")
        if datum:
            status = datums[datum]["status"]
            expected = {"placeholder": ("placeholder",), "provisional": ("external_nominal",), "verified": ("verified",)}[status]
            if state not in expected:
                problems.append(f"{group}: datum {datum} has status {status} but the group's state is {state} "
                                f"(expected {' or '.join(expected)})")
    return problems


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


def report_lines(label, kg, links, datums, base_link_mass, provenance):
    """The T10 report. `datums` is the parsed datums.yaml `datums` map and `provenance` the parsed mass_provenance.yaml `groups`:
    the limit, the F2S estimate, their sources, each row's provenance state and the placeholder clause all come from them."""
    total = sum(kg.values())
    limit, f2s = datums["gross_mass_limit"], datums[F2S_ESTIMATE]
    placeholders, external = groups_in_state(provenance, "placeholder"), groups_in_state(provenance, "external_nominal")
    lines = [f"Modelled mass at {label}"]
    for key, name, what in GROUPS:
        if key == "unclassified" and not kg[key]:
            continue
        state = f"[{provenance[key]['state']}]" if key in provenance else ""
        lines.append(f"  {name:<22} {kg[key]:>7.2f} kg   {state:<18} {what}")
    lines += [f"  {'modelled total':<22} {total:>7.2f} kg   vs gross_mass_limit {limit['value']:g} kg",
              f"  gross_mass_limit {limit['value']:g} kg [{limit['source']}, {limit['status']}]: {limit['ref']}",
              "  (F2S vehicle estimate " + f"{f2s['value']:g} kg" + (f" \u2014 {PLACEHOLDER_CLAUSE}" if placeholders else "") + ")",
              f"  F2S vehicle estimate [{f2s['source']}, {f2s['status']}]: {f2s['ref']}"]
    if placeholders:
        lines.append(f"  placeholder groups (no source, chosen so the model expands): {', '.join(placeholders)}")
    if external:
        lines.append(f"  external_nominal groups (another owner's figure, not verified by us): {', '.join(external)}")
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
    provenance = load_provenance()
    lines = report_lines("mast_1350 / openarm", kg, links, datums, base_link, provenance)
    problems = provenance_problems(provenance, datums)

    totals, exceeded = {}, []
    for mast_id, arm_package in MATRIX:
        group_kg, _ = mass_by_group(expand(mast_id=mast_id, arm_package=arm_package))
        totals[f"{mast_id}/{arm_package}"] = round(sum(group_kg.values()), 3)
        if sum(group_kg.values()) > limit:
            exceeded.append(f"{mast_id}/{arm_package}: {sum(group_kg.values()):.2f} kg > {limit:g} kg")

    unclassified = links["unclassified"]
    result = "FAIL" if exceeded or problems else ("WARN" if unclassified else "PASS")
    write_sidecar("T10", "mass_report", "modelled mass by owner vs gross_mass_limit", result, {
        "report_expansion": "mast_1350/openarm/mock",
        "kg_by_owner": {key: round(value, 3) for key, value in kg.items() if value or key != "unclassified"},
        "links_by_owner": {key: sorted(names) for key, names in links.items() if names},
        "modelled_total_kg": round(sum(kg.values()), 3),
        "gross_mass_limit_kg": limit,
        "gross_mass_limit_source": {key: datums["gross_mass_limit"][key] for key in ("source", "status", "ref")},
        "f2s_vehicle_estimate_kg": float(datums[F2S_ESTIMATE]["value"]),
        "f2s_vehicle_estimate_source": {key: datums[F2S_ESTIMATE][key] for key in ("source", "status", "ref")},
        "mass_provenance": {group: {"state": entry["state"], "source": entry["source"]} for group, entry in provenance.items()},
        "per_link_masses_still_placeholders": bool(groups_in_state(provenance, "placeholder")),
        "placeholder_groups": groups_in_state(provenance, "placeholder"),
        "external_nominal_groups": groups_in_state(provenance, "external_nominal"),
        "verified_groups": groups_in_state(provenance, "verified"),
        "provenance_problems": problems,
        "modelled_total_kg_by_expansion": totals,
        "not_modelled": NOT_MODELLED,
        "base_link_inertia_not_seen_by_kdl": {"id": "W01", "base_link_mass_kg": round(base_link, 3)},
        "report": lines,
    })
    print("\n" + "\n".join(lines))
    if unclassified:
        warnings.warn(f"T10 WARN: massed links with no owner in frame_contract.yaml: {', '.join(unclassified)}",
                      UserWarning, stacklevel=1)
    assert not problems, "\n" + "\n".join(problems)
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

def _report(datums=None, provenance=None):
    datums = datums or load_datums()["datums"]
    provenance = provenance or load_provenance()
    kg, links = mass_by_group(expand())
    return report_lines("mast_1350 / openarm", kg, links, datums, 4.18, provenance), datums


def _provenance(**states):
    """The real map with some groups' states replaced; `verified` gets a document so the map stays valid."""
    provenance = load_provenance()
    for group, state in states.items():
        provenance[group]["state"] = state
        provenance[group].pop("document", None)
        if state == "verified":
            provenance[group]["document"] = "weighing record no. X, rev 2"
    return provenance


def _datums_with_mast_mass(status):
    datums = load_datums()["datums"]
    datums["mast_mass"]["status"] = status
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


# --- the provenance map: three states over four groups, and the caveat reads off it ---------------------------------------

def test_t10_mass_provenance_the_real_map_is_valid_and_says_what_the_design_says():
    provenance, datums = load_provenance(), load_datums()["datums"]
    assert provenance_problems(provenance, datums) == []
    assert {g: provenance[g]["state"] for g in MASS_GROUPS} == {
        "base": "external_nominal", "mast": "placeholder", "openarm": "external_nominal", "sensors": "placeholder"}
    assert not groups_in_state(provenance, "verified"), "nothing is measured yet; a verified group needs its document first"


def test_t10_with_the_current_tree_the_caveat_and_the_external_nominal_line_appear():
    lines, _ = _report()
    assert "  (F2S vehicle estimate 93 kg \u2014 per-link masses still placeholders)" in lines
    assert any(line.startswith("  placeholder groups") and line.endswith("mast, sensors") for line in lines)
    assert any(line.startswith("  external_nominal groups") and line.endswith("base, openarm") for line in lines)
    row = {name: line for line in lines for _, name, _ in GROUPS if line.startswith(f"  {name} ")}
    assert "[external_nominal]" in row["base (platform-sw)"] and "[placeholder]" in row["mast (ours)"]
    assert "[external_nominal]" in row["openarm_* (upstream)"] and "[placeholder]" in row["sensors (ours)"]


def test_t10_the_caveat_is_present_when_any_one_group_is_a_placeholder_and_absent_when_none_is():
    for lone in MASS_GROUPS:
        states = {g: "external_nominal" for g in MASS_GROUPS}
        states[lone] = "placeholder"
        lines = _report(provenance=_provenance(**states))[0]
        assert PLACEHOLDER_CLAUSE in "\n".join(lines), f"{lone} alone is a placeholder and the caveat must stay"
        assert any(line.endswith(f"expands): {lone}") for line in lines)
    quiet = "\n".join(_report(provenance=_provenance(**{g: "external_nominal" for g in MASS_GROUPS}))[0])
    assert PLACEHOLDER_CLAUSE not in quiet and "placeholder groups" not in quiet
    assert "  (F2S vehicle estimate 93 kg)" in quiet                   # the estimate itself is still printed


def test_t10_the_external_nominal_line_is_its_own_line_and_follows_the_map():
    all_verified = "\n".join(_report(provenance=_provenance(**{g: "verified" for g in MASS_GROUPS}))[0])
    assert PLACEHOLDER_CLAUSE not in all_verified and "external_nominal groups" not in all_verified
    only_arms = _report(provenance=_provenance(base="verified", mast="verified", sensors="verified"))[0]
    assert any(line.endswith("not verified by us): openarm") for line in only_arms)
    assert PLACEHOLDER_CLAUSE not in "\n".join(only_arms)


def test_t10_the_caveat_follows_the_map_back_and_forth():
    states = {g: "verified" for g in MASS_GROUPS}
    assert PLACEHOLDER_CLAUSE not in "\n".join(_report(provenance=_provenance(**states))[0])
    states["sensors"] = "placeholder"
    assert PLACEHOLDER_CLAUSE in "\n".join(_report(provenance=_provenance(**states))[0])
    states["sensors"] = "external_nominal"                                # only `placeholder` triggers it
    assert PLACEHOLDER_CLAUSE not in "\n".join(_report(provenance=_provenance(**states))[0])


def test_t10_a_map_that_is_not_three_states_over_the_four_groups_is_a_problem():
    datums = load_datums()["datums"]
    provenance = load_provenance()
    del provenance["sensors"]
    assert any("not exactly" in m for m in provenance_problems(provenance, datums))
    provenance = load_provenance()
    provenance["base"]["state"] = "provisional"                           # a datum status is not a provenance state
    assert any("base: state 'provisional'" in m for m in provenance_problems(provenance, datums))
    provenance = load_provenance()
    provenance["openarm"]["source"] = ""
    assert any("openarm: no `source`" in m for m in provenance_problems(provenance, datums))


def test_t10_verified_needs_a_document_and_nothing_else_may_claim_one():
    datums = _datums_with_mast_mass("verified")
    provenance = _provenance(mast="verified")
    assert provenance_problems(provenance, datums) == []
    del provenance["mast"]["document"]
    assert any("mast: `verified` needs a `document`" in m for m in provenance_problems(provenance, datums))
    provenance = load_provenance()
    provenance["base"]["document"] = "a drawing"
    assert any("base: a external_nominal group cannot name a `document`" in m or "base: a external_nominal" in m
               for m in provenance_problems(provenance, load_datums()["datums"]))


def test_t10_the_map_has_to_agree_with_the_datum_a_group_draws_its_mass_from():
    """P09's tripwire, in its place: the day mast_mass becomes verified the map cannot be left saying `placeholder` (the caveat
    would outlive its cause), and a map that says `verified` over a placeholder datum is the lie that would silence it."""
    assert provenance_problems(load_provenance(), _datums_with_mast_mass("placeholder")) == []
    stale = provenance_problems(load_provenance(), _datums_with_mast_mass("verified"))
    assert stale and "datum mast_mass has status verified but the group's state is placeholder" in stale[0]
    over_claimed = provenance_problems(_provenance(mast="verified"), _datums_with_mast_mass("placeholder"))
    assert over_claimed and "has status placeholder but the group's state is verified" in over_claimed[0]
    assert provenance_problems(_provenance(mast="external_nominal"), _datums_with_mast_mass("provisional")) == []
    assert provenance_problems(load_provenance(), _datums_with_mast_mass("provisional"))        # still `placeholder` over a provisional datum
    assert provenance_problems(_provenance(mast="verified"), _datums_with_mast_mass("provisional"))


def test_t10_mass_datums_are_the_ones_the_xacro_reads():
    # MASS_DATUMS is the one hand-kept list the map's datum link depends on: every entry must be a real datum that the xacro reads
    # for a link mass, and the map must name it. A datum renamed or dropped from mast.xacro fails here instead of silently
    # breaking the agreement check.
    from test_t13_datum_references import referenced_in_package
    datums, referenced, provenance = load_datums()["datums"], referenced_in_package(), load_provenance()
    for group, names in MASS_DATUMS.items():
        assert group in {key for key, _, _ in GROUPS}
        assert [provenance[group].get("datum")] == names, f"{group}'s provenance entry must name the datum {names}"
        for name in names:
            assert name in datums and name in referenced, f"{group}: {name} is not a datum the xacro reads"
    assert {g for g, entry in provenance.items() if entry.get("datum")} == set(MASS_DATUMS)
    root = expand()
    kg, links = mass_by_group(root)
    mast_mass = float(datums["mast_mass"]["value"])
    assert any(abs(float(l.find("inertial/mass").get("value")) - mast_mass) < 1e-9
               for l in root.findall("link") if l.get("name") in links["mast"]), "no mast link carries mast_mass"
