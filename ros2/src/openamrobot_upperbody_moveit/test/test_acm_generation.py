"""The collision matrix's settings are on record and are the ones A7 asks for: at least 10000 samples at mast_1350 with the
"never in collision", "adjacent" and "default" categories on, and nothing in the SRDF that the record does not account for.

An ACM nobody can reproduce is not evidence: config/acm/acm_generation.yaml is written by scripts/merge_acm.py from the very run
that produced the pairs, and these tests read it back."""
import xml.etree.ElementTree as ET

from conftest import load_yaml, srdf_root, srdf_text

BEGIN, END = "<!-- BEGIN GENERATED ACM -->", "<!-- END GENERATED ACM -->"


def generated_pairs():
    block = srdf_text().split(BEGIN)[1].split(END)[0]
    root = ET.fromstring(f"<x>{block}</x>")
    return [(d.get("link1"), d.get("link2"), d.get("reason")) for d in root.findall("disable_collisions")]


def test_the_generation_settings_are_the_ones_the_prompt_asks_for():
    meta = load_yaml("acm/acm_generation.yaml")
    assert meta["tool"] == "moveit_setup_assistant collisions_updater"
    assert meta["trials"] >= 10000
    assert meta["mast_id"] == "mast_1350" and "mast_id:=mast_1350" in meta["xacro_args"]
    assert meta["categories"] == {"never_in_collision": True, "adjacent": True, "default": True, "always": False}
    assert "--default" in meta["command"] and "--always" not in meta["command"] and f"--trials {meta['trials']}" in meta["command"]
    assert meta["tool_statistics"]["always_in_collision"] == 0, "a pair that collides in every sample is a modelling question"


def test_the_srdf_holds_exactly_what_the_record_says_was_written():
    meta = load_yaml("acm/acm_generation.yaml")
    pairs = generated_pairs()
    assert len(pairs) == meta["written_total"]
    for reason, count in meta["written"].items():
        assert sum(1 for *_, r in pairs if r == reason) == count
    never = meta["written"]["Never"]
    assert never == meta["tool_statistics"]["never_in_collision"], "the tool's own count and what was merged must agree"
    assert len(srdf_root().findall("disable_collisions")) == meta["written_total"] + meta["imported_before_the_markers"]


def test_no_generated_pair_repeats_one_imported_before_the_markers():
    text = srdf_text()
    head = text.split(BEGIN)[0] + "</robot>"
    imported = {frozenset((d.get("link1"), d.get("link2"))) for d in ET.fromstring(head).findall("disable_collisions")}
    assert imported and not [p for p in generated_pairs() if frozenset(p[:2]) in imported]


# --- the checked-in matrix IS the contract (P11 task 3) -----------------------------------------------------------------------------------
def test_the_checked_in_matrix_is_the_one_whose_fingerprint_is_on_record():
    """collisions_updater has no seed, so regenerating gives a different matrix: the SRDF in the repository is the contract, and a change to its
    matrix must come with a new fingerprint in acm_generation.yaml, which is a visible change in a review."""
    from openamrobot_upperbody_moveit.acm_split import acm_fingerprint, acm_pairs
    contract = load_yaml("acm/acm_generation.yaml")["contract"]
    pairs = acm_pairs(srdf_root())
    assert acm_fingerprint(pairs) == contract["fingerprint"], \
        "the SRDF's collision matrix is not the one on record: either this is a deliberate change of the contract (regenerate_acm.sh --write, " \
        "or merge_acm.py --record-contract, and say so in the commit) or the matrix was edited by hand"
    assert contract["pairs"] == {"total": len(pairs), **{r: sum(1 for x in pairs.values() if x == r) for r in ("Never", "Adjacent", "Default")}}
    assert contract["file"] == "config/openamrobot.srdf"


def test_the_record_says_the_generation_is_not_seeded_and_what_that_means_for_regenerating():
    contract = load_yaml("acm/acm_generation.yaml")["contract"]
    assert contract["generation_is_seeded"] is False
    text = contract["regeneration"]
    assert "no --seed" in text and "not a verification step" in text and "validate_acm.py" in text


def test_the_fingerprint_follows_the_matrix_and_nothing_else():
    from openamrobot_upperbody_moveit.acm_split import acm_fingerprint, diff_acm
    base = {("a", "b"): "Never", ("a", "c"): "Adjacent"}
    assert acm_fingerprint(base) == acm_fingerprint(dict(reversed(list(base.items())))), "file order is not part of the matrix"
    assert acm_fingerprint(base) != acm_fingerprint({**base, ("b", "c"): "Never"})
    assert acm_fingerprint(base) != acm_fingerprint({**base, ("a", "b"): "Adjacent"})
    assert diff_acm(base, {**base, ("b", "c"): "Never", ("a", "b"): "Adjacent"}) == (
        {("b", "c"): "Never"}, {}, {("a", "b"): ("Never", "Adjacent")})


def test_merging_a_new_matrix_proposes_and_leaves_the_contract_and_its_record_alone(tmp_path):
    """The default of merge_acm.py (and so of regenerate_acm.sh) is a proposal: a re-run is not allowed to overwrite the contract silently."""
    import hashlib
    import importlib.util
    import os
    from conftest import SHARE
    spec = importlib.util.spec_from_file_location("merge_acm", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "merge_acm.py"))
    merge_acm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(merge_acm)
    srdf, meta = tmp_path / "openamrobot.srdf", tmp_path / "acm_generation.yaml"
    srdf.write_text(srdf_text())
    meta.write_text(open(os.path.join(SHARE, "config", "acm", "acm_generation.yaml")).read())
    generated = tmp_path / "tool_output.srdf"
    generated.write_text(srdf_text().replace('link1="openarm_left_link1" link2="openarm_left_link3" reason="Never"',
                                             'link1="openarm_left_link1" link2="openarm_left_link3" reason="Adjacent"'))
    proposal = tmp_path / "proposal.srdf"
    before = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in (srdf, meta)}
    common = ["--generated", str(generated), "--srdf", str(srdf), "--meta", str(meta), "--trials", "10000", "--mast-id", "mast_1350",
              "--xacro-args", "x", "--date", "2026-10-07", "--tool-command", "tool"]
    assert merge_acm.main([*common, "--proposal", str(proposal)]) == 0
    assert {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in (srdf, meta)} == before, "a proposal changed the contract"
    assert proposal.is_file() and proposal.read_text() != srdf.read_text()
    assert merge_acm.main([*common, "--write"]) == 0
    assert hashlib.sha256(srdf.read_bytes()).hexdigest() != before[srdf], "--write is how the contract is replaced"
    assert "contract" in meta.read_text() and meta.read_text() != open(os.path.join(SHARE, "config", "acm", "acm_generation.yaml")).read()
