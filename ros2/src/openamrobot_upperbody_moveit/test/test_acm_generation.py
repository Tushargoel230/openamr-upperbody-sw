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
