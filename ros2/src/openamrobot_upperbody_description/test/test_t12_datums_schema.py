"""T12: datums.yaml validates against datums.schema.json, and the schema rejects what it must reject."""
import os

import pytest
from jsonschema import Draft202012Validator

from conftest import FIXTURES, load_datums, load_schema, load_yaml


def _errors(doc):
    return list(Draft202012Validator(load_schema()).iter_errors(doc))


def _failing_datums(doc):
    """Names of datums that have at least one schema error."""
    out = set()
    for e in _errors(doc):
        path = list(e.absolute_path)
        if len(path) >= 2 and path[0] == "datums":
            out.add(path[1])
    return out


def test_t12_schema_is_a_valid_2020_12_schema():
    schema = load_schema()
    assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    Draft202012Validator.check_schema(schema)


def test_t12_datums_yaml_validates():
    errors = _errors(load_datums())
    assert not errors, "\n".join(f"{'/'.join(map(str, e.absolute_path))}: {e.message}" for e in errors)


def test_t12_schema_allows_report_only_and_pending_reference():
    doc = load_datums()
    assert "report_only" in doc and "pending_reference" in doc, "datums.yaml should carry both top-level maps"
    assert not _errors(doc)
    # and neither is mandatory
    for key in ("report_only", "pending_reference"):
        trimmed = load_datums()
        del trimmed[key]
        assert not _errors(trimmed), f"{key} must be optional"


def test_t12_broken_fixture_is_rejected():
    doc = load_yaml(os.path.join(FIXTURES, "datums_broken.yaml"))
    errors = _errors(doc)
    assert errors, "datums_broken.yaml validated, so the schema is not enforcing anything"
    # ... and for the reason each datum was written to break, not for some accidental one
    assert _failing_datums(doc) == set(doc["datums"]), (
        f"expected every datum in the fixture to fail, got {sorted(_failing_datums(doc))}")
    assert any(list(e.absolute_path)[:1] == ["pending_reference"] for e in errors), "bad pending step not caught"


# One mutation per rule, applied to a copy of the real file, so each rule is shown to bite on its own.
def _verified_derived(d):
    d["datums"]["mast_mass"].update(source="derived", status="verified")


def _verified_upstream(d):
    d["datums"]["mast_mass"].update(source="upstream", status="verified")


def _verified_placeholder_source(d):
    d["datums"]["mast_mass"].update(source="placeholder", status="verified")


def _verified_empty_ref(d):
    d["datums"]["gross_mass_limit"]["ref"] = ""


def _verified_blank_ref(d):
    d["datums"]["gross_mass_limit"]["ref"] = "   "


def _verified_missing_ref(d):
    del d["datums"]["gross_mass_limit"]["ref"]


def _placeholder_source_provisional(d):
    d["datums"]["mast_mass"]["status"] = "provisional"


def _placeholder_source_verified_with_ref(d):
    d["datums"]["mast_mass"]["status"] = "verified"


def _bad_source(d):
    d["datums"]["mast_mass"]["source"] = "guessed"


def _bad_status(d):
    d["datums"]["mast_mass"]["status"] = "approved"


def _missing(field):
    def mutate(d):
        del d["datums"]["mast_mass"][field]
    mutate.__name__ = f"missing_{field}"
    return mutate


def _unknown_field(d):
    d["datums"]["mast_mass"]["verified_by"] = "me"


def _bad_pending_step(d):
    d["pending_reference"]["head_camera_pose"] = "later"


def _pending_not_a_map(d):
    d["pending_reference"] = ["head_camera_pose"]


def _report_only_duplicate(d):
    d["report_only"].append(d["report_only"][0])


def _unknown_top_level(d):
    d["report_also"] = []


def _pose_missing_rpy(d):
    d["datums"]["mast_base_pose"]["value"] = {"xyz": [0.0, 0.0, 0.2]}


def _bad_revision(d):
    d["revision"] = "v2"


MUTATIONS = [
    _verified_derived, _verified_upstream, _verified_placeholder_source, _verified_empty_ref, _verified_blank_ref,
    _verified_missing_ref, _placeholder_source_provisional, _placeholder_source_verified_with_ref, _bad_source,
    _bad_status, _missing("value"), _missing("source"), _missing("status"), _missing("ref"), _missing("owner"),
    _unknown_field, _bad_pending_step, _pending_not_a_map, _report_only_duplicate, _unknown_top_level,
    _pose_missing_rpy, _bad_revision,
]


@pytest.mark.parametrize("mutate", MUTATIONS, ids=lambda m: m.__name__.strip("_"))
def test_t12_each_rule_rejects_its_violation(mutate):
    doc = load_datums()
    assert not _errors(doc)
    mutate(doc)
    assert _errors(doc)


@pytest.mark.parametrize("source", ["measured", "contractor_m01", "f2s", "supplier", "plan"])
def test_t12_verified_is_accepted_with_an_evidenced_source_and_a_ref(source):
    doc = load_datums()
    doc["datums"]["mast_mass"].update(source=source, status="verified", ref="drawing no. X, rev 2")
    assert not _errors(doc)

