"""C01: the configuration id (ARCHITECTURE.md s7) has the format the openamrobot-interfaces C9 contract requires, and
changes when, and only when, one of its inputs changes.

Format: `sha256:` + 64 lowercase hex. A 12-character truncation (the original s7) fails the C9 validator, so a test here
turns red if anyone brings it back. When openamrobot-interfaces is checked out in the workspace, the pattern is also read
from ITS schema (every field the C9 contract requires a configuration hash in) and the id is validated against that, so
a change on their side turns our test red instead of leaving us silently different. Without that checkout those two
tests SKIP with the reason, and C01's evidence says so. (That the package pin equals a present upstream checkout is C02.)
"""
import json
import os
import re
import subprocess
import sys

import jsonschema
import pytest

import evidence_common as ev
import print_config_id as pci
from conftest import MAST_IDS, SCRIPTS, SHARE, expand_xml, load_yaml, write_sidecar

WORKSPACE = ev.workspace_root(SHARE)
C9_SCHEMA = os.path.join(WORKSPACE, "src", "openamrobot-interfaces", "schemas", "json", "c9-lerobot-dataset.schema.json")
# where C9 requires a configuration hash: the schema path of each field, as a list of keys
C9_HASH_FIELDS = [
    ["$defs", "openAMRobot", "properties", "provenance", "properties", "robot_configuration_hash"],
    ["$defs", "cameraStream", "properties", "configuration_hash"],
    ["$defs", "stateStream", "properties", "configuration_hash"],
    ["$defs", "actionStream", "properties", "configuration_hash"],
]
COMMIT = "0123456789abcdef0123456789abcdef01234567"


def _commit():
    return ev.upstream_commit(WORKSPACE)


MATRIX = ev.expansion_matrix(MAST_IDS)


def _id(mast_id="mast_1350", arm_package="openarm", hardware_profile="mock", commit=COMMIT, **kw):
    return pci.config_id_of(SHARE, commit, mast_id, arm_package, hardware_profile, **kw)


# --- format -------------------------------------------------------------------------------------------------------

def test_c01_every_config_id_in_the_matrix_is_sha256_plus_64_lowercase_hex():
    commit = _commit()
    ids = {ev.expansion_label(*c): pci.config_id_of(SHARE, commit, *c) for c in MATRIX}
    assert len(ids) == 14
    bad = {k: v for k, v in ids.items() if not ev.SHA256_RE.match(v)}
    assert not bad, bad
    assert all(len(v) == len("sha256:") + 64 for v in ids.values())
    assert len(set(ids.values())) == len(ids), "two different expansions share an id"


def test_c01_the_pattern_itself_rejects_the_old_twelve_character_form():
    full = _id()
    assert ev.SHA256_RE.match(full)
    assert not ev.SHA256_RE.match(full[:7 + 12])                       # sha256:<12 hex>
    assert not ev.SHA256_RE.match(full.split(":")[1])                  # no prefix
    assert not ev.SHA256_RE.match(full.upper().replace("SHA256", "sha256"))   # uppercase hex


def _resolve(schema, ref):
    assert ref.startswith("#/"), f"only local refs are followed: {ref}"
    node = schema
    for part in ref[2:].split("/"):
        node = node[part]
    return node


def _c9_hash_definitions():
    """[(field path, resolved JSON-schema node)] for every field C9 requires a configuration hash in."""
    with open(C9_SCHEMA) as f:
        schema = json.load(f)
    out = []
    for path in C9_HASH_FIELDS:
        node = schema
        for key in path:
            node = node[key]
        while "$ref" in node:
            node = _resolve(schema, node["$ref"])
        out.append(("/".join(path), node))
    return out


needs_c9 = pytest.mark.skipif(not os.path.isfile(C9_SCHEMA),
                              reason=f"openamrobot-interfaces is not in the workspace ({C9_SCHEMA} missing)")


@needs_c9
def test_c01_the_id_validates_against_the_pattern_in_the_c9_schema():
    definitions = _c9_hash_definitions()
    assert len(definitions) == len(C9_HASH_FIELDS)
    cid = _id(commit=_commit())
    for path, node in definitions:
        jsonschema.validate(cid, node)                                 # type + pattern, as their validator sees it
        assert re.search(node["pattern"], cid), path


@needs_c9
def test_c01_the_c9_validator_would_reject_the_old_twelve_character_id():
    cid = _id()
    for path, node in _c9_hash_definitions():
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(cid[:7 + 12], node)


@needs_c9
def test_c01_our_pattern_is_the_one_c9_publishes():
    # if they change their definition this fails, and says which side moved
    patterns = {node["pattern"] for _, node in _c9_hash_definitions()}
    assert patterns == {ev.SHA256_RE.pattern}, f"C9 now requires {patterns}; evidence_common.SHA256_RE is {ev.SHA256_RE.pattern}"


# --- inputs ---------------------------------------------------------------------------------------------------------

def test_c01_the_same_inputs_give_the_same_id_and_each_input_changes_it():
    base = _id()
    assert _id() == base
    assert _id(mast_id="mast_1400") != base
    assert _id(arm_package="so101") != base
    assert _id(hardware_profile="gazebo") != base                      # an input in its own right, not only via the URDF
    assert _id(commit="f" * 40) != base                                # the pinned upstream commit
    assert _id(enable_planar_base="true") != base                      # through the URDF content


def test_c01_datums_revision_is_an_input(tmp_path):
    datums = load_yaml(os.path.join(SHARE, "config", "datums.yaml"))
    with open(os.path.join(SHARE, "config", "datums.yaml")) as f:
        text = f.read()
    original = _id()
    same = tmp_path / "same.yaml"
    same.write_text(text)
    assert _id(datums_file=str(same)) == original                       # same content, other path: same id
    bumped = tmp_path / "bumped.yaml"
    bumped.write_text(text.replace(f'revision: "{datums["revision"]}"', 'revision: "9999-01-01.z"'))
    assert bumped.read_text() != text
    assert _id(datums_file=str(bumped)) != original                     # only the revision differs, the numbers do not


def test_c01_comments_whitespace_and_the_autogenerated_path_do_not_change_the_id():
    urdf = expand_xml()
    args = ("mast_1350", "openarm", "mock", "rev", COMMIT)
    base = pci.config_id(urdf, *args)
    with_banner = urdf.replace("<robot ", "<!-- banner naming /some/other/path -->\n<robot ", 1)
    spaced = urdf.replace("><", ">\n   <")
    renamed = urdf.replace('<link name="base_link">', '<link name="base_link2">', 1)
    assert len({urdf, with_banner, spaced, renamed}) == 4                 # each edit really changed the text
    assert pci.config_id(with_banner, *args) == base
    assert pci.config_id(spaced, *args) == base
    assert pci.config_id(renamed, *args) != base                          # a content change is not normalised away


def test_c01_the_expanded_urdf_carries_no_machine_specific_path():
    # an absolute path inside the URDF would make the id differ between checkouts of the same commit
    canonical = pci.canonical_urdf(expand_xml())
    assert os.path.expanduser("~") not in canonical
    assert WORKSPACE not in canonical


def test_c01_serialisation_is_what_the_docstring_says():
    import hashlib
    urdf = expand_xml()
    members = {"canonical_urdf": pci.canonical_urdf(urdf), "mast_id": "mast_1350", "arm_package": "openarm",
               "hardware_profile": "mock", "datums_revision": "r", "openarm_description_commit": COMMIT}
    payload = json.dumps(members, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    assert pci.config_id(urdf, "mast_1350", "openarm", "mock", "r", COMMIT) == "sha256:" + hashlib.sha256(payload).hexdigest()


# --- the command ----------------------------------------------------------------------------------------------------

def _run(*args):
    return subprocess.run([sys.executable, os.path.join(SCRIPTS, "print_config_id.py"), *args], capture_output=True, text=True)


def test_c01_command_prints_exactly_the_full_id_and_matches_the_function():
    result = _run("mast_id:=mast_1350", "arm_package:=openarm", "hardware_profile:=mock", "--upstream-commit", COMMIT)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == _id()
    assert ev.SHA256_RE.match(result.stdout.strip()) and result.stdout.count("\n") == 1


def test_c01_command_labels_the_short_form_as_a_prefix():
    result = _run("mast_id:=mast_1350", "arm_package:=openarm", "hardware_profile:=mock", "--upstream-commit", COMMIT,
                  "--explain")
    assert result.returncode == 0, result.stderr
    full = result.stdout.strip().split(":")[1]
    assert f"prefix (12 hex, for log lines only, not an id): {full[:12]}" in result.stderr


@pytest.mark.parametrize("args,needle", [
    (["mast_id:=mast_1350", "arm_package:=openarm"], "missing: hardware_profile:=..."),
    (["mast_id:=mast_9999", "arm_package:=openarm", "hardware_profile:=mock"], "Accepted values"),
    (["mast_id:=mast_1350", "arm_package:=openarm", "hardware_profile:=mock", "bogus:=1"], "unrecognised argument"),
])
def test_c01_command_fails_loudly_on_bad_input(args, needle):
    result = _run(*args, "--upstream-commit", COMMIT)
    assert result.returncode != 0
    assert needle in result.stderr and not result.stdout.strip()


# --- the pinned upstream commit -------------------------------------------------------------------------------------

def test_c01_the_pinned_upstream_commit_is_a_full_hash_and_matches_the_baseline_pin():
    commit = _commit()
    assert re.fullmatch(r"[0-9a-f]{40}", commit)
    pinned = ev.pinned_commit_in_baseline(WORKSPACE)
    if pinned is None:
        pytest.skip("openamrobot-manipulation's baseline .repos is not in the workspace")
    assert commit == pinned, f"checkout is at {commit}, openarm_v2.dev.repos pins {pinned}"


def _checkout(tmp_path):
    upstream = tmp_path / "src" / "upstream" / "openarm_description"
    upstream.mkdir(parents=True)
    git = lambda *a: subprocess.run(["git", "-C", str(upstream), *a], check=True, capture_output=True)  # noqa: E731
    git("init", "-q")
    git("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "--allow-empty", "-m", "x")
    return upstream


def _pins(tmp_path, commit=COMMIT):
    path = tmp_path / "pins.yaml"
    path.write_text(f"schema_version: 1\npins:\n  openarm_description: {commit}\n")
    return str(path)


def test_c01_a_git_checkout_wins_over_the_package_pin(tmp_path):
    upstream = _checkout(tmp_path)
    commit, source = ev.upstream_commit_and_source(str(tmp_path), pins_file=_pins(tmp_path))
    assert source == ev.GIT_SOURCE and commit != COMMIT and re.fullmatch(r"[0-9a-f]{40}", commit)
    assert commit == ev.git_head(str(upstream))


def test_c01_a_dirty_checkout_is_an_error_and_is_not_replaced_by_the_pin(tmp_path):
    upstream = _checkout(tmp_path)
    (upstream / "local_edit.txt").write_text("x")
    with pytest.raises(ev.EvidenceError, match="local changes"):
        ev.upstream_commit(str(tmp_path), pins_file=_pins(tmp_path))


def test_c01_without_a_checkout_the_package_pin_is_used_and_labelled(tmp_path):
    assert ev.upstream_commit_and_source(str(tmp_path), pins_file=_pins(tmp_path)) == (COMMIT, ev.METADATA_SOURCE)


def test_c01_no_checkout_and_no_pin_is_an_error_not_a_guess(tmp_path):
    with pytest.raises(ev.EvidenceError, match="no git checkout"):
        ev.upstream_commit(str(tmp_path), pins_file=str(tmp_path / "missing.yaml"))
    other = tmp_path / "other.yaml"
    other.write_text("schema_version: 1\npins:\n  something_else: abc\n")
    with pytest.raises(ev.EvidenceError, match="no git checkout"):
        ev.upstream_commit(str(tmp_path), pins_file=str(other))


def test_c01_the_package_pin_is_a_full_hash_and_equals_the_baseline_pin():
    """The fallback constant may not rot against the manipulation baseline. Its agreement with a present checkout is C02:
    here the comparison would pass vacuously with no checkout, because the pin is then what `_commit()` returns."""
    pin = ev.pinned_commit_in_metadata()
    assert pin and re.fullmatch(r"[0-9a-f]{40}", pin)
    baseline = ev.pinned_commit_in_baseline(WORKSPACE)
    if baseline is not None:
        assert pin == baseline


def test_c01_the_fallback_gives_the_same_id_as_the_checkout(tmp_path):
    """An empty workspace has no checkout, so the package pin is used; the id it gives is the checkout's id."""
    commit, source = ev.upstream_commit_and_source(str(tmp_path))
    assert source == ev.METADATA_SOURCE
    assert _id(commit=commit) == _id(commit=_commit())


def test_c01_records_its_inputs_and_where_the_c9_pattern_came_from():
    commit = _commit()
    pinned = ev.pinned_commit_in_baseline(WORKSPACE)
    if os.path.isfile(C9_SCHEMA):
        c9 = {"checked": True, "schema": os.path.relpath(C9_SCHEMA, WORKSPACE),
              "pattern": next(iter({n["pattern"] for _, n in _c9_hash_definitions()})),
              "fields": [p for p, _ in _c9_hash_definitions()]}
    else:
        c9 = {"checked": False, "skipped_because": "openamrobot-interfaces is not in the workspace"}
    write_sidecar("C01", "config_id", "config id is sha256:<64 hex>, accepted by the C9 pattern", "PASS", {
        "format": "sha256:<64 lowercase hex>", "pattern": ev.SHA256_RE.pattern,
        "inputs": ["canonical expanded URDF (C14N, no comments, whitespace-stripped)", "mast_id", "arm_package",
                   "hardware_profile", "datums.revision", "openarm_description commit (full hash)"],
        "enable_planar_base": "not a separate input: it changes the expanded URDF",
        "openarm_description_commit": commit,
        "matches_baseline_pin": None if pinned is None else commit == pinned,
        "c9_schema": c9,
    })
