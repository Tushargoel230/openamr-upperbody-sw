"""scripts/build_evidence.py: the report only combines what the tests recorded. Its mapping and combining rules are
tested here, on synthetic results, including that a FAIL cannot be hidden by a PASS and that a missing result is not a pass.
"""
import json
import os
import subprocess
import sys
import time

import pytest

import build_evidence as be
import evidence_common as ev
from conftest import SCRIPTS


def test_every_a6_check_id_in_verification_md_is_in_the_report():
    ids = [i for i, _ in be.A6_CHECKS]
    assert ids == [f"T{n:02d}" for n in range(1, 17)] + ["L01", "E01"]
    assert len(set(i for i, _ in be.A6_CHECKS + be.EXTRA_CHECKS)) == len(be.A6_CHECKS + be.EXTRA_CHECKS)
    assert set(be.NOT_IMPLEMENTED) == {"T14", "T15", "T16", "L01"} and set(be.NOT_IMPLEMENTED) <= set(ids)
    assert be.SIDECAR_REQUIRED <= {i for i, _ in be.A6_CHECKS + be.EXTRA_CHECKS}


@pytest.mark.parametrize("classname,name,expected", [
    ("pkg.test.test_t05_shoulder_height", "test_t05_shoulder_height_matches_mast_id[mast_1300]", "T05"),
    ("pkg.test.test_t07_collision", "test_t08_collision_mesh_same_file_as_visual_mesh_is_a_warning", "T08"),   # function wins
    ("pkg.test.test_t04_frame_contract", "test_checker_fails_a_missing_r_frame", "T04"),                      # module fallback
    ("pkg.test.test_w01_kdl_root_inertia", "test_w01_base_link_inertia_not_seen_by_kdl", "W01"),
    ("pkg.test.test_e01_envelope", "test_e01_assembled_height_against_the_envelope", "E01"),
    ("pkg.test.test_c01_config_id", "test_c01_command_prints_exactly_the_full_id", "C01"),
    ("pkg.test.test_datums_revision", "test_guard_skips_with_a_single_commit", "REV"),
    ("pkg.test.test_datums_report", "test_report_exits_zero_and_lists_every_placeholder", "AUX"),
    ("pkg.test.test_s4_sensors", "test_camera_links_are_visual_only[head]", "AUX"),
    ("pkg.test.test_urdf_utils", "test_rpy_axes_follow_the_urdf_convention", "AUX"),
    ("pkg.test.test_checker_like", "test_checker_without_id", "AUX"),
])
def test_check_id_mapping(classname, name, expected):
    assert be.check_id_of(classname, name) == expected


GOOD = {"tests": 5, "failures": 0, "skipped": 0, "failed_tests": []}


def _sidecar(result, **details):
    return {"id": "X", "name": "from sidecar", "result": result, "details": details}


@pytest.mark.parametrize("check_id,counts,sidecar,expected", [
    ("T01", GOOD, None, "PASS"),
    ("T01", {**GOOD, "failures": 1, "failed_tests": ["t"]}, None, "FAIL"),
    ("T04", GOOD, _sidecar("PASS"), "PASS"),
    ("T04", GOOD, _sidecar("WARN"), "WARN"),
    ("T04", {**GOOD, "failures": 1}, _sidecar("PASS"), "FAIL"),            # a PASS sidecar cannot hide a failing test
    ("T04", GOOD, _sidecar("FAIL"), "FAIL"),                               # nor a passing test hide a FAIL sidecar
    ("T04", {**GOOD, "failures": 1}, _sidecar("WARN"), "FAIL"),            # nor a WARN
    ("T10", GOOD, None, "FAIL"),                                           # sidecar required, test did not complete
    ("E01", None, None, "FAIL"),
    ("T05", None, None, "FAIL"),                                           # no result is not a pass
    ("T14", None, None, "SKIP"),                                           # explicitly not implemented
    ("L01", None, None, "SKIP"),
    ("T01", {**GOOD, "skipped": 5}, None, "SKIP"),                         # every test skipped
    ("T01", {**GOOD, "skipped": 2}, None, "PASS"),                         # some ran and passed
    ("C01", {**GOOD, "skipped": 2}, _sidecar("PASS"), "PASS"),
])
def test_combine(check_id, counts, sidecar, expected):
    assert be.combine(check_id, counts, sidecar)[0] == expected


def test_combine_keeps_the_sidecar_details_and_the_test_counts():
    result, details = be.combine("T09", {**GOOD, "failures": 0}, _sidecar("PASS", massed_links=36))
    assert result == "PASS" and details["massed_links"] == 36 and details["tests"] == 5


def test_trees_differ_reports_changed_added_and_missing_files(tmp_path):
    for side in ("src", "inst"):
        (tmp_path / side / "urdf").mkdir(parents=True)
        (tmp_path / side / "urdf" / "a.xacro").write_text("same")
        (tmp_path / side / "config").mkdir()
        (tmp_path / side / "config" / "d.yaml").write_text("same")
    assert be.trees_differ(str(tmp_path / "src"), str(tmp_path / "inst")) == []
    (tmp_path / "src" / "urdf" / "a.xacro").write_text("changed")
    (tmp_path / "src" / "urdf" / "new.xacro").write_text("only in src")
    (tmp_path / "inst" / "config" / "old.yaml").write_text("only installed")
    assert be.trees_differ(str(tmp_path / "src"), str(tmp_path / "inst")) == [
        "config/old.yaml", "urdf/a.xacro", "urdf/new.xacro"]


def test_trees_differ_ignores_content_free_leftovers_but_not_a_stale_installed_xacro(tmp_path):
    for side in ("src", "inst"):
        (tmp_path / side / "urdf").mkdir(parents=True)
        (tmp_path / side / "urdf" / "a.xacro").write_text("same")
    (tmp_path / "inst" / "urdf" / ".gitkeep").symlink_to(tmp_path / "gone")          # dangling, as left by an old scaffold
    (tmp_path / "inst" / "urdf" / "__pycache__").mkdir()
    assert be.trees_differ(str(tmp_path / "src"), str(tmp_path / "inst")) == []
    (tmp_path / "inst" / "urdf" / "stale.xacro").write_text("removed from the source long ago")
    assert be.trees_differ(str(tmp_path / "src"), str(tmp_path / "inst")) == ["urdf/stale.xacro"]


def _xunit(path, cases):
    body = "".join(
        f'<testcase classname="p.test.{mod}" name="{name}">{inner}</testcase>' for mod, name, inner in cases)
    path.write_text(f'<?xml version="1.0"?><testsuites><testsuite name="pytest">{body}</testsuite></testsuites>')


def test_read_xunit_counts_per_id_and_ignores_stale_files(tmp_path):
    _xunit(tmp_path / "a.xunit.xml", [
        ("test_t01_expansion", "test_t01_ok[1]", ""), ("test_t01_expansion", "test_t01_bad", "<failure message='x'/>"),
        ("test_t01_expansion", "test_t01_err", "<error message='x'/>"), ("test_t01_expansion", "test_t01_skip", "<skipped/>")])
    _xunit(tmp_path / "old.xunit.xml", [("test_t03_tree", "test_t03_x", "")])
    os.utime(tmp_path / "old.xunit.xml", (1, 1))
    counts, stale = be.read_xunit(str(tmp_path), since=time.time() - 3600)
    assert counts["T01"] == {"tests": 4, "failures": 2, "skipped": 1, "failed_tests": ["test_t01_bad", "test_t01_err"]}
    assert "T03" not in counts and stale == ["old.xunit.xml"]


def test_a_stale_sidecar_is_not_used(tmp_path):
    ev.write_sidecar("T99", "probe", "probe", "PASS", {}, directory=str(tmp_path))
    old = next(p for p in tmp_path.iterdir())
    os.utime(old, (1, 1))
    ev.write_sidecar("T98", "fresh", "fresh", "WARN", {}, directory=str(tmp_path))
    assert set(be.fresh_sidecars(str(tmp_path), since=time.time() - 3600)) == {"T98"}


def test_summary_and_exit_status_follow_the_checks(tmp_path, monkeypatch):
    # end to end on the real installed package with an empty results directory: nothing ran, so it must FAIL, and still write
    monkeypatch.setenv(ev.EVIDENCE_ENV, str(tmp_path))
    result = subprocess.run([sys.executable, os.path.join(SCRIPTS, "build_evidence.py"), "--since", str(time.time()),
                             "--results-dir", str(tmp_path / "none")], capture_output=True, text=True,
                            env={**os.environ, ev.EVIDENCE_ENV: str(tmp_path)})
    assert result.returncode == 1, result.stdout + result.stderr
    reports = [p for p in tmp_path.iterdir() if p.name.endswith("_dev.json")]
    assert len(reports) == 1
    doc = json.loads(reports[0].read_text())
    assert doc["schema"] == "openamr.evidence.v1" and doc["evidence_class"] == "static+mock" and doc["profile"] == "dev"
    by_id = {c["id"]: c["result"] for c in doc["checks"]}
    assert by_id["T01"] == "FAIL" and by_id["T14"] == "SKIP" and by_id["V01"] == "WARN"
    assert by_id["AUX"] == "FAIL"                                          # the three steps were never run
    assert set(doc["config_ids"]) and all(ev.SHA256_RE.match(v) for v in doc["config_ids"].values())
    assert len(doc["config_ids"]) == 14 and "mast_1350/openarm/mock" in doc["config_ids"]
    assert doc["limitations"] and all(isinstance(x, str) for x in doc["limitations"])
    assert "Result: FAIL" in result.stdout


def test_clear_mode_removes_only_sidecars(tmp_path):
    ev.write_sidecar("T99", "probe", "probe", "PASS", {}, directory=str(tmp_path))
    (tmp_path / "S4.json").write_text("{}")
    result = subprocess.run([sys.executable, os.path.join(SCRIPTS, "build_evidence.py"), "--clear"], capture_output=True,
                            text=True, env={**os.environ, ev.EVIDENCE_ENV: str(tmp_path)})
    assert result.returncode == 0 and result.stdout.strip() == str(tmp_path)
    assert sorted(os.listdir(tmp_path)) == ["S4.json"]
