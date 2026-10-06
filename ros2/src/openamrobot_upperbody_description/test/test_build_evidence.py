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
    assert set(be.NOT_IMPLEMENTED) <= set(ids)
    assert be.SIDECAR_REQUIRED <= {i for i, _ in be.A6_CHECKS + be.EXTRA_CHECKS + be.GAZEBO_CHECKS}


@pytest.mark.parametrize("classname,name,expected", [
    ("pkg.test.test_t05_shoulder_height", "test_t05_shoulder_height_matches_mast_id[mast_1300]", "T05"),
    ("pkg.test.test_t07_collision", "test_t08_collision_mesh_same_file_as_visual_mesh_is_a_warning", "T08"),   # function wins
    ("pkg.test.test_t04_frame_contract", "test_checker_fails_a_missing_r_frame", "T04"),                      # module fallback
    ("pkg.test.test_w01_kdl_root_inertia", "test_w01_base_link_inertia_not_seen_by_kdl", "W01"),
    ("pkg.test.test_e01_envelope", "test_e01_assembled_height_against_the_envelope", "E01"),
    ("pkg.test.test_c01_config_id", "test_c01_command_prints_exactly_the_full_id", "C01"),
    ("pkg.test.test_c02_upstream_pin", "test_c02_a_wrong_pin_fails_and_names_both_hashes", "C02"),
    ("pkg.test.test_datums_revision", "test_guard_skips_with_a_single_commit", "REV"),
    ("pkg.test.test_datums_report", "test_report_exits_zero_and_lists_every_placeholder", "AUX"),
    ("pkg.test.test_s4_sensors", "test_camera_links_are_visual_only[head]", "AUX"),
    ("pkg.test.test_urdf_utils", "test_rpy_axes_follow_the_urdf_convention", "AUX"),
    ("pkg.test.test_checker_like", "test_checker_without_id", "AUX"),
    ("pkg.test.test_g01_gazebo_smoke", "test_g01_3_the_nav2_facing_topics_are_platform_sws_by_name_and_type", "G01"),
    ("pkg.test.test_g02_gripper_mimic", "test_g02_1_both_grippers_close_and_the_mimic_follows", "G02"),
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
    ("T14", None, None, "FAIL"),                                           # implemented since P06: no result is a FAIL
    ("L01", None, None, "FAIL"),
    ("T01", {**GOOD, "skipped": 5}, None, "SKIP"),                         # every test skipped
    ("T01", {**GOOD, "skipped": 2}, None, "PASS"),                         # some ran and passed
    ("C01", {**GOOD, "skipped": 2}, _sidecar("PASS"), "PASS"),
    ("C02", GOOD, _sidecar("PASS"), "PASS"),
    ("C02", {**GOOD, "skipped": 1}, _sidecar("SKIP", reason="no checkout"), "SKIP"),   # the one legitimate SKIP, by sidecar
    ("C02", GOOD, _sidecar("FAIL"), "FAIL"),                                           # drift is never softened
    ("C02", GOOD, None, "FAIL"),                                                       # the recording test did not complete
])
def test_combine(check_id, counts, sidecar, expected):
    assert be.combine(check_id, counts, sidecar)[0] == expected


def test_a_check_listed_as_not_implemented_is_a_skip_with_the_reason(monkeypatch):
    monkeypatch.setitem(be.NOT_IMPLEMENTED, "T14", "no test yet")
    assert be.combine("T14", None, None) == ("SKIP", {"reason": "no test yet"})


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


def test_read_xunit_dirs_merges_a_second_package_and_adds_counts_for_one_id(tmp_path):
    (tmp_path / "a").mkdir()
    (tmp_path / "b").mkdir()
    _xunit(tmp_path / "a" / "a.xunit.xml", [("test_t01_expansion", "test_t01_ok", "")])
    _xunit(tmp_path / "b" / "launch.xunit.xml", [("TestMockControllers", "test_l01_1_all_five_controllers_active", ""),
                                                 ("TestShutdown", "test_l01_processes_exit_cleanly", "<failure message='x'/>")])
    counts, stale = be.read_xunit_dirs([str(tmp_path / "a"), str(tmp_path / "b"), str(tmp_path / "missing")],
                                       since=time.time() - 3600)
    assert counts["T01"]["tests"] == 1 and stale == []
    assert counts["L01"]["tests"] == 2 and counts["L01"]["failures"] == 1
    assert be.combine("L01", counts["L01"], None)[0] == "FAIL"        # one failing L01 test fails L01


def test_a_stale_sidecar_is_not_used(tmp_path):
    ev.write_sidecar("T99", "probe", "probe", "PASS", {}, directory=str(tmp_path))
    old = next(p for p in tmp_path.iterdir())
    os.utime(old, (1, 1))
    ev.write_sidecar("T98", "fresh", "fresh", "WARN", {}, directory=str(tmp_path))
    assert set(be.fresh_sidecars(str(tmp_path), since=time.time() - 3600)) == {"T98"}


def _repo(tmp_path, dirty=False):
    """A throwaway git repository with one commit, standing in for openamr-upperbody-sw (--repo)."""
    repo = tmp_path / "repo"
    repo.mkdir()
    git = lambda *a: subprocess.run(["git", "-C", str(repo), *a], check=True, capture_output=True)  # noqa: E731
    git("init", "-q")
    (repo / "tracked.txt").write_text("a")
    git("add", "tracked.txt")
    git("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "-m", "x")
    if dirty:
        (repo / "tracked.txt").write_text("changed")
        (repo / "untracked.txt").write_text("new")
    return str(repo)


def _run_build(tmp_path, repo):
    evidence = tmp_path / "evidence"
    result = subprocess.run([sys.executable, os.path.join(SCRIPTS, "build_evidence.py"), "--since", str(time.time()),
                             "--results-dir", str(tmp_path / "none"), "--repo", repo], capture_output=True, text=True,
                            env={**os.environ, ev.EVIDENCE_ENV: str(evidence)})
    return result, [p for p in evidence.iterdir() if p.name.endswith("_dev.json")] if evidence.is_dir() else []


def test_summary_and_exit_status_follow_the_checks(tmp_path):
    # end to end on the real installed package with an empty results directory: nothing ran, so it must FAIL, and still write
    # (the repository it describes is a clean one: a dirty tree writes nothing, see below)
    result, reports = _run_build(tmp_path, _repo(tmp_path))
    assert result.returncode == 1, result.stdout + result.stderr
    assert len(reports) == 1
    doc = json.loads(reports[0].read_text())
    assert doc["schema"] == "openamr.evidence.v1" and doc["evidence_class"] == "static+mock" and doc["profile"] == "dev"
    assert doc["dirty"] is False and "dirty_files" not in doc
    by_id = {c["id"]: c["result"] for c in doc["checks"]}
    assert by_id["T01"] == "FAIL" and by_id["T14"] == "FAIL" and by_id["L01"] == "FAIL" and by_id["V01"] == "WARN"
    assert by_id["AUX"] == "FAIL"                                          # the three steps were never run
    assert set(doc["config_ids"]) and all(ev.SHA256_RE.match(v) for v in doc["config_ids"].values())
    assert len(doc["config_ids"]) == 14 and "mast_1350/openarm/mock" in doc["config_ids"]
    assert doc["limitations"] and all(isinstance(x, str) for x in doc["limitations"])
    assert "Result: FAIL" in result.stdout and f"Evidence: {reports[0]}" in result.stdout


def _run_gazebo_build(tmp_path, repo):
    evidence = tmp_path / "evidence"
    result = subprocess.run([sys.executable, os.path.join(SCRIPTS, "build_evidence.py"), "--profile", "gazebo", "--since",
                             str(time.time()), "--gazebo-results-dir", str(tmp_path / "none"), "--repo", repo],
                            capture_output=True, text=True, env={**os.environ, ev.EVIDENCE_ENV: str(evidence)})
    return result, sorted(p for p in evidence.iterdir() if p.name.endswith("_gazebo.json")) if evidence.is_dir() else []


def test_the_gazebo_profile_writes_a_simulation_report_and_never_a_physical_claim(tmp_path):
    result, reports = _run_gazebo_build(tmp_path, _repo(tmp_path))
    assert len(reports) == 1, result.stdout + result.stderr
    doc = json.loads(reports[0].read_text())
    assert doc["profile"] == "gazebo" and doc["evidence_class"] == "simulation" and doc["dirty"] is False
    assert [c["id"] for c in doc["checks"]] == [i for i, _ in be.GAZEBO_CHECKS]
    by_id = {c["id"]: c["result"] for c in doc["checks"]}
    assert by_id["G01"] == "FAIL" and by_id["G02"] == "FAIL"                       # nothing ran: no result is not a pass
    assert by_id["V01"] == "WARN" and "pending" in next(c for c in doc["checks"] if c["id"] == "V01")["details"]["performed"]
    assert set(doc["config_ids"]) == {"mast_1350/openarm/gazebo", "mast_1300/openarm/gazebo"}
    assert set(doc["simulator"]) == {"gz_sim", "gz_ros2_control", "ros_gz_sim"}
    text = " ".join(doc["limitations"])
    assert "no physical evidence" in doc["limitations"][-1] and "simulation only" in doc["limitations"][-1]
    for claim in ("works on hardware", "production-ready", "validated"):
        assert claim not in text.replace("not validated", "")


def test_the_gazebo_profile_refuses_a_dirty_tree_like_the_dev_one(tmp_path):
    result, reports = _run_gazebo_build(tmp_path, _repo(tmp_path, dirty=True))
    assert reports == [] and "Evidence: NOT WRITTEN" in result.stdout


def test_a_gazebo_check_with_no_sidecar_is_a_fail_not_a_pass():
    assert {"G01", "G02"} <= be.SIDECAR_REQUIRED
    assert be.combine("G01", GOOD, None)[0] == "FAIL"
    assert be.combine("G01", GOOD, _sidecar("PASS"))[0] == "PASS"
    assert be.combine("G02", GOOD, _sidecar("FAIL"))[0] == "FAIL"


def test_a_dirty_tree_writes_no_evidence_file_and_says_why(tmp_path):
    """The P08 failure: a dirty-tree run was left beside the clean one and the reviewer had to be told which to trust."""
    result, reports = _run_build(tmp_path, _repo(tmp_path, dirty=True))
    assert reports == [], "a report was written from a dirty tree"
    assert "Evidence: NOT WRITTEN" in result.stdout and "uncommitted changes" in result.stdout
    assert "tracked.txt" in result.stdout and "untracked.txt" in result.stdout        # names the files, tracked or not
    assert "(dirty)" in result.stdout                                                  # the console results still print
    assert result.returncode == 1                                                      # the FAILs of this empty run win


def test_a_dirty_tree_with_no_failing_check_exits_3_not_0(monkeypatch, capsys, tmp_path):
    """Exit 0 would read as 'recorded'. Nothing was, so it is its own status."""
    doc = {"repo": "openamr-upperbody-sw", "profile": "dev", "git_hash": "a" * 40, "dirty": True, "dirty_files": ["x.py"],
           "datums_revision": "r", "evidence_class": "static+mock", "config_ids": {}, "checks": [
               {"id": "T01", "name": "n", "result": "PASS", "details": {}}]}
    monkeypatch.setattr(be, "build", lambda args: (doc, None, {}))
    assert be.main(["--since", "0"]) == be.REFUSED_DIRTY == 3
    assert "NOT WRITTEN" in capsys.readouterr().out
    monkeypatch.setattr(be, "build", lambda args: ({**doc, "dirty": False, "dirty_files": []}, str(tmp_path / "r.json"), {}))
    assert be.main(["--since", "0"]) == 0


def test_clear_mode_removes_only_sidecars(tmp_path):
    ev.write_sidecar("T99", "probe", "probe", "PASS", {}, directory=str(tmp_path))
    (tmp_path / "S4.json").write_text("{}")
    result = subprocess.run([sys.executable, os.path.join(SCRIPTS, "build_evidence.py"), "--clear"], capture_output=True,
                            text=True, env={**os.environ, ev.EVIDENCE_ENV: str(tmp_path)})
    assert result.returncode == 0 and result.stdout.strip() == str(tmp_path)
    assert sorted(os.listdir(tmp_path)) == ["S4.json"]


def test_each_profile_clears_only_its_own_sidecars(tmp_path):
    """verify_gazebo.sh must not wipe the A6 sidecars the dev report was built from, nor the other way round."""
    for check_id in ("T99", "G01"):
        ev.write_sidecar(check_id, "probe", "probe", "PASS", {}, directory=str(tmp_path))
    env = {**os.environ, ev.EVIDENCE_ENV: str(tmp_path)}
    run = lambda *a: subprocess.run([sys.executable, os.path.join(SCRIPTS, "build_evidence.py"), *a, "--clear"],  # noqa: E731
                                    capture_output=True, text=True, env=env)
    assert run("--profile", "gazebo").returncode == 0 and sorted(os.listdir(tmp_path)) == ["T99_probe.json"]
    ev.write_sidecar("G01", "probe", "probe", "PASS", {}, directory=str(tmp_path))
    assert run("--profile", "dev").returncode == 0 and sorted(os.listdir(tmp_path)) == ["G01_probe.json"]
