# SPDX-License-Identifier: MIT
"""W04 (P18): a gz server the launch's stop helper had to force is a WARN of the A7 report; a survivor is still the launch test's FAIL. No ROS graph.

The aggregation reads the stop-helper report each simulator launch test (G01, G02, M02, M07) leaves in its sidecar. NOT_RUN, never PASS, when one is
missing: a W04 that cannot see is not a W04 that saw nothing."""
import importlib.util
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def builder():
    spec = importlib.util.spec_from_file_location("build_a7_evidence_w04", os.path.join(HERE, os.pardir, "scripts", "build_a7_evidence.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sidecar(state, **extra):
    return [{"details": {"gz_server_after_shutdown": {"survivors": [], "stop_helper": {"state": state, "elapsed_s": 1.4, **extra}}}}]


def sidecars(g01="stopped", g02="stopped", m02="stopped", m07="stopped"):
    return {"G01": sidecar(g01), "G02": sidecar(g02), "M02": sidecar(m02), "M07": sidecar(m07)}


def test_every_launch_stopped_on_request_is_a_pass():
    result, details = builder().gz_force_stop_check(sidecars())
    assert result == "PASS" and details["forced"] == []


def test_one_forced_server_is_a_warn_naming_the_launch():
    result, details = builder().gz_force_stop_check(sidecars(g01="force_stopped"))
    assert result == "WARN" and details["forced"] == ["G01"] and details["problems"]


def test_a_launch_with_no_helper_report_is_not_run_not_a_pass():
    b = builder()
    missing = sidecars()
    missing["G02"] = [{"details": {"gz_server_after_shutdown": {"survivors": [], "stop_helper": None}}}]
    assert b.gz_force_stop_check(missing)[0] == b.be.NOT_RUN
    assert b.gz_force_stop_check({"G01": sidecar("stopped")})[0] == b.be.NOT_RUN


def test_a_forced_server_of_m07_is_counted_too():
    result, details = builder().gz_force_stop_check(sidecars(m07="force_stopped"))
    assert result == "WARN" and details["forced"] == ["M07"]


def test_w04_is_listed_m07_stays_listed_and_w03_is_not_a_check():
    """M07 is Alex's and stands (P19); W03 is evidence files and a helper, not an entry of the report."""
    b = builder()
    ids = [i for i, _ in b.A7_CHECKS]
    assert "W04" in ids and "M07" in ids and "W03" not in ids
    assert "M07" in b.SIDECAR_REQUIRED and "W03" not in b.SIDECAR_REQUIRED
