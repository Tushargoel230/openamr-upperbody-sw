"""C02: config/upstream_pins.yaml agrees with the upstream checkout when there is one.

The pin is the fallback that lets a reviewer with no `src/upstream/` compute the same `config_id` as the author (C01). That
only holds while the pin equals the commit a real checkout is at, so the two are compared whenever both exist: equal is
PASS, different is FAIL naming both hashes, a checkout with local changes is FAIL. With no checkout the result is SKIP,
with the reason, and that is the only legitimate SKIP in this tree.

The check compares the pin with the checkout, not with upstream's latest commit: holding both behind upstream on purpose
(they are pinned to the manipulation baseline) is not drift.

The three outcomes are demonstrated on synthetic checkouts in tmp_path, so they run whether or not the real checkout is
there; the test that records C02's own result for the evidence report runs against the real workspace.
"""
import os
import subprocess

import pytest

import evidence_common as ev
from conftest import SHARE, write_sidecar

WORKSPACE = ev.workspace_root(SHARE)
CHECK_NAME = "upstream pin file agrees with the upstream checkout"


def _checkout(tmp_path):
    upstream = tmp_path / "src" / "upstream" / "openarm_description"
    upstream.mkdir(parents=True)
    git = lambda *a: subprocess.run(["git", "-C", str(upstream), *a], check=True, capture_output=True)  # noqa: E731
    git("init", "-q")
    git("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "--allow-empty", "-m", "x")
    return upstream, ev.git_head(str(upstream))


def _pins(tmp_path, commit):
    path = tmp_path / "pins.yaml"
    path.write_text(f"schema_version: 1\npins:\n  openarm_description: {commit}\n")
    return str(path)


def test_c02_the_real_pin_file_agrees_with_the_real_checkout_or_says_why_it_skips():
    result, details = ev.pin_vs_checkout(WORKSPACE)
    write_sidecar("C02", "upstream_pin", CHECK_NAME, result, details)
    if result == "SKIP":
        pytest.skip(details["reason"])
    assert result == "PASS", details["reason"]
    assert details["pin"] == details["checkout_head"]


def test_c02_equal_pin_and_checkout_pass(tmp_path):
    _, head = _checkout(tmp_path)
    result, details = ev.pin_vs_checkout(str(tmp_path), pins_file=_pins(tmp_path, head))
    assert result == "PASS" and details["pin"] == details["checkout_head"] == head


def test_c02_a_wrong_pin_fails_and_names_both_hashes(tmp_path):
    _, head = _checkout(tmp_path)
    wrong = "deadbeef" * 5
    result, details = ev.pin_vs_checkout(str(tmp_path), pins_file=_pins(tmp_path, wrong))
    assert result == "FAIL"
    assert wrong in details["reason"] and head in details["reason"]
    assert (details["pin"], details["checkout_head"]) == (wrong, head)


def test_c02_an_unquoted_all_digit_pin_is_not_silently_compared_as_a_number(tmp_path):
    _, head = _checkout(tmp_path)
    result, details = ev.pin_vs_checkout(str(tmp_path), pins_file=_pins(tmp_path, "0" * 40))   # YAML reads this as 0
    assert result == "FAIL" and "not a string" in details["reason"] and details["checkout_head"] == head


def test_c02_a_wrong_pin_fails_against_the_real_checkout_too(tmp_path):
    real = ev.upstream_checkout(WORKSPACE, "openarm_description")
    if ev.git_head(real) is None:
        pytest.skip(f"no real checkout at {real}; the synthetic test above covers the same path")
    wrong = "f" * 40
    result, details = ev.pin_vs_checkout(WORKSPACE, pins_file=_pins(tmp_path, wrong))
    assert result == "FAIL" and wrong in details["reason"] and ev.git_head(real) in details["reason"]


def test_c02_an_absent_checkout_skips_and_says_it_is_the_only_legitimate_skip(tmp_path):
    result, details = ev.pin_vs_checkout(str(tmp_path), pins_file=_pins(tmp_path, "a" * 40))
    assert result == "SKIP"
    assert "no git checkout of openarm_description" in details["reason"]
    assert "only legitimate SKIP" in details["reason"]


def test_c02_a_directory_that_is_not_a_git_checkout_is_absent_not_a_pass(tmp_path):
    (tmp_path / "src" / "upstream" / "openarm_description").mkdir(parents=True)
    result, _ = ev.pin_vs_checkout(str(tmp_path), pins_file=_pins(tmp_path, "a" * 40))
    assert result == "SKIP"


def test_c02_a_dirty_checkout_fails_even_when_head_equals_the_pin(tmp_path):
    upstream, head = _checkout(tmp_path)
    (upstream / "local_edit.txt").write_text("x")
    result, details = ev.pin_vs_checkout(str(tmp_path), pins_file=_pins(tmp_path, head))
    assert result == "FAIL" and "local changes" in details["reason"]


def test_c02_a_checkout_with_no_pin_in_the_file_fails(tmp_path):
    _, head = _checkout(tmp_path)
    other = tmp_path / "other.yaml"
    other.write_text("schema_version: 1\npins:\n  something_else: abc\n")
    result, details = ev.pin_vs_checkout(str(tmp_path), pins_file=str(other))
    assert result == "FAIL" and "no pin for openarm_description" in details["reason"] and details["checkout_head"] == head


def test_c02_a_pin_held_behind_the_upstream_tip_is_not_drift(tmp_path):
    # the checkout is NOT advanced to a newer commit: a later upstream commit that nobody checked out changes nothing
    upstream, head = _checkout(tmp_path)
    subprocess.run(["git", "-C", str(upstream), "-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q",
                    "--allow-empty", "-m", "newer"], check=True, capture_output=True)
    newer = ev.git_head(str(upstream))
    subprocess.run(["git", "-C", str(upstream), "checkout", "-q", head], check=True, capture_output=True)
    result, _ = ev.pin_vs_checkout(str(tmp_path), pins_file=_pins(tmp_path, head))
    assert head != newer and result == "PASS"
