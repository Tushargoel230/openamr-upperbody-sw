"""M03: the SRDF's collision matrix, validated by sampling (scripts/validate_acm.py, moveit_py).

For each of the four mast IDs, N = 2000 random joint states inside the limits; each is checked for collision IGNORING the SRDF's disabled
list; a pair disabled as `Never` that collides in any sample is a FAIL, reported with the pair name and a reproducing sample. The pairs
disabled as `Never` that never collided are recorded too (the expected outcome, and what makes the check legible).

THIS TEST IS NOT SOFTENED: when M03 finds a disabled pair that collides, the ACM is wrong, and the test fails with that pair and sample.
It must not be satisfied by re-enabling the pair and moving on: which pair is wrong is the finding (A7 stop condition). The mutation test
below shows the check CAN fail: it relabels a pair that really collides as `Never` and expects exactly that pair to be reported."""
import json
import os
import re
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_prefix

from conftest import SRDF_FILE, load_yaml, read_text, write_sidecar

SCRIPT = os.path.join(get_package_prefix("openamrobot_upperbody_moveit"), "lib", "openamrobot_upperbody_moveit", "validate_acm.py")
SAMPLES = 2000
MASTS = ("mast_1300", "mast_1350", "mast_1400", "mast_1450")


def run_validate(*args, timeout):
    env = {**os.environ, "ROS_DOMAIN_ID": str(__import__("random").randint(201, 232)), "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
           "PYTHONDONTWRITEBYTECODE": "1"}
    return subprocess.run([sys.executable, SCRIPT, *args], capture_output=True, text=True, timeout=timeout, env=env)


def test_m03_1_no_pair_disabled_as_never_collides_in_any_sample_at_any_mast_id():
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, "m03.json")
        done = run_validate("--samples", str(SAMPLES), "--json", out, timeout=1500)
        assert os.path.isfile(out), f"validate_acm.py produced no result (exit {done.returncode}):\n{done.stdout[-1500:]}\n{done.stderr[-1500:]}"
        with open(out) as f:
            summary = json.load(f)
    meta = load_yaml("acm/acm_generation.yaml")
    results = summary["results"]
    failing = {r["mast_id"]: r["never_pairs_that_collided"] for r in results if r["never_pairs_that_collided"]}
    first = results[0] if results else {}
    status = "FAIL" if summary["result"] != "PASS" else "PASS"
    write_sidecar("M03", "acm_sampling", "ACM validated by sampling: no pair disabled as Never collides (N=2000 at each mast id)", status, {
        "samples_per_mast_id": SAMPLES, "seed": summary["seed"], "mast_ids": [r["mast_id"] for r in results],
        "method": "random joint states inside the limits (numpy default_rng(seed), same states at every mast id); collision checked "
                  "through moveit_py with an allowed-collision matrix of its own that allows everything except the pairs under test, "
                  "so the SRDF's disabled list is never consulted; colliding pairs found by bisection",
        "acm_generation": {k: meta[k] for k in ("tool", "moveit_version", "date", "mast_id", "trials", "categories", "command",
                                                "tool_statistics", "written", "imported_before_the_markers")},
        "disabled_pairs": first.get("disabled_pairs"),
        "never_pairs_that_collided": failing,
        "never_pairs_that_never_collided_count": {r["mast_id"]: len(r["never_pairs_that_never_collided"]) for r in results},
        "never_pairs_that_never_collided": first.get("never_pairs_that_never_collided"),
        "never_pairs_that_never_collided_listed_for": first.get("mast_id"),
        "adjacent_or_default_pairs_that_touched": {r["mast_id"]: [r["adjacent_or_default_pairs_that_touched"], r["adjacent_or_default_pairs_total"]]
                                                   for r in results},
        "seconds_per_mast_id": {r["mast_id"]: r["seconds"] for r in results},
        "stop_condition": "A7: a disabled pair that collides means the ACM is wrong; report the pair and the sample, do not re-enable it "
                          "and move on" if failing else None,
    })
    assert summary["result"] == "PASS", "\n" + done.stdout[-3500:]
    assert [r["mast_id"] for r in results] == list(MASTS)


def test_m03_2_the_check_can_fail_a_pair_that_really_collides_relabelled_never_is_reported():
    """Mutation: openarm_left_link1 / openarm_left_link2 are adjacent links of the left arm: they touch in nearly every sample. Disabled as
    `Adjacent` they are fine; relabelled `Never` the check must name exactly that pair."""
    pair = ("openarm_left_link1", "openarm_left_link2")
    text = read_text(SRDF_FILE)
    mutated, count = re.subn(r'(link1="openarm_left_link1" link2="openarm_left_link2" reason=")Adjacent(")', r"\1Never\2", text)
    assert count == 1, "the SRDF no longer has that adjacent pair under that name; pick another pair that touches by design"
    with tempfile.TemporaryDirectory() as tmp:
        srdf = os.path.join(tmp, "mutated.srdf")
        out = os.path.join(tmp, "m03.json")
        with open(srdf, "w") as f:
            f.write(mutated)
        done = run_validate("--mast-id", "mast_1350", "--samples", "100", "--srdf", srdf, "--json", out, timeout=300)
        assert done.returncode == 1, f"expected exit 1 (a Never pair collided), got {done.returncode}:\n{done.stdout[-1200:]}\n{done.stderr[-800:]}"
        with open(out) as f:
            result = json.load(f)["results"][0]
    assert result["result"] == "FAIL"
    hit, = [h for h in result["never_pairs_that_collided"] if tuple(h["pair"]) == pair]
    assert hit["collided_in_samples"] > 10 and set(hit["reproducing_sample"]) >= {"openarm_left_joint1", "openarm_left_joint2"}
    assert f"{pair[0]} <-> {pair[1]}" in done.stdout


def test_m03_3_bisection_finds_every_colliding_pair_and_calls_the_check_once_when_nothing_collides():
    sys.path.insert(0, os.path.dirname(SCRIPT))
    import validate_acm as v
    pairs = [(f"a{i}", f"b{i}") for i in range(37)]
    for hits in (set(), {pairs[0]}, {pairs[36]}, {pairs[3], pairs[4], pairs[30]}, set(pairs)):
        calls = []

        def check(subset):
            calls.append(len(subset))
            return any(p in hits for p in subset)

        assert sorted(v.colliding_pairs(check, pairs)) == sorted(hits)
        if not hits:
            assert calls == [37]
