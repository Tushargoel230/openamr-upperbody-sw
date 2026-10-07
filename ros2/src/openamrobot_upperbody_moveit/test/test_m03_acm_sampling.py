"""M03: the SRDF's collision matrix, split by who owns the claim (scripts/validate_acm.py, moveit_py, acm_split.py).

The matrix has two parts. INHERITED pairs have two upstream links (Enactic's: the frame contract's owner column, else the `openarm_` prefix);
a colliding one is a WARN naming the pair, the seed and the sample index. OUR pairs have at least one link of ours (base, wheels, casters,
lidar, camera, mast, carriage, whatever is added next, because the split is derived); a colliding disabled pair there is a FAIL. Within
our `Never` pairs, those with no movable joint between their links are certified by construction (one evaluation settles every
configuration); sampling is applied only to the rest, and sampling can only falsify: its result is "no counterexample in N samples at
seed S", never "validated".

THIS TEST IS NOT SOFTENED: when M03 finds a disabled pair of ours that collides, the ACM is wrong, and the test fails with that pair and
sample. It must not be satisfied by re-enabling the pair and moving on. The mutation tests below show the check CAN fail, by each of its two
routes, and that the same defect in an inherited pair is a WARN."""
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


# Pairs that REALLY collide, measured with nothing ignored (nothing is assumed from a link being `Adjacent`: the P10 version of this test
# relabelled openarm_left_link1 / link2 and expected more than 10 hits in 100 samples, and it never got them: it failed in P10's own run).
#   base_link / mast_base_link          ours, no joint between: collides in the default state, so in every state (certified by construction)
#   mast_base_link / openarm_left_ee_base_link   ours, arm joints between: collides in 7 of 300 states at seed 20261006 (sampled)
#   openarm_left_link5 / openarm_left_link6      inherited (both upstream's): collides in every one of 300 states
BY_CONSTRUCTION = ("base_link", "mast_base_link")
BY_SAMPLING = ("mast_base_link", "openarm_left_ee_base_link")
INHERITED = ("openarm_left_link5", "openarm_left_link6")


def mutated_srdf(pair, add=False):
    """The installed SRDF's text with `pair` disabled as `Never`: relabelled if it is disabled already, added if it is not."""
    text = read_text(SRDF_FILE)
    if add:
        assert f'link1="{pair[0]}" link2="{pair[1]}"' not in text, f"{pair} is disabled already"
        return text.replace("    <!-- END GENERATED ACM -->", f'    <disable_collisions link1="{pair[0]}" link2="{pair[1]}" reason="Never"/>\n    <!-- END GENERATED ACM -->')
    mutated, count = re.subn(rf'(link1="{pair[0]}" link2="{pair[1]}" reason=")(Adjacent)(")', r"\1Never\3", text)
    assert count == 1, f"the SRDF no longer has {pair} disabled as Adjacent"
    return mutated


def validate_mutated(pair, tmp, samples, add=False):
    srdf, out = os.path.join(tmp, "mutated.srdf"), os.path.join(tmp, "m03.json")
    with open(srdf, "w") as f:
        f.write(mutated_srdf(pair, add))
    done = run_validate("--mast-id", "mast_1350", "--samples", str(samples), "--srdf", srdf, "--json", out, timeout=300)
    with open(out) as f:
        return done, json.load(f)["results"][0]


def test_m03_1_no_disabled_pair_of_ours_collides_by_construction_or_in_any_sample_at_any_mast_id():
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, "m03.json")
        done = run_validate("--samples", str(SAMPLES), "--json", out, timeout=2400)
        assert os.path.isfile(out), f"validate_acm.py produced no result (exit {done.returncode}):\n{done.stdout[-1500:]}\n{done.stderr[-1500:]}"
        with open(out) as f:
            summary = json.load(f)
    meta = load_yaml("acm/acm_generation.yaml")
    results = summary["results"]
    first = results[0] if results else {}
    status = "PASS" if summary["result"] == "PASS" else "FAIL"
    failing = {r["mast_id"]: {"sampled": r["our_pairs_that_collided"], "by_construction": r["by_construction"]["colliding"],
                              "cross_check_disagreements": r["by_construction"]["cross_check_with_sampled_states"]["disagreements"]}
               for r in results if r["result"] == "FAIL"}
    n, seed = summary["samples_per_mast"], summary["seed"]
    write_sidecar("M03", "acm_sampling", "ACM: no disabled pair of ours collides (by construction or in N=2000 samples at each mast id); "
                  "inherited collisions are WARNs", status, {
        "claim": f"no counterexample found among our sampled pairs in {n} samples at seed {seed} (per mast id); our by-construction pairs are "
                 "clear in the one configuration that decides them. Sampling falsifies and never certifies: this is not 'validated'",
        "samples_per_mast_id": n, "seed": seed, "mast_ids": [r["mast_id"] for r in results],
        "pairs": {"disabled_total": first.get("disabled_pairs", {}).get("total"),
                  "ours": first.get("our_pairs"), "inherited": first.get("inherited_pairs")},
        "certified_by_construction": {"count": {r["mast_id"]: r["our_pairs"]["certified_by_construction"] for r in results},
                                      "colliding": {r["mast_id"]: r["by_construction"]["colliding"] for r in results},
                                      "evaluated": first.get("by_construction", {}).get("evaluated"),
                                      "cross_check_with_sampled_states": {r["mast_id"]: r["by_construction"]["cross_check_with_sampled_states"]
                                                                          for r in results},
                                      "pairs_at_the_first_mast_id": first.get("by_construction", {}).get("pairs")},
        "sampled": {"count": {r["mast_id"]: r["our_pairs"]["sampled"] for r in results}, "samples": n, "seed": seed},
        "warnings": summary["warnings"],
        "warnings_note": "inherited pairs (both links upstream's) that collided although disabled `Never`: an observation about the matrix's "
                         "upstream-owned part, not a defect of ours. `origin` says whether Enactic listed the pair (imported_from_upstream) "
                         "or the collision-matrix tool wrote it (generated_for_upstream_links); `arms: left_vs_right` pairs depend on how this "
                         "model places the two arms, which is ours",
        "method": "random joint states inside the limits (numpy default_rng(seed), same states at every mast id); collision checked through "
                  "moveit_py with an allowed-collision matrix of its own that allows everything except the pairs under test, so the SRDF's "
                  "disabled list is never consulted; colliding pairs found by bisection",
        "link_provenance": {"ours": first.get("link_provenance", {}).get("ours"), "upstream": first.get("link_provenance", {}).get("upstream"),
                            "derived_from": "frame_contract.yaml owner column, else the openarm_ prefix; unlisted links are ours"},
        "acm_generation": {k: meta[k] for k in ("tool", "moveit_version", "date", "mast_id", "trials", "categories", "command",
                                                "tool_statistics", "written", "imported_before_the_markers")},
        "ours_never_pairs_that_collided": failing,
        "adjacent_or_default_pairs_that_touched": {r["mast_id"]: [r["adjacent_or_default_pairs_that_touched"], r["adjacent_or_default_pairs_total"]]
                                                   for r in results},
        "seconds_per_mast_id": {r["mast_id"]: r["seconds"] for r in results},
        "stop_condition": "A7: a disabled pair of ours that collides means our part of the ACM is wrong; report the pair and the sample, do not "
                          "re-enable it and move on" if failing else None,
    })
    assert summary["result"] == "PASS", "\n" + done.stdout[-3500:]
    assert [r["mast_id"] for r in results] == list(MASTS)
    assert all(r["our_pairs"]["certified_by_construction"] > 0 and r["our_pairs"]["sampled"] > 0 for r in results)
    assert all(not r["by_construction"]["cross_check_with_sampled_states"]["disagreements"] for r in results)


def test_m03_2_the_check_can_fail_a_pair_of_ours_by_sampling_and_by_construction():
    """Mutation: a pair of ours that really collides is disabled as `Never`, and the check must name exactly that pair, by the route that applies
    to it: sampled (arm joints between the links) and by construction (no joint between them)."""
    with tempfile.TemporaryDirectory() as tmp:
        done, sampled = validate_mutated(BY_SAMPLING, tmp, 300, add=True)
        assert done.returncode == 1, f"expected exit 1, got {done.returncode}:\n{done.stdout[-1200:]}\n{done.stderr[-800:]}"
        assert sampled["result"] == "FAIL"
        hit, = [h for h in sampled["our_pairs_that_collided"] if tuple(h["pair"]) == BY_SAMPLING]
        assert hit["collided_in_samples"] >= 1 and hit["seed"] == 20261006 and "openarm_left_joint1" in hit["reproducing_sample"]
        assert not sampled["by_construction"]["colliding"]
        assert f"{BY_SAMPLING[0]} <-> {BY_SAMPLING[1]}" in done.stdout

        done, constructed = validate_mutated(BY_CONSTRUCTION, tmp, 20)
        assert done.returncode == 1, f"expected exit 1, got {done.returncode}:\n{done.stdout[-1200:]}\n{done.stderr[-800:]}"
        assert constructed["result"] == "FAIL"
        assert [tuple(c["pair"]) for c in constructed["by_construction"]["colliding"]] == [BY_CONSTRUCTION]
        assert not constructed["our_pairs_that_collided"], "a pair settled by construction is not sampled"
        assert constructed["our_pairs"]["certified_by_construction"] == sampled["our_pairs"]["certified_by_construction"] + 1
        assert f"FAIL by construction: {BY_CONSTRUCTION[0]} <-> {BY_CONSTRUCTION[1]}" in done.stdout


def test_m03_3_the_same_defect_in_an_inherited_pair_is_a_warn_with_the_pair_the_seed_and_the_sample_index():
    """Both links are upstream's, so the pair is inherited: a collision is a WARN naming the pair, the seed and the sample index, and exit 0."""
    with tempfile.TemporaryDirectory() as tmp:
        done, result = validate_mutated(INHERITED, tmp, 50)
    assert done.returncode == 0, f"an inherited collision must not fail M03; exit {done.returncode}:\n{done.stdout[-1200:]}"
    assert result["result"] == "PASS" and not result["our_pairs_that_collided"] and not result["by_construction"]["colliding"]
    hit, = [h for h in result["inherited_pairs_that_collided"] if tuple(h["pair"]) == INHERITED]
    assert hit["collided_in_samples"] == 50 and hit["seed"] == 20261006 and hit["first_sample_index"] == 0
    assert hit["origin"] == "imported_from_upstream" and hit["arms"] == "same_arm"
    assert f"WARN: inherited {INHERITED[0]} <-> {INHERITED[1]}" in done.stdout


def test_m03_4_bisection_finds_every_colliding_pair_and_calls_the_check_once_when_nothing_collides():
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
