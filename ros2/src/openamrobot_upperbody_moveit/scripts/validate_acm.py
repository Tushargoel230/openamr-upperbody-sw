#!/usr/bin/env python3
"""M03: validate the SRDF's collision matrix, split by who owns the claim, and say what each part of the check can and cannot establish.

    ros2 run openamrobot_upperbody_moveit validate_acm.py [--samples 2000] [--seed N] [--mast-id ID ...] [--srdf PATH] [--json OUT]

The matrix is split, by derivation and never by a list (acm_split.py): a pair whose two links are both upstream's (Enactic's: the frame
contract's owner column, else the `openarm_` prefix) is **inherited**; every other pair is **ours**. Only pairs disabled as `Never` are
claims about collision; `Adjacent` and `Default` pairs are disabled because they touch by design, and are counted, never failed.

  * OUR `Never` pairs with no movable joint between their links are **certified by construction**: their relative transform is a
    constant, so ONE evaluation (the default state) settles them for every configuration. They are not sampled. As a check on that
    reasoning, and not as a certification, the first 200 sampled states must agree with the one evaluation, pair for pair.
  * OUR `Never` pairs with a movable joint between them are **sampled**: N random states inside the limits, the same states at every
    mast ID. Sampling falsifies and never certifies: a pass is only "no counterexample in N samples at seed S". A colliding pair of ours,
    by construction or by sampling, is a FAIL, named with the state that reproduces it.
  * INHERITED `Never` pairs are sampled too, and one that collides is a WARN: an observation about upstream's matrix (or about how
    this model places upstream's arms), carrying the pair, the seed and the sample index. It is not a defect of ours.

Every state is checked IGNORING the SRDF's disabled list: the check is built from an allowed-collision matrix of its own, which allows every
pair except the ones under test, so nothing the SRDF says is consulted.

If this finds a disabled pair of ours that collides, the ACM is wrong and WHICH pair is wrong is the finding: do not re-enable it and
move on.

Why a bisection: moveit_py's CollisionResult exposes `collision` and `contact_count` but not the contact map (the Contact type is not
bound), so the colliding pairs are found by group testing: one check with only the pairs under test forbidden, and, on a hit, halving
the set until single pairs are left. Reproducible: states come from numpy's default_rng(seed), the same seed at every mast ID.

Exit status: 1 if a pair of ours collided, 2 if the check could not run, else 0 (inherited collisions are WARNs and do not change it).
Each mast ID runs in its own process (a moveit_py instance does not shut down cleanly, and the process leaves with os._exit after writing
its result).
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

MAST_IDS = ("mast_1300", "mast_1350", "mast_1400", "mast_1450")
DEFAULT_SAMPLES = 2000
DEFAULT_SEED = 20261006
REASONS_EXPECTED_TO_TOUCH = ("Adjacent", "Default")
CROSS_CHECK_SAMPLES = 200       # states on which the by-construction pairs are re-evaluated, to test the reasoning (not to certify)
BEGIN_MARKER = "<!-- BEGIN GENERATED ACM -->"
_KEEP_ALIVE = []        # a MoveItPy that is garbage collected segfaults; the worker keeps it until os._exit


def pair_key(a, b):
    return tuple(sorted((a, b)))


def srdf_pairs(path):
    """{pair: reason} for every disable_collisions of an SRDF file."""
    root = ET.parse(path).getroot()
    return {pair_key(d.get("link1"), d.get("link2")): d.get("reason") for d in root.findall("disable_collisions")}


def srdf_imported_pairs(path):
    """The pairs disabled in the SRDF before the generated block: section 1, copied from upstream's v2.0 SRDF (Enactic's own assertion)."""
    with open(path) as f:
        text = f.read()
    head = text.split(BEGIN_MARKER)[0] + "</robot>"
    return {pair_key(d.get("link1"), d.get("link2")) for d in ET.fromstring(head).findall("disable_collisions")}


def joint_table(urdf_text):
    """({joint: (lower, upper)} for the joints that are sampled, {mimic joint: (leader, multiplier, offset)}, [link names])."""
    root = ET.fromstring(urdf_text)
    free, mimic = {}, {}
    for joint in root.findall("joint"):
        kind, name = joint.get("type"), joint.get("name")
        if kind == "fixed":
            continue
        m = joint.find("mimic")
        if m is not None:
            mimic[name] = (m.get("joint"), float(m.get("multiplier", 1.0)), float(m.get("offset", 0.0)))
        elif kind == "continuous":
            free[name] = (-math.pi, math.pi)
        else:
            limit = joint.find("limit")
            free[name] = (float(limit.get("lower")), float(limit.get("upper")))
    return free, mimic, [link.get("name") for link in root.findall("link")]


def sample_states(free, mimic, count, seed):
    """`count` joint states, each {joint: value}, uniform inside the limits (continuous joints in [-pi, pi]); a mimic joint follows its
    leader exactly as the URDF says. numpy's default_rng(seed): the same states at every run and every mast ID."""
    import numpy as np
    rng = np.random.default_rng(seed)
    names = sorted(free)
    lows = np.array([free[n][0] for n in names])
    highs = np.array([free[n][1] for n in names])
    for values in rng.uniform(lows, highs, size=(count, len(names))):
        state = {n: float(v) for n, v in zip(names, values)}
        for joint, (leader, multiplier, offset) in mimic.items():
            state[joint] = state[leader] * multiplier + offset
        yield state


def colliding_pairs(check, pairs):
    """Every pair of `pairs` that collides in the state `check` looks at, by halving. `check(subset)` is True when any pair of the subset
    collides. One call when nothing collides."""
    if not pairs or not check(pairs):
        return []
    if len(pairs) == 1:
        return list(pairs)
    middle = len(pairs) // 2
    return colliding_pairs(check, pairs[:middle]) + colliding_pairs(check, pairs[middle:])


def sample_one_mast(mast_id, samples, seed, srdf_file, hardware_profile="mock"):
    """Run the check for one mast ID inside this process (moveit_py). Returns the result dict."""
    from ament_index_python.packages import get_package_share_directory
    from moveit.core import collision_detection as cd
    from moveit.core.robot_state import RobotState
    from moveit.planning import MoveItPy

    from openamrobot_upperbody_moveit import acm_split
    from openamrobot_upperbody_moveit.moveit_config import moveit_config

    cfg = moveit_config(mast_id, hardware_profile, srdf_file=srdf_file).to_dict()
    cfg["planning_scene_monitor_options"] = {
        "name": "planning_scene_monitor", "robot_description": "robot_description", "joint_state_topic": "/joint_states",
        "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
        "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
        "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
        "wait_for_initial_state_timeout": 0.0}            # 0: do not wait for a /joint_states that nothing publishes here
    cfg["planning_pipelines"] = {"pipeline_names": ["ompl"]}
    cfg["plan_request_params"] = {"planning_attempts": 1, "planning_pipeline": "ompl", "max_velocity_scaling_factor": 0.1,
                                  "max_acceleration_scaling_factor": 0.1, "planning_time": 1.0}
    free, mimic, link_names = joint_table(cfg["robot_description"])
    srdf_file = srdf_file or default_srdf()
    pairs = srdf_pairs(srdf_file)
    imported = srdf_imported_pairs(srdf_file)
    missing = sorted({link for p in pairs for link in p} - set(link_names))
    if missing:
        raise SystemExit(f"the SRDF disables links the model does not have: {missing}")

    # The split. Provenance first (who owns each link), then the kinematic tree (is a joint between the two links).
    owners = acm_split.load_contract_owners(get_package_share_directory("openamrobot_upperbody_description"))
    provenance = acm_split.link_provenance(link_names, owners)
    inherited, ours = acm_split.split_pairs(list(pairs), provenance)
    assert len(inherited) + len(ours) == len(pairs) and not set(inherited) & set(ours)
    never = lambda group: [p for p in group if pairs[p] == "Never"]
    touching = sorted(p for p, reason in pairs.items() if reason in REASONS_EXPECTED_TO_TOUCH)
    unknown = sorted({reason for reason in pairs.values()} - {"Never", *REASONS_EXPECTED_TO_TOUCH})
    ours_never, inherited_never = never(ours), never(inherited)
    by_construction, ours_sampled = acm_split.split_by_kinematics(ours_never, acm_split.parse_urdf(cfg["robot_description"]))

    moveit_py = MoveItPy(node_name=f"validate_acm_{mast_id}", config_dict=cfg)
    _KEEP_ALIVE.append(moveit_py)
    robot_model = moveit_py.get_robot_model()
    state = RobotState(robot_model)
    request = cd.CollisionRequest()               # no contacts: the contact map cannot be read from Python, see the docstring

    def acm_forbidding(subset):
        acm = cd.AllowedCollisionMatrix(link_names, True)        # everything allowed ...
        for a, b in subset:
            acm.set_entry(a, b, False)                          # ... except the pairs under test
        return acm

    started = time.monotonic()
    ours_hits, inherited_hits = {}, {}        # pair -> {"count", "first_sample_index", "sample"}
    touching_hits = {}                        # pair -> count
    with moveit_py.get_planning_scene_monitor().read_only() as scene:
        def check_fn(subset_pairs):
            result = cd.CollisionResult()
            scene.check_collision(request, result, state, acm_forbidding(subset_pairs))
            return result.collision

        # (1) By construction: ONE evaluation, in the default state. A fixed relative transform gives the same answer in every state.
        state.set_to_default_values()
        state.update()
        constructed_colliding = set(colliding_pairs(check_fn, by_construction))
        cross_check = {"states": min(CROSS_CHECK_SAMPLES, samples), "disagreements": []}

        # (2) By sampling: ours (FAIL when it collides) and inherited (WARN), plus the Adjacent/Default pairs that are counted.
        for index, sample in enumerate(sample_states(free, mimic, samples, seed)):
            state.joint_positions = {**state.joint_positions, **sample}
            state.update()
            for pair in colliding_pairs(check_fn, ours_sampled):
                hit = ours_hits.setdefault(pair, {"count": 0, "first_sample_index": index, "sample": sample})
                hit["count"] += 1
            for pair in colliding_pairs(check_fn, inherited_never):
                hit = inherited_hits.setdefault(pair, {"count": 0, "first_sample_index": index, "sample": sample})
                hit["count"] += 1
            for pair in colliding_pairs(check_fn, touching):
                touching_hits[pair] = touching_hits.get(pair, 0) + 1
            if index < cross_check["states"]:
                again = set(colliding_pairs(check_fn, by_construction))
                if again != constructed_colliding:
                    cross_check["disagreements"].append({"sample_index": index, "differs": sorted(map(list, again ^ constructed_colliding))})
    elapsed = time.monotonic() - started

    def report(hits):
        return [{"pair": list(pair), "collided_in_samples": hit["count"], "first_sample_index": hit["first_sample_index"], "seed": seed,
                 "reproducing_sample": hit["sample"],
                 "reproduce": f"seed={seed}, sample index {hit['first_sample_index']} (sample_states(...) yields it), or set these joint values"}
                for pair, hit in sorted(hits.items())]

    inherited_report = report(inherited_hits)
    for item in inherited_report:
        pair = tuple(item["pair"])
        item["origin"] = acm_split.origin_of(pair, imported)
        item["arms"] = acm_split.arm_relation(pair)
        item["why_a_warn"] = "both links are upstream's: an observation about upstream's matrix, not a defect of ours"
    ours_report = report(ours_hits)
    constructed_report = [{"pair": list(p), "evaluated_in": "the default state, once", "collides": True} for p in sorted(constructed_colliding)]
    failing = bool(ours_report or constructed_report or cross_check["disagreements"] or unknown)
    return {
        "mast_id": mast_id, "samples": samples, "seed": seed, "seconds": round(elapsed, 1), "srdf": srdf_file,
        "disabled_pairs": {"total": len(pairs), "never": len(ours_never) + len(inherited_never),
                           "touching_by_design": len(touching), "other_reasons": unknown},
        "our_pairs": {"total": len(ours), "never": len(ours_never), "certified_by_construction": len(by_construction),
                      "sampled": len(ours_sampled), "adjacent_or_default": len(ours) - len(ours_never)},
        "inherited_pairs": {"total": len(inherited), "never": len(inherited_never), "adjacent_or_default": len(inherited) - len(inherited_never),
                            "imported_from_upstream": sum(1 for p in inherited if p in imported)},
        "link_provenance": {"upstream": sorted(l for l, (side, _) in provenance.items() if side == "upstream"),
                            "ours": sorted(l for l, (side, _) in provenance.items() if side == "ours"),
                            "ours_but_not_in_the_frame_contract": sorted(l for l, (side, how) in provenance.items() if how == "unlisted")},
        "by_construction": {"evaluated": "once, in the default state", "pairs": [list(p) for p in by_construction],
                            "colliding": constructed_report, "cross_check_with_sampled_states": cross_check},
        "our_pairs_that_collided": ours_report,
        "our_never_pairs_that_never_collided_by_sampling": [list(p) for p in ours_sampled if p not in ours_hits],
        "inherited_pairs_that_collided": inherited_report,
        "inherited_never_pairs_that_never_collided": [list(p) for p in inherited_never if p not in inherited_hits],
        "adjacent_or_default_pairs_that_touched": len(touching_hits), "adjacent_or_default_pairs_total": len(touching),
        "result": "FAIL" if failing else "PASS",
    }


def default_srdf():
    from openamrobot_upperbody_moveit.moveit_config import srdf_path
    return srdf_path()


def run_one(args):
    result = sample_one_mast(args.mast_id[0], args.samples, args.seed, args.srdf)
    with open(args.json, "w") as f:
        json.dump(result, f)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)           # moveit_py segfaults when the interpreter tears it down; the result is already on disk


def print_mast(result, show_list):
    ours, inh, cons = result["our_pairs"], result["inherited_pairs"], result["by_construction"]
    print(f"\n{result['mast_id']}: {result['samples']} random joint states (seed {result['seed']}), {result['seconds']} s")
    print(f"  disabled in the SRDF: {result['disabled_pairs']['total']} pairs: ours {ours['total']}, inherited {inh['total']}")
    print(f"  ours: {ours['never']} `Never` = {ours['certified_by_construction']} certified by construction (evaluated once) + "
          f"{ours['sampled']} sampled; {ours['adjacent_or_default']} Adjacent/Default (touch by design, not claimed clear)")
    print(f"  inherited: {inh['never']} `Never` (sampled), {inh['adjacent_or_default']} Adjacent/Default, {inh['imported_from_upstream']} imported from upstream")
    for item in cons["colliding"]:
        print(f"  FAIL by construction: {item['pair'][0]} <-> {item['pair'][1]} collides in the default state, and so in every state")
    for item in cons["cross_check_with_sampled_states"]["disagreements"]:
        print(f"  FAIL: the construction reasoning is wrong: sample {item['sample_index']} differs from the one evaluation on {item['differs']}")
    bad = result["our_pairs_that_collided"]
    if bad:
        print(f"  FAIL: {len(bad)} pair(s) of ours disabled as `Never` collided:")
        for item in bad:
            print(f"    {item['pair'][0]} <-> {item['pair'][1]}: collided in {item['collided_in_samples']} sample(s); "
                  f"first at seed {item['seed']}, sample index {item['first_sample_index']}")
            print("      reproducing sample: " + json.dumps({k: round(v, 6) for k, v in item["reproducing_sample"].items()}))
    else:
        print(f"  no counterexample: none of our {ours['sampled']} sampled `Never` pairs collided in {result['samples']} samples at seed "
              f"{result['seed']}, and the {ours['certified_by_construction']} by-construction pairs are clear in the one configuration that decides them")
    for item in result["inherited_pairs_that_collided"]:
        print(f"  WARN: inherited {item['pair'][0]} <-> {item['pair'][1]} ({item['origin']}, {item['arms']}) collided in "
              f"{item['collided_in_samples']} sample(s); seed {item['seed']}, first at sample index {item['first_sample_index']}")
    if not result["inherited_pairs_that_collided"]:
        print(f"  no inherited `Never` pair collided in {result['samples']} samples at seed {result['seed']}")
    if show_list:
        for a, b in result["our_never_pairs_that_never_collided_by_sampling"]:
            print(f"    sampled, no counterexample: {a} <-> {b}")


def run_all(args):
    masts = args.mast_id or list(MAST_IDS)
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        for index, mast in enumerate(masts):
            out = os.path.join(tmp, f"{mast}.json")
            command = [sys.executable, os.path.abspath(__file__), "--worker", "--mast-id", mast, "--samples", str(args.samples),
                       "--seed", str(args.seed), "--json", out]
            if args.srdf:
                command += ["--srdf", args.srdf]
            done = subprocess.run(command, capture_output=True, text=True, env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
            if done.returncode != 0 or not os.path.isfile(out):
                sys.stderr.write(done.stdout[-2000:] + done.stderr[-2000:])
                print(f"validate_acm: the sampling at {mast} could not run (exit {done.returncode})", file=sys.stderr)
                return 2, results
            with open(out) as f:
                results.append(json.load(f))
            print_mast(results[-1], show_list=(index == 0 or args.show_all))
    return (1 if any(r["result"] == "FAIL" for r in results) else 0), results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES, help="random joint states per mast ID (default 2000)")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--mast-id", action="append", choices=MAST_IDS, help="repeatable; default all four")
    parser.add_argument("--srdf", default=None, help="an SRDF to check instead of the installed one (the mutation tests use this)")
    parser.add_argument("--json", default=None, help="write the result here (one mast ID: its result; several: the summary)")
    parser.add_argument("--show-all", action="store_true", help="print the never-collided list for every mast ID, not only the first")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)     # how this script runs one mast ID in a process of its own
    args = parser.parse_args(argv)
    if args.worker:
        return run_one(args)
    status, results = run_all(args)
    warnings = [{"mast_id": r["mast_id"], **w} for r in results for w in r["inherited_pairs_that_collided"]]
    summary = {"samples_per_mast": args.samples, "seed": args.seed, "results": results, "warnings": warnings,
               "result": "ERROR" if status == 2 else ("FAIL" if status == 1 else "PASS")}
    if args.json:
        with open(args.json, "w") as f:
            json.dump(summary, f, indent=1)
    print(f"\nM03: {summary['result']}  ({len(results)} mast ID(s) x {args.samples} samples at seed {args.seed}; {len(warnings)} inherited WARN(s))")
    return status


if __name__ == "__main__":
    sys.exit(main())
