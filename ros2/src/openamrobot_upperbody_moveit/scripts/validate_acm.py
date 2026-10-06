#!/usr/bin/env python3
"""M03: validate the SRDF's collision matrix by sampling, not by trusting it.

    ros2 run openamrobot_upperbody_moveit validate_acm.py [--samples 2000] [--seed N] [--mast-id ID ...] [--srdf PATH] [--json OUT]

For each mast ID (all four by default) it draws N random joint states inside the joint limits of the combined model (moveit_py) and,
for every state, checks collision IGNORING the SRDF's disabled list: the check is built from an allowed-collision matrix of its own,
which allows every pair except the ones under test, so nothing the SRDF says is consulted. A pair that is disabled in the SRDF with
the reason `Never` and collides in any sample is a FAIL, reported by name with the sample that reproduces it. The pairs disabled as
`Never` that did not collide in any sample are printed too: that is the expected outcome, and printing it is what makes the check
legible. Pairs disabled as `Adjacent` or `Default` are expected to touch (that is why they are disabled), so they are counted and
never failed.

If this finds a disabled pair that collides, the ACM is wrong and WHICH pair is wrong is the finding: do not re-enable it and move on.

Why a bisection: moveit_py's CollisionResult exposes `collision` and `contact_count` but not the contact map (the Contact type is not
bound), so the colliding pairs are found by group testing: one check with only the pairs under test forbidden, and, on a hit, halving
the set until single pairs are left. Reproducible: states come from numpy's default_rng(seed), the same seed at every mast ID.

Exit status: 1 if any `Never` pair collided, 2 if the check could not run, else 0. Each mast ID runs in its own process (a moveit_py
instance does not shut down cleanly, and the process leaves with os._exit after writing its result).
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
_KEEP_ALIVE = []        # a MoveItPy that is garbage collected segfaults; the worker keeps it until os._exit


def pair_key(a, b):
    return tuple(sorted((a, b)))


def srdf_pairs(path):
    """{pair: reason} for every disable_collisions of an SRDF file."""
    root = ET.parse(path).getroot()
    return {pair_key(d.get("link1"), d.get("link2")): d.get("reason") for d in root.findall("disable_collisions")}


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
    """Run the sampling for one mast ID inside this process (moveit_py). Returns the result dict."""
    from moveit.core import collision_detection as cd
    from moveit.core.robot_state import RobotState
    from moveit.planning import MoveItPy

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
    pairs = srdf_pairs(srdf_file or default_srdf())
    never = sorted(p for p, reason in pairs.items() if reason == "Never")
    touching = sorted(p for p, reason in pairs.items() if reason in REASONS_EXPECTED_TO_TOUCH)
    unknown = sorted({reason for reason in pairs.values()} - {"Never", *REASONS_EXPECTED_TO_TOUCH})
    missing = sorted({link for p in pairs for link in p} - set(link_names))
    if missing:
        raise SystemExit(f"the SRDF disables links the model does not have: {missing}")

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
    never_hits = {}            # pair -> {"count", "first_sample_index", "sample"}
    touching_hits = {}         # pair -> count
    with moveit_py.get_planning_scene_monitor().read_only() as scene:
        def check_fn(subset_pairs):
            result = cd.CollisionResult()
            scene.check_collision(request, result, state, acm_forbidding(subset_pairs))
            return result.collision

        for index, sample in enumerate(sample_states(free, mimic, samples, seed)):
            state.joint_positions = {**state.joint_positions, **sample}
            state.update()
            for pair in colliding_pairs(check_fn, never):
                hit = never_hits.setdefault(pair, {"count": 0, "first_sample_index": index, "sample": sample})
                hit["count"] += 1
            for pair in colliding_pairs(check_fn, touching):
                touching_hits[pair] = touching_hits.get(pair, 0) + 1
    elapsed = time.monotonic() - started
    return {
        "mast_id": mast_id, "samples": samples, "seed": seed, "seconds": round(elapsed, 1),
        "srdf": srdf_file or default_srdf(),
        "disabled_pairs": {"total": len(pairs), "never": len(never), "touching_by_design": len(touching), "other_reasons": unknown},
        "never_pairs_that_collided": [
            {"pair": list(pair), "collided_in_samples": hit["count"], "first_sample_index": hit["first_sample_index"],
             "reproducing_sample": hit["sample"], "reproduce": f"seed={seed}, sample index {hit['first_sample_index']} "
                                                              "(sample_states(...) yields it), or set these joint values"}
            for pair, hit in sorted(never_hits.items())],
        "never_pairs_that_never_collided": [list(p) for p in never if p not in never_hits],
        "adjacent_or_default_pairs_that_touched": len(touching_hits), "adjacent_or_default_pairs_total": len(touching),
        "result": "FAIL" if never_hits or unknown else "PASS",
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
    n_never = result["disabled_pairs"]["never"]
    bad = result["never_pairs_that_collided"]
    print(f"\n{result['mast_id']}: {result['samples']} random joint states (seed {result['seed']}), {result['seconds']} s")
    print(f"  disabled in the SRDF: {result['disabled_pairs']['total']} pairs, of which {n_never} `Never` and "
          f"{result['disabled_pairs']['touching_by_design']} Adjacent/Default (touch by design: "
          f"{result['adjacent_or_default_pairs_that_touched']} of them did)")
    if bad:
        print(f"  FAIL: {len(bad)} pair(s) disabled as `Never` collided:")
        for item in bad:
            print(f"    {item['pair'][0]} <-> {item['pair'][1]}: collided in {item['collided_in_samples']} sample(s); "
                  f"first at sample index {item['first_sample_index']}")
            print("      reproducing sample: " + json.dumps({k: round(v, 6) for k, v in item["reproducing_sample"].items()}))
    else:
        print(f"  PASS: none of the {n_never} `Never` pairs collided in any sample")
    never_ok = result["never_pairs_that_never_collided"]
    print(f"  `Never` pairs that never collided: {len(never_ok)} of {n_never}")
    if show_list:
        for a, b in never_ok:
            print(f"    {a} <-> {b}")


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
    summary = {"samples_per_mast": args.samples, "seed": args.seed, "results": results,
               "result": "ERROR" if status == 2 else ("FAIL" if status == 1 else "PASS")}
    if args.json:
        with open(args.json, "w") as f:
            json.dump(summary, f, indent=1)
    print(f"\nM03: {summary['result']}  ({len(results)} mast ID(s) x {args.samples} samples)")
    return status


if __name__ == "__main__":
    sys.exit(main())
