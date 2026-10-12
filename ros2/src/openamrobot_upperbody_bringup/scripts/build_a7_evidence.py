#!/usr/bin/env python3
"""Assemble the A7 evidence report. Called by run_a7_regression.sh after the tests ran.

    build_a7_evidence.py --since EPOCH [--step name=returncode ...] [--repo PATH]
    build_a7_evidence.py --clear

Writes `<evidence dir>/<UTC timestamp>_a7.json` in the openamr.evidence.v1 shape (VERIFICATION.md), `profile: "a7"`,
`evidence_class: "simulation"`: G01 and G02 (the Gazebo package's launch tests), M01 (once per mast ID), M02, M03, M04, the M05 and M06 checks
this step proposes, I01, V01 (still WARN: a person has to answer docs/v01_visual_check.md) and AUX. It reuses the A6 report's machinery
(build_evidence.py and evidence_common.py of the description package, installed next to it) and its rules:

  * a check with no result is NOT_RUN ("no result"), never a pass and never silently a skip; a skipped M01/M02 is no result. NOT_RUN fails the
    exit status like FAIL (build_evidence.py explains the value). Every test CMake registered must have left a result file (check H02), and
    results filed under an ID this report does not list land in AUX instead of being dropped
  * a dirty tree writes NO file (P08): a report records a commit, and a tree with uncommitted changes is no commit. The summary still prints,
    labelled as not recorded, and the exit status is 3 when nothing failed
  * sidecars and xunit files older than --since are ignored

Exit status: 1 if any check is FAIL, 3 if nothing failed but the tree is dirty (nothing recorded), 2 if the report cannot be assembled, else 0.
Everything in this report is `simulation`: it permits no statement about a physical robot, a payload, a torque, or a validated limit.
"""
import argparse
import datetime
import hashlib
import json
import os
import sys
from collections import defaultdict

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def _description_scripts():
    path = os.path.join(get_package_prefix("openamrobot_upperbody_description"), "lib", "openamrobot_upperbody_description")
    if path not in sys.path:
        sys.path.insert(0, path)
    return path


_description_scripts()
import build_evidence as be  # noqa: E402
import evidence_common as ev  # noqa: E402

REPO = be.REPO
PROFILE = "a7"
EVIDENCE_CLASS = "simulation"
MAST_IDS = ("mast_1300", "mast_1350", "mast_1400", "mast_1450")
A7_CHECKS = [
    ("G01", "combined robot in Gazebo: arm controllers, arm goal, Nav2-facing topics unchanged"),
    ("G02", "grippers close in Gazebo and finger_joint2 follows finger_joint1 in the physics (Bullet-Featherstone, passed explicitly)"),
    ("M01", "named poses plan and execute on mock at every release-candidate mast ID"),
    ("M02", "named poses plan and execute in Gazebo at mast_1350"),
    ("M03", "ACM: every disabled pair has a reason that can be shown (upstream-declared, Adjacent, ConstantTransform); every other pair is enabled"),
    ("M04", "a goal inside the nearest reachable structure volume (mast_base_link) is rejected by planning (the base body is out of the arms' reach)"),
    ("M05", "per-hand gripper values: closed is each hand's own far limit, opposite signs (ADR-009 point 5); the single-number version fails both ways"),
    ("M06", "named-pose plans need some acceleration limit and are insensitive to its value across three orders of magnitude (the limits stay placeholders)"),
    ("L01", "launch test: five controllers active on mock, one trajectory followed"),
    ("H01", "every test file is registered and every launch test checks that its processes started and exited cleanly"),
    ("H02", "every test CMake registered left a result file (nothing registered but not run)"),
    ("I01", "installed packages match the source the tests and this report describe"),
    ("V01", "a person has looked at the simulator and RViz and answered the checklist (eight items)"),
    ("AUX", "supporting tests and steps (SRDF, imported elements, ACM record, config files, harness, colcon test of the packages)"),
]
PACKAGES = {"gazebo": "openamrobot_upperbody_gazebo", "moveit": "openamrobot_upperbody_moveit", "bringup": "openamrobot_upperbody_bringup"}
EXPECTED_STEPS = ("colcon_test_gazebo", "colcon_test_moveit", "colcon_test_bringup")
# A check recorded by one sidecar per run, and the runs it must have. M01 is once per mast ID: the cut order in the prompt allows fewer
# (mast_1350 plus one other, with the reason recorded) through A7_M01_MASTS, and the report then says so.
SIDECAR_REQUIRED = {"G01", "G02", "M01", "M02", "M03", "M04", "M05", "M06"}
REFUSED_DIRTY = be.REFUSED_DIRTY


def expected_m01_masts():
    masts = tuple(m for m in os.environ.get("A7_M01_MASTS", ",".join(MAST_IDS)).split(",") if m)
    unknown = [m for m in masts if m not in MAST_IDS]
    if unknown:
        raise ev.EvidenceError(f"A7_M01_MASTS names {unknown}; accepted: {', '.join(MAST_IDS)}")
    return masts


def all_fresh_sidecars(directory, since):
    """{check id: [sidecar dicts]}: unlike evidence_common.read_sidecars, EVERY file of an id (M01 has one per mast ID)."""
    out = defaultdict(list)
    for filename in sorted(os.listdir(directory)):
        if ev.SIDECAR_RE.match(filename) and os.path.getmtime(os.path.join(directory, filename)) >= since:
            with open(os.path.join(directory, filename)) as f:
                doc = json.load(f)
            out[doc["id"]].append({**doc, "file": filename})
    return out


def run_summary(details):
    clamp = details.get("clamp") or {}
    return {"poses_ok": details.get("poses_ok"), "poses_run": details.get("poses_run"), "worst_final_error_rad": details.get("worst_final_error_rad"),
            "clamp": {k: clamp.get(k) for k in ("present", "epsilon_rad", "messages", "activations", "episodes", "max_magnitude_rad",
                                                "max_magnitude_joint", "over_epsilon", "max_over_epsilon_rad")}}


def clamp_totals(runs):
    """The clamp's counters across the runs of M01/M02: how often it fired, the largest magnitude it clamped, and any reading it did not clamp."""
    rows = [(name, r["clamp"]) for name, r in runs.items() if r.get("clamp")]
    present = [(n, c) for n, c in rows if c.get("present")]
    return {"epsilon_rad": next((c["epsilon_rad"] for _, c in present), None), "runs_with_a_clamp": len(present), "runs": len(rows),
            "activations": sum(c["activations"] for _, c in present), "episodes": sum(c["episodes"] for _, c in present),
            "max_magnitude_rad": max((c["max_magnitude_rad"] for _, c in present), default=None),
            "over_epsilon": sum(c["over_epsilon"] for _, c in present),
            "max_over_epsilon_rad": max((c["max_over_epsilon_rad"] for _, c in present), default=None)}


def combine_multi(check_id, counts, docs, expected_masts, missing=None):
    """-> (result, details) for a check recorded once per mast ID (M01) or once (M02, with its mast in the details)."""
    details = {}
    if counts:
        details = {"tests": counts["tests"], "failed": counts["failures"], "skipped": counts["skipped"]}
        if counts["failed_tests"]:
            details["failed_tests"] = counts["failed_tests"][:40]
    by_mast = {d["details"].get("mast_id"): d for d in docs}
    runs, problems, no_result = {}, [], []
    if missing:
        details["tests_without_a_result"] = list(missing)[:40]
    for mast in expected_masts:
        doc = by_mast.get(mast)
        if doc is None:
            problems.append(f"no result for {mast}")
            no_result.append(mast)
            continue
        runs[mast] = {"result": doc["result"], **run_summary(doc["details"])}
        if doc["result"] != "PASS":
            problems.append(f"{mast}: {doc['result']}")
    details["expected_mast_ids"] = list(expected_masts)
    details["runs"] = runs
    if check_id == "M01" and tuple(expected_masts) != MAST_IDS:
        details["reduced_set"] = f"the cut order reduced M01 to {', '.join(expected_masts)} (A7_M01_MASTS); the other mast IDs were not run"
    # The clamp's counters are part of the result: no counters is no evidence that the joint state was only ever float noise away from a limit,
    # and a reading beyond the epsilon is something other than float noise moving a joint, which a PASS must not hide.
    totals = clamp_totals(runs)
    details["clamp_totals"] = totals
    for name, run in runs.items():
        if not (run.get("clamp") or {}).get("present"):
            problems.append(f"{name}: no joint-state clamp counters in the run")
    if counts and counts["failures"]:
        return "FAIL", {**details, "problems": problems or ["a test of this check failed"]}
    if any(doc["result"] == "FAIL" for doc in docs):
        return "FAIL", {**details, "problems": problems}
    if missing:
        return be.NOT_RUN, {**details, "problems": problems + [f"registered test(s) left no result: {', '.join(list(missing)[:3])}"]}
    if no_result:
        return be.NOT_RUN, {**details, "problems": problems}
    if problems:
        return "FAIL", {**details, "problems": problems}
    if details.get("reduced_set"):
        return "WARN", {**details, "problems": [details["reduced_set"]]}
    if totals["over_epsilon"]:
        return "WARN", {**details, "problems": [f"the clamp met {totals['over_epsilon']} reading(s) beyond its epsilon (largest {totals['max_over_epsilon_rad']:.3g} "
                                                f"rad): not float noise, passed through unclamped; the poses still reached their goals, but look at why"]}
    return "PASS", details


def moveit_identity():
    """What the MoveIt configuration IS, as hashes: the SRDF (which carries the generated ACM) and the other config files, and the ACM's own
    fingerprint (the contract: the matrix as a set of pairs, whatever the file's order)."""
    share = get_package_share_directory("openamrobot_upperbody_moveit")
    import xml.etree.ElementTree as ET
    from openamrobot_upperbody_moveit.acm_split import acm_fingerprint, acm_pairs
    out = {"acm_fingerprint": acm_fingerprint(acm_pairs(ET.parse(os.path.join(share, "config", "openamrobot.srdf")).getroot()))}
    for name in ("openamrobot.srdf", "joint_limits.yaml", "moveit_controllers.yaml", "kinematics.yaml", "ompl_planning.yaml",
                 "pilz_cartesian_limits.yaml", "acm/acm_generation.yaml"):
        with open(os.path.join(share, "config", name), "rb") as f:
            out[name] = "sha256:" + hashlib.sha256(f.read()).hexdigest()
    return out


def limitations(sidecars):
    m03 = (sidecars.get("M03") or [{}])[0]
    m03_details = m03.get("details", {})
    out = [
        "`simulation` names the evidence class: every result here is how the model behaves in Gazebo Harmonic or on mock_components, planned by "
        "MoveIt 2.12.4 under OpenAMRobot's own declared limits. Every claim reads 'plans and executes in simulation under declared OpenAMRobot "
        "limits'; nothing here is a validated limit, a payload, a torque, or a statement about a physical robot",
        "the acceleration limits are engineering defaults (source: engineering_default, status: placeholder; rule: velocity / 2 s) written because "
        "upstream declares none (openamrobot-manipulation#10, ADR-009); M06 shows the plans need SOME limit and do not depend on its value",
        "MoveIt never sees base_link's 4.18 kg inertial: KDL drops the inertia of a root link (W01)",
        "`park` is a placeholder copy of upstream's `home`, until the safe-pose definition lands under the manipulation action contract",
        "the gripper controller is still the deprecated position_controllers/GripperActionController serving GripperCommand (option 1 of "
        "ADR-009 point 4): this report does not choose between the three gripper interfaces (manipulation#11)",
        "G02 and M02 pass the Bullet-Featherstone engine explicitly; sim.launch.py defaults to DART, the engine platform-sw tuned the base on, on "
        "which a closed gripper pose does not close. The base's contact behaviour was compared on neither engine",
        "the arms' position interface in gz_ros2_control is a velocity servo, so a final error under 0.01 rad in M02 says nothing about the real "
        "arm's controller, gains, compliance or latency",
        "MoveIt 2.12.4 refuses a start state any amount outside a joint limit ('Joint ... from the starting state is outside bounds by'), and a "
        "simulated joint resting on its limit (openarm_*_joint4 = 0, the home pose) reads back a hair past it. The joint state move_group plans "
        "from is therefore clamped to the limit by joint_state_clamp.py, by at most its epsilon, and only that topic: no limit was widened and no "
        "pose moved. The clamp's counters are in M01/M02 (clamp_totals) and a reading beyond the epsilon is passed through, never clamped",
        "move_group exits with SIGSEGV when it is stopped (MoveIt 2.12.4); the checks allow that code for that one process and no other",
        "the arms cannot reach the platform's base body: M04's refused goal is inside mast_base_link's volume, the nearest base-side volume a "
        "left gripper can be put in",
        "the collision matrix is DERIVED, not sampled (scripts/derive_acm.py): a pair is disabled only if upstream declares it, it is a structural "
        "neighbour (Adjacent), or no movable joint lies between its links and it is clear in the one evaluation that settles every configuration "
        "(ConstantTransform); every other pair is enabled. The Setup Assistant's sampler, which has no seed and gave a different matrix on every "
        "run, is no longer used. The checked-in SRDF is the contract and M03 validates that file",
        "M03's sampled part is information about the ENABLED pairs, never a reason: sampling falsifies and does not certify. It lists the enabled pairs "
        "that collide in random states and how often (for A4); nothing in the matrix rests on it",
        be.V01_LIMITATION,
        "no physical evidence; simulation only. This report is not a statement about hardware",
    ]
    if m03.get("result") == "FAIL":
        failing = m03_details.get("failures", {})
        pairs = sorted({f"{p['pair'][0]} <-> {p['pair'][1]}" for item in failing.values() for key in ("unjustified", "bad_labels") for p in item.get(key, [])}
                       | {f"{p[0]} <-> {p[1]}" for item in failing.values() for key in ("constant_transform_colliding", "enabled_constant_pairs_colliding")
                          for p in item.get(key, [])})
        out.insert(0, "M03 FAILS: the collision matrix is wrong for " + ", ".join(pairs) + ". A disabled pair with no reason that can be shown, a false label, "
                      "a constant pair that collides, or a permanent collision that is enabled: the pair is the finding, and it is not re-disabled on statistics")
    return out


def build(args):
    share = ev.share_dir()
    workspace = ev.workspace_root(share)
    edir = ev.evidence_dir(share)
    repo = args.repo or os.path.join(workspace, "src", REPO)
    steps = dict(s.split("=", 1) for s in args.step)
    result_dirs = [os.path.join(workspace, "build", p, "test_results", p) for p in PACKAGES.values()]
    counts, stale_xunit = be.read_xunit_dirs(result_dirs, args.since)
    sidecars = all_fresh_sidecars(edir, args.since)
    accounted = be.accounting(workspace, list(PACKAGES.values()), args.since)
    claimed = {i for i, _ in A7_CHECKS}
    unclaimed = be.fold_unclaimed(counts, claimed)
    missing = be.missing_by_id(accounted["problems"], claimed)

    source = lambda pkg: os.path.join(repo, "ros2", "src", pkg)
    differing = []
    for pkg, names in (("openamrobot_upperbody_description", ("urdf", "config", "launch", "meshes")),
                       ("openamrobot_upperbody_control", ("config",)),
                       ("openamrobot_upperbody_gazebo", ("launch", "config", "worlds")),
                       ("openamrobot_upperbody_moveit", ("launch", "config")),
                       ("openamrobot_upperbody_bringup", ("launch",))):
        differing += [f"{pkg}/{d}" for d in be.trees_differ(source(pkg), ev.package_share(pkg), names=names)]

    masts = expected_m01_masts()
    checks = []
    for check_id, name in A7_CHECKS:
        docs = sidecars.get(check_id, [])
        if check_id == "M01":
            result, details = combine_multi("M01", counts.get("M01"), docs, masts, missing.get("M01"))
        elif check_id == "M02":
            result, details = combine_multi("M02", counts.get("M02"), docs, ("mast_1350",), missing.get("M02"))
        elif check_id == "V01":
            result, details = "WARN", be.v01_details(8)
        elif check_id == "I01":
            result, details = ("FAIL", {"differing": differing, "hint": "run colcon build, then run_a7_regression.sh again"}) if differing \
                else ("PASS", {"compared": ["description: urdf, config, launch, meshes", "control: config", "gazebo: launch, config, worlds",
                                            "moveit: launch, config", "bringup: launch"]})
        elif check_id == "H02":
            result, details = (be.NOT_RUN, {"reason": f"{len(accounted['problems'])} registered test(s) left no usable result"
                                                      if accounted["problems"] else "no registered tests were found to account for",
                                            "problems": accounted["problems"][:20], "registered": accounted["registered"],
                                            "with_a_result": accounted["with_a_result"]}) if (accounted["problems"] or not accounted["registered"]) \
                else ("PASS", {"registered": accounted["registered"], "with_a_result": accounted["with_a_result"],
                               "packages": list(PACKAGES.values())})
        elif check_id == "AUX":
            result, details = ("PASS", {}) if not counts.get("AUX") else be.combine("AUX", counts["AUX"], None, missing.get("AUX"))
        else:
            result, details = be.combine(check_id, counts.get(check_id), docs[0] if docs else None, missing.get(check_id))
            if check_id in SIDECAR_REQUIRED and not docs and result in ("PASS", "WARN", "SKIP"):
                result, details["reason"] = be.NOT_RUN, "no sidecar: the test that records this check did not complete"
        if check_id == "AUX":
            if unclaimed:
                details["unclaimed_check_ids"] = {cid: {"tests": r["tests"], "failures": r["failures"]} for cid, r in unclaimed.items()}
            problems = {}
            for step in EXPECTED_STEPS:
                if steps.get(step) is None:
                    problems[step] = "step was not run"
                elif steps[step] != "0":
                    problems[step] = f"exited {steps[step]} (the failing checks above say which test)"
            if stale_xunit:
                problems["stale_xunit"] = stale_xunit
            aux_counts = counts.get("AUX")
            if aux_counts:
                details.update({"tests": aux_counts["tests"], "failed": aux_counts["failures"], "skipped": aux_counts["skipped"]})
                if aux_counts["failures"]:
                    problems["failed_tests"] = aux_counts["failed_tests"][:40]
            details["steps"] = steps
            if problems:
                result, details["step_problems"] = "FAIL", problems
        checks.append({"id": check_id, "name": name, "result": result, "details": details})

    with open(os.path.join(share, "config", "datums.yaml")) as f:
        datums = yaml_load(f)
    with open(os.path.join(share, "config", "mast_ids.yaml")) as f:
        mast_ids = yaml_load(f)
    commit, commit_source = ev.upstream_commit_and_source(workspace)
    import print_config_id as pci
    matrix = [c for c in ev.expansion_matrix(mast_ids) if c[1] == "openarm" and c[2] in ("mock", "gazebo")]
    config_ids = {ev.expansion_label(*c): pci.config_id_of(share, commit, *c) for c in matrix}
    provenance = defaultdict(int)
    for d in datums["datums"].values():
        provenance[d["status"]] += 1

    dirty, dirty_files = ev.git_dirty(repo)
    upstream = {"openarm_description": commit}
    for name in ("openarm_ros2",):
        head = ev.git_head(ev.upstream_checkout(workspace, name))
        if head:
            upstream[name] = head
    simulator = be.simulator_versions()
    try:
        import subprocess
        moveit_version = subprocess.run(["dpkg-query", "-W", "-f=${Version}", "ros-jazzy-moveit-core"], capture_output=True, text=True).stdout or "unknown"
    except OSError:
        moveit_version = "unknown"
    now = datetime.datetime.now(datetime.timezone.utc)
    doc = {
        "schema": "openamr.evidence.v1", "repo": REPO, "git_hash": ev.git_head(repo), "dirty": dirty,
        "timestamp": now.strftime("%Y-%m-%dT%H:%M:%SZ"), "profile": PROFILE, "evidence_class": EVIDENCE_CLASS,
        "simulator": {**simulator, "moveit": moveit_version}, "upstream": upstream, "datums_revision": datums["revision"],
        "checks": checks, "config_ids": config_ids, "moveit_config": moveit_identity(),
        "provenance": {k: provenance[k] for k in ("placeholder", "provisional", "verified")},
        "limitations": limitations(sidecars),
        # What the joint-state clamp did, summed over M01/M02. Not a result of its own: a reading beyond the epsilon makes M01/M02 WARN.
        "clamp": {c["id"]: c["details"].get("clamp_totals") for c in checks if c["id"] in ("M01", "M02")},
    }
    if dirty:
        doc["dirty_files"] = dirty_files
        return doc, None
    path = os.path.join(edir, now.strftime("%Y%m%dT%H%M%SZ") + f"_{PROFILE}.json")
    with open(path, "w") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")
    return doc, path


def yaml_load(f):
    import yaml
    return yaml.safe_load(f)


def summary(doc, path):
    lines = ["", f"A7 evidence  {doc['repo']} @ {(doc['git_hash'] or '?')[:7]}{' (dirty)' if doc['dirty'] else ''}  class {doc['evidence_class']}  "
                 f"MoveIt {doc['simulator'].get('moveit')}"]
    for c in doc["checks"]:
        extra = (c["details"].get("problems") or [c["details"].get("reason") or ""])[0] if c["result"] in ("FAIL", "WARN", be.NOT_RUN) else ""
        if not extra and c["details"].get("failed_tests"):
            extra = "failing: " + ", ".join(c["details"]["failed_tests"][:3])
        tests = f"  [{c['details']['tests']} tests]" if "tests" in c["details"] else ""
        lines.append(f"  {c['result']:<4}  {c['id']:<4} {c['name'][:84]}{tests}{('  -- ' + str(extra)[:90]) if extra else ''}")
    counts = defaultdict(int)
    for c in doc["checks"]:
        counts[c["result"]] += 1
    lines += ["", "Limitations"] + [f"  - {text}" for text in doc["limitations"][:4]] + ["  - ... (all in the evidence file)"]
    if path is None:
        files = doc["dirty_files"]
        lines += ["", f"Evidence: NOT WRITTEN. {doc['repo']} has uncommitted changes ({len(files)}: {', '.join(files[:5])}{', ...' if len(files) > 5 else ''}), "
                      "so a report would describe no commit.",
                  "          Commit, then run this again. Nothing above is a record; the evidence directory only holds clean-tree runs."]
    else:
        lines += ["", f"Evidence: {path}"]
    lines += ["Result: " + ("FAIL" if counts["FAIL"] else ("NOT_RUN" if counts[be.NOT_RUN] else "no FAIL")) +
              "  (" + ", ".join(f"{counts[r]} {r}" for r in be.RESULT_ORDER) + ")"]
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--since", type=float, help="epoch seconds: results older than this are stale (required to build)")
    parser.add_argument("--clear", action="store_true", help="delete the previous run's G* and M* sidecars from the evidence directory, print it, and stop")
    parser.add_argument("--step", action="append", default=[], metavar="NAME=RC", help="return code of a step run_a7_regression.sh ran")
    parser.add_argument("--repo", default=None, help=f"the {REPO} checkout (default <workspace>/src/{REPO})")
    args = parser.parse_args(argv)
    if args.clear:
        directory = ev.evidence_dir()
        ev.clear_sidecars(directory, only=lambda filename: filename[0] in "GM")
        print(directory)
        return 0
    if args.since is None:
        parser.error("--since is required")
    try:
        doc, path = build(args)
    except ev.EvidenceError as err:
        print(f"build_a7_evidence: {err}", file=sys.stderr)
        return 2
    print(summary(doc, path))
    if any(c["result"] in be.FAILING for c in doc["checks"]):
        return 1
    return REFUSED_DIRTY if path is None else 0


if __name__ == "__main__":
    sys.exit(main())
