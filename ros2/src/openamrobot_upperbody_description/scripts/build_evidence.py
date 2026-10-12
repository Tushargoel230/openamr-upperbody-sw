#!/usr/bin/env python3
"""Assemble the A6 evidence report. Called by verify_a6.sh after `colcon test` and check_urdf_matrix.py have run.

    build_evidence.py --since EPOCH [--profile dev|gazebo] [--step name=returncode ...] [--repo PATH]
                      [--results-dir PATH] [--bringup-results-dir PATH] [--gazebo-results-dir PATH]

Writes `<evidence dir>/<UTC timestamp>_<profile>.json` in the openamr.evidence.v1 shape of VERIFICATION.md and prints a
summary. Profile `dev` (default, scripts/verify_a6.sh) is the A6 report, evidence_class `static+mock`. Profile `gazebo`
(scripts/verify_gazebo.sh) is the Gazebo report, evidence_class `simulation`: G01 and G02 from the launch tests of
openamrobot_upperbody_gazebo, I01 for that package, and V01, which stays WARN until a person has answered the checklist. Exit code is 1 if any check is FAIL, 2 if the evidence cannot be assembled, and 3 if every check passed but the
working tree is dirty.

A dirty tree writes NO file (decided P09). A report records `git_hash`, and a tree with uncommitted changes is not that
commit, so a file written from one is an untrustworthy run lying next to the trustworthy ones, and "which one is the
record" would be something a reviewer has to know rather than something the file says. The summary is still printed so a
developer iterating before a commit sees every result; it is labelled as not recorded. The evidence directory therefore
only ever holds runs of a clean commit, with `dirty: false`.

Where each result comes from
  - pytest results (colcon's xunit files, from this package and from openamrobot_upperbody_bringup, whose launch test
    is L01) are mapped to a check ID by the `test_<id>_` prefix of the test function, else of
    its module. A failing or erroring test fails its ID.
  - a check's sidecar (`<ID>_<slug>.json` in the evidence directory, written by the test or by check_urdf_matrix.py) adds
    its details and can raise a WARN. Sidecars and xunit files older than --since are ignored, so a stale result from an
    earlier run is never mixed into this one.
  - a check with no test and no sidecar is NOT_RUN ("no result"), except those listed in NOT_IMPLEMENTED, which are SKIP
    with the reason: SKIP is a statement, a missing result is not.
  - NOT_RUN is a result value of its own (P12A): the check's backing test did not run to a result. It is not FAIL (nothing was observed to be
    wrong) and not SKIP (nobody said it would not run), it is never PASS, and it fails the exit status like FAIL does.
  - every test CMake registered for the packages a report covers (`ctest -N`) must have left a fresh result file, whatever its name says it
    is: a registered test with no result, a package with no tests registered, and a result file produced by ament's own `missing_result`
    stand-in (a test that timed out or died by a signal) are all listed by check H02 and make the checks they back NOT_RUN. Results whose test
    names carry a check ID this report does not list are not dropped: they land in AUX, and fail it.
Nothing here decides a PASS on its own: the report only combines what the tests recorded, and a PASS needs a test that ran behind it.
"""
import argparse
import datetime
import filecmp
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

import yaml

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import evidence_common as ev  # noqa: E402
import print_config_id as pci  # noqa: E402

REPO = "openamr-upperbody-sw"

# Every A6 check ID in VERIFICATION.md (T01-T16, L01, E01), in order, then the extra IDs this report also carries.
A6_CHECKS = [
    ("T01", "expansion matrix"),
    ("T02", "check_urdf accepts every T01 expansion"),
    ("T03", "single root, unique link names"),
    ("T04", "frame contract"),
    ("T05", "shoulder height per mast id"),
    ("T06", "arg validation"),
    ("T07", "every link with an inertial or visual has a collision or is collision_free"),
    ("T08", "collision mesh is the same file as the visual mesh"),
    ("T09", "massed links: mass > 0, principal moments > 0, triangle inequality, tensor PD"),
    ("T10", "modelled mass by owner vs gross_mass_limit"),
    ("T11", "planar base joint absent by default, planar world -> base_link when enabled"),
    ("T12", "datum schema and provenance rules"),
    ("T13", "datum references; visualization constants"),
    ("T14", "mock profile has no OpenArmHW plugin"),
    ("T15", "gazebo profile uses GazeboSimSystem for arm joints"),
    ("T16", "real profile expands with OpenArmHW and its CAN parameters"),
    ("L01", "launch test: five controllers active on mock, one trajectory followed"),
    ("E01", "assembled envelope height, width, depth (lower bound)"),
]
EXTRA_CHECKS = [
    ("C01", "config id is sha256:<64 hex> and accepted by the C9 pattern"),
    ("C02", "upstream pin file agrees with the upstream checkout"),
    ("W01", "base_link inertia not seen by KDL consumers"),
    ("W02", "KDL converts base_planar_joint to a fixed joint"),
    ("REV", "datums.yaml content change carries a revision change"),
    ("H01", "every test file is registered and every launch test checks that its processes started and exited cleanly"),
    ("H02", "every test CMake registered left a result file (nothing registered but not run)"),
    ("V01", "RViz launch at mast_1350"),
    ("I01", "installed package matches the source the tests and this report describe"),
    ("AUX", "supporting tests and steps (urdf_utils, datums_report, s4_sensors, evidence_dir, build_evidence, scripts)"),
]
# The `gazebo` profile's checks (scripts/verify_gazebo.sh). Everything static is already covered by the `dev` report.
GAZEBO_CHECKS = [
    ("G01", "combined robot in Gazebo: arm controllers, arm goal, Nav2-facing topics unchanged"),
    ("G02", "grippers close in Gazebo and finger_joint2 follows finger_joint1 in the physics"),
    ("H02", "every test CMake registered left a result file (nothing registered but not run)"),
    ("I01", "installed packages match the source the tests and this report describe"),
    ("V01", "a person has looked at the simulator and RViz and answered the checklist"),
    ("AUX", "supporting steps (colcon test of the gazebo package)"),
]
GAZEBO_PKG = "openamrobot_upperbody_gazebo"
PROFILES = {"dev": "static+mock", "gazebo": "simulation"}
# A check that has no test yet is listed here with the reason, and reports SKIP. Empty since P06 landed T14-T16 and L01.
# (C02 SKIPs by its own sidecar, only when there is no upstream checkout; that is a result, not a missing test.)
NOT_IMPLEMENTED = {}
BRINGUP_PKG = "openamrobot_upperbody_bringup"

# V01 after a person looked (P14; the architecture chat's V01_MAST_VISUAL finding, recorded in docs/v01_visual_check.md). It stays WARN, and
# the report says in words what the looking found. Two things about it are deliberate. It is not PASS: PASS needs every checklist item answered,
# and only item 1 (the mast) has an answer on record. And it is not recorded as a check that found nothing: its value was that it found two
# defects no automated check was looking for. build_a7_evidence.py reads these constants, so the three reports cannot tell different stories.
V01_FINDINGS = [
    {"id": "V01-F1", "item": 1,
     "found": "the mast profile is probably rotated 90 degrees: mast_section [0.060, 0.120] puts 60 mm fore-aft and 120 mm lateral; option B "
              "(P-03 rev18.9 item 18) orients the same HFS6-60120 cross-section (its 340 mm arm profile, not a full-height mast) 120 mm "
              "fore-aft and 60 mm lateral, the stiff direction against the arms' forward cantilever. The value is right. Its axis "
              "assignment was only a prose note in the datum ('orientation unconfirmed') and no test asserts it",
     "proposed_rule": "every datum that is a pair or triple of dimensions declares its axis convention, and a test asserts the assignment",
     "not_fixed_because": "mast.xacro is replaced when option B is adopted; the arm mounts at y = +/-0.060 are coupled to this orientation, so "
                          "turning the box alone would be wrong"},
    {"id": "V01-F2", "item": 1,
     "found": "mast_mass is 3.0 kg (placeholder) for 1.2365 m of HFS6-60120: about 337 kg/m3 over the bounding box, low by roughly 2.5 to 4 "
              "times for an aluminium extrusion (an estimate; the MISUMI datasheet is the source). Labelled placeholder, so nothing lied, but "
              "the placeholder is implausible and nothing checked plausibility",
     "proposed_rule": "the density implied by a mass over its bounding-box volume must fall in a sane range or WARN, placeholders included",
     "not_fixed_because": "the mast and its mass are replaced when option B is adopted; the figure to land needs the datasheet"},
]
V01_ANSWERED = {"1": "the mast reads as a thin spike. Its dimensions are those of an HFS6-60120 profile 14 mm shorter than the 1250 mm Alex "
                     "specified on 28 Sep, so the appearance is an accurate thin extrusion and not a sizing defect; its orientation and its "
                     "mass are defects (V01-F1, V01-F2)"}
V01_READING = ("The manual check found what the automated ones could not: no test of ours examines an axis assignment or the plausibility "
               "of a placeholder. V01 stays WARN with these findings attached, not PASS (no other item has a recorded answer) and not a check "
               "that found nothing")
V01_LIMITATION = ("V01 is WARN with two findings from a manual look at the mast (V01-F1 the profile is probably rotated 90 degrees, V01-F2 "
                  "the mast mass is implausible); the other checklist items have no recorded answer. This report launches nothing for V01 "
                  "and records no inspection of its own")


def v01_details(items, **extra):
    """The details of V01 in every report: what was answered, what was found, what is still open. `items` is the checklist's length."""
    return {"performed": "a person looked at the simulation and answered item 1 (the mast); items 2 to %d have no recorded answer" % items,
            "answered_items": dict(V01_ANSWERED), "unanswered_items": list(range(2, items + 1)),
            "findings": [dict(f) for f in V01_FINDINGS], "reading": V01_READING, "checklist": "docs/v01_visual_check.md", **extra}
# Checks whose sidecar is where their details (and any WARN) live: no sidecar means the test did not complete.
SIDECAR_REQUIRED = {"T02", "T04", "T07", "T08", "T09", "T10", "T13", "E01", "C01", "C02", "W01", "W02", "G01", "G02"}
# Test modules with no check ID in their name.
MODULE_IDS = {"test_datums_revision": "REV"}
ID_IN_NAME = re.compile(r"^test_([a-z]\d{2})_")
EXPECTED_STEPS = {"dev": ("colcon_test", "check_urdf_matrix", "datums_report"), "gazebo": ("colcon_test",)}
REFUSED_DIRTY = 3          # exit status: every check passed, but the tree is dirty so no evidence file was written
NOT_RUN = "NOT_RUN"        # the check's backing test did not run to a result (P12A): never PASS, fails the exit status
FAILING = ("FAIL", NOT_RUN)
RESULT_ORDER = ("PASS", "WARN", "SKIP", "FAIL", NOT_RUN)
# The packages whose tests each profile runs (verify_a6.sh runs the first three, verify_gazebo.sh the gazebo package). The A7 builder
# (build_a7_evidence.py) passes its own list. Every one of them is read: a package that is run but not read is a place for failures to hide.
PROFILE_PACKAGES = {"dev": (ev.PKG, "openamrobot_upperbody_control", "openamrobot_upperbody_bringup"), "gazebo": ("openamrobot_upperbody_gazebo",)}


# --- mapping and combining (pure, tested in test_build_evidence.py) --------------------------------------------------

def check_id_of(classname, name):
    """Check ID for a pytest testcase: the `test_<id>_` prefix of the function, else of its module, else AUX."""
    for text in (name.split("[")[0], classname.rsplit(".", 1)[-1]):
        match = ID_IN_NAME.match(text)
        if match:
            return match.group(1).upper()
    return MODULE_IDS.get(classname.rsplit(".", 1)[-1], "AUX")


def read_xunit(results_dir, since):
    """({id: {tests, failures, skipped}}, [stale file names]) from colcon's xunit files."""
    counts = defaultdict(lambda: {"tests": 0, "failures": 0, "skipped": 0, "failed_tests": []})
    stale = []
    for filename in sorted(os.listdir(results_dir)) if os.path.isdir(results_dir) else []:
        if not filename.endswith(".xunit.xml"):
            continue
        path = os.path.join(results_dir, filename)
        if os.path.getmtime(path) < since:
            stale.append(filename)
            continue
        for case in ET.parse(path).getroot().iter("testcase"):
            cid = check_id_of(case.get("classname", ""), case.get("name", ""))
            row = counts[cid]
            row["tests"] += 1
            if case.find("failure") is not None or case.find("error") is not None:
                row["failures"] += 1
                row["failed_tests"].append(case.get("name"))
            elif case.find("skipped") is not None:
                row["skipped"] += 1
    return dict(counts), stale


def read_xunit_dirs(results_dirs, since):
    """read_xunit over several result directories; counts for the same ID add up."""
    merged, stale = {}, []
    for results_dir in results_dirs:
        counts, old = read_xunit(results_dir, since)
        stale += old
        for cid, row in counts.items():
            if cid in merged:
                for key in ("tests", "failures", "skipped"):
                    merged[cid][key] += row[key]
                merged[cid]["failed_tests"] += row["failed_tests"]
            else:
                merged[cid] = row
    return merged, stale


def combine(check_id, counts, sidecar, missing=None):
    """-> (result, details) for one check from its pytest counts (or None), its sidecar dict (or None) and the registered tests attributed to
    it that left no result (`missing`, names). A PASS needs a test that ran behind it; FAIL beats NOT_RUN beats everything else."""
    details = dict(sidecar["details"]) if sidecar else {}
    if counts:
        details = {"tests": counts["tests"], "failed": counts["failures"], "skipped": counts["skipped"], **details}
        if counts["failed_tests"]:
            details["failed_tests"] = counts["failed_tests"][:40]
    if missing:
        details["tests_without_a_result"] = list(missing)[:10]
    if counts and counts["failures"]:
        return "FAIL", details
    if sidecar and sidecar["result"] == "FAIL":
        return "FAIL", details
    if missing:
        return NOT_RUN, {**details, "reason": f"registered test(s) left no result: {', '.join(list(missing)[:3])}"}
    if not counts:
        if check_id in NOT_IMPLEMENTED:
            return "SKIP", {"reason": NOT_IMPLEMENTED[check_id]}
        why = "a sidecar exists but no test result backs it: the test that wrote it did not complete" if sidecar \
            else "no result: no test ran for this check ID"
        return NOT_RUN, {**details, "reason": why}
    if check_id in SIDECAR_REQUIRED and not sidecar:
        return NOT_RUN, {**details, "reason": "no sidecar: the test that records this check did not complete"}
    if counts["tests"] == counts["skipped"]:
        if sidecar and sidecar["result"] in ("WARN", "SKIP"):
            return sidecar["result"], details
        return "SKIP", details
    if sidecar and sidecar["result"] in ("WARN", "SKIP"):
        return sidecar["result"], details
    return "PASS", details


# --- accounting: what CMake registered against what left a result ------------------------------------------------------

def test_name_id(test_name):
    """Check ID of a ctest test by its name: `t04_frame_contract` and `test_test_l01_mock_controllers.py` -> T04, L01; else AUX."""
    name = test_name
    while name.startswith("test_"):
        name = name[5:]
    match = re.match(r"^([a-z]\d{2})_", name)
    if match:
        return match.group(1).upper()
    return MODULE_IDS.get("test_" + name) or MODULE_IDS.get(test_name) or "AUX"


def registered_tests(workspace, package):
    """Names of the tests CMake registered for the package (`ctest -N` in its build directory), or None if that cannot be read."""
    import subprocess
    build = os.path.join(workspace, "build", package)
    try:
        done = subprocess.run(["ctest", "--test-dir", build, "-N"], capture_output=True, text=True, timeout=120)
    except (OSError, subprocess.SubprocessError):
        return None
    if done.returncode != 0:
        return None
    return re.findall(r"^\s*Test\s+#\d+:\s+(\S+)\s*$", done.stdout, flags=re.M)


def result_status(results_dir, test, since):
    """`ok`, or why the test left no usable result: no_result_file, stale, empty, or did_not_complete (ament's own `missing_result`
    stand-in, written when a test times out or dies of a signal before it can write its file)."""
    path = os.path.join(results_dir, test + ".xunit.xml")
    if not os.path.isfile(path):
        return "no_result_file"
    if os.path.getmtime(path) < since:
        return "stale"
    try:
        cases = list(ET.parse(path).getroot().iter("testcase"))
    except ET.ParseError:
        return "unreadable"
    if not cases:
        return "empty"
    if any(c.get("name", "").endswith(".xunit.missing_result") for c in cases):
        return "did_not_complete"
    return "ok"


def accounting(workspace, packages, since):
    """{registered, with_a_result, problems[{package, test, status, id}]} over the packages' registered tests. A package with no registered
    test is itself a problem: a suite that vanished must not look like a suite that passed."""
    registered, problems = 0, []
    for package in packages:
        names = registered_tests(workspace, package)
        if not names:
            problems.append({"package": package, "test": None, "id": "AUX",
                             "status": "no registered tests (no ctest project in the build directory, or none registered)"})
            continue
        results_dir = os.path.join(workspace, "build", package, "test_results", package)
        for test in names:
            registered += 1
            status = result_status(results_dir, test, since)
            if status != "ok":
                problems.append({"package": package, "test": test, "status": status, "id": test_name_id(test)})
    return {"registered": registered, "with_a_result": registered - len([p for p in problems if p["test"]]), "problems": problems}


def fold_unclaimed(counts, claimed_ids):
    """Results whose ID is not one of the report's checks are not dropped: they are added to AUX (and returned by ID, for the report to say
    so). `counts` is changed in place."""
    unclaimed = {cid: counts.pop(cid) for cid in [c for c in counts if c not in claimed_ids]}
    if unclaimed:
        aux = counts.setdefault("AUX", {"tests": 0, "failures": 0, "skipped": 0, "failed_tests": []})
        for row in unclaimed.values():
            for key in ("tests", "failures", "skipped"):
                aux[key] += row[key]
            aux["failed_tests"] += row["failed_tests"]
    return unclaimed


def missing_by_id(problems, claimed_ids):
    """{check ID: [test names]} of the registered tests with no result; an ID the report does not list is attributed to AUX."""
    out = {}
    for p in problems:
        cid = p["id"] if p["id"] in claimed_ids else "AUX"
        out.setdefault(cid, []).append(p["test"] or p["package"])
    return out


# Names that carry no content: a scaffold placeholder and compiled caches. Anything else only in the install (a stale
# .xacro or .yaml that `$(find ...)` could still pick up) is a difference.
CONTENT_FREE = [".gitkeep", "__pycache__"]


def trees_differ(source, installed, names=("urdf", "config", "launch", "meshes")):
    """Relative paths that differ between the package source and its installed copy (empty = same)."""
    problems = []

    def walk(cmp, prefix):
        problems.extend(f"{prefix}{n}" for n in cmp.left_only + cmp.right_only + cmp.diff_files + cmp.funny_files)
        for name, sub in cmp.subdirs.items():
            walk(sub, f"{prefix}{name}/")

    for name in names:
        a, b = os.path.join(source, name), os.path.join(installed, name)
        if not os.path.isdir(a):
            continue
        if not os.path.isdir(b):
            problems.append(f"{name}/ (not installed)")
            continue
        walk(filecmp.dircmp(a, b, ignore=filecmp.DEFAULT_IGNORES + CONTENT_FREE), f"{name}/")
    return sorted(problems)


# --- reading the sidecars --------------------------------------------------------------------------------------------

def fresh_sidecars(directory, since):
    out = {}
    for doc in ev.read_sidecars(directory).values():
        if os.path.getmtime(os.path.join(directory, doc["file"])) >= since:
            out[doc["id"]] = doc
    return out


# --- the report ------------------------------------------------------------------------------------------------------

def limitations(datums, sidecars):
    placeholders = sorted(n for n, d in datums["datums"].items() if d["status"] == "placeholder")
    out = [
        f"placeholder datums: {len(placeholders)} of {len(datums['datums'])} ({', '.join(placeholders)})",
        "base inertials are platform-sw CAD-export placeholders, the mast mass is a placeholder and the arm masses are "
        "upstream nominal values; none of them is a measured mass",
        "T05 asserts the identity height(openarm_left_base_link) = shoulder_height(mast_id), which holds for any datum "
        "values; it does not validate the datums themselves",
        "E01 is WARN, never PASS: height, width and depth are lower bounds from zero pose, link frame origins and "
        "box/cylinder/sphere extents; mesh extents are not included, so a figure under the limit proves nothing about the "
        "real envelope. A4 decides",
        "T10 reports what the description models. The battery, hub motors, electronics and the mast structure are not "
        "modelled (all F2S), so the total is not the gross mass of the robot",
        "T14, T15 and T16 are expansion checks: nothing here loads the gazebo or the real plugin (the gazebo profile report, "
        "G01 and G02, is where the gazebo one runs), and the real profile cannot even be built here (openarm_hardware needs "
        "OpenArmCAN)",
        "the gripper controllers are the deprecated position_controllers/GripperActionController on purpose: the "
        "replacement serves ParallelGripperCommand instead of GripperCommand, which would decide manipulation#11 / "
        "ADR-009 point 4 (open)",
        "L01 launches mock.launch.py only: controller_manager on mock_components/GenericSystem, which reflects its command "
        "back as state, so it shows the controller wiring and not tracking performance",
        "T02 check_urdf parses the XML and builds the link tree; it does not resolve package:// mesh URIs",
        "W01: KDL consumers (robot_state_publisher, MoveIt in A7) do not see base_link's inertia",
        "W02: with enable_planar_base:=true KDL consumers see world -> base_link as static; planar base motion can "
        "never come from robot_state_publisher (A7: it comes from Gazebo's odom -> base_link, ADR-003 / ADR-005)",
        V01_LIMITATION,
        "config ids are hashes of xacro's output text: a different xacro version that formats a number differently "
        "gives a different id for the same sources",
        "`static+mock` names the evidence class: expansions of every hardware_profile, plus a headless launch of the mock "
        "bringup (L01) and the robot_state_publisher observation for W02. Nothing in this report ran on Gazebo or real hardware",
        "no physical evidence; static expansion and zero-pose FK only",
    ]
    return out


def gazebo_limitations(datums, sidecars, simulator):
    """What the `gazebo` report does not say. The last line is the A6 report's own sentence about physical evidence, kept in
    its words on purpose: a simulation result permits no physical claim of any kind."""
    def engine_of(check_id):
        return (sidecars.get(check_id) or {}).get("details", {}).get("physics_engine_used", "unknown")
    return [
        f"`simulation` names the evidence class: a headless Gazebo Harmonic server ({simulator['gz_sim']}) with "
        f"gz_ros2_control {simulator['gz_ros2_control']}, run by launch tests: G01 on the physics engine {engine_of('G01')} "
        f"(the launch default), G02 on {engine_of('G02')} (passed explicitly). It is a statement about how the model behaves "
        "in that simulator and about nothing else",
        "the arms' <position> interface in gz_ros2_control is a velocity servo (position_proportional_gain 0.1 per step), so "
        "tracking to 0.02 rad says nothing about the real arm's controller, gains, compliance or latency",
        "arm, mast and base masses are placeholders, upstream nominals or platform-sw's CAD export (T10); the base reaction "
        "to an arm motion, tracking and any torque or payload figure in a simulation are not claims about the robot",
        "the launch defaults to DART, the engine platform-sw tuned the base on; under DART neither gripper moves and "
        "finger_joint2 never follows finger_joint1, so G02 passes Bullet-Featherstone explicitly (it fails on DART, which is "
        "the documented limitation and not a defect). The base's contact behaviour was compared on neither engine; G01 "
        "measures kinematics (distance and yaw against the command)",
        "the base's wheel radius, separation and acceleration limits are the DiffDrive plugin's parameters, platform-sw's, "
        "and were not measured here; /odom here is the simulator's own odometry, not an estimate from sensing",
        "G01's rates and distances are properties of this simulator on this machine (headless, software rendering, real-time "
        "factor below 1); they are not requirements on the robot",
        "the gripper controller is still the deprecated position_controllers/GripperActionController on purpose "
        "(manipulation#11 / ADR-009 point 4, open): nothing here chooses between the gripper interfaces",
        V01_LIMITATION,
        "no physical evidence; simulation only. This report is not a statement about hardware",
    ]


def simulator_versions():
    """Versions of what the simulation ran on, read from the installed packages and the simulator itself."""
    import subprocess
    from ament_index_python.packages import get_package_share_directory
    import xml.etree.ElementTree as ET
    try:
        gz = subprocess.run(["gz", "sim", "--versions"], capture_output=True, text=True, timeout=30).stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        gz = "unknown"
    versions = {"gz_sim": gz}
    for package in ("gz_ros2_control", "ros_gz_sim"):
        try:
            root = ET.parse(os.path.join(get_package_share_directory(package), "package.xml")).getroot()
            versions[package] = root.findtext("version")
        except (OSError, LookupError, ET.ParseError):
            versions[package] = "unknown"
    return versions


def build(args):
    share = ev.share_dir()
    workspace = ev.workspace_root(share)
    edir = ev.evidence_dir(share)
    repo = args.repo or os.path.join(workspace, "src", REPO)
    pkg_src = os.path.join(repo, "ros2", "src", ev.PKG)
    results_dir = args.results_dir or os.path.join(workspace, "build", ev.PKG, "test_results", ev.PKG)
    steps = dict(s.split("=", 1) for s in args.step)

    profile = args.profile
    bringup_results_dir = args.bringup_results_dir or os.path.join(workspace, "build", BRINGUP_PKG, "test_results", BRINGUP_PKG)
    gazebo_results_dir = args.gazebo_results_dir or os.path.join(workspace, "build", GAZEBO_PKG, "test_results", GAZEBO_PKG)
    packages = PROFILE_PACKAGES[profile]
    result_dirs = [os.path.join(workspace, "build", p, "test_results", p) for p in packages]
    fixture = bool(args.results_dir or (profile == "gazebo" and args.gazebo_results_dir))        # the tests of this script point at fixtures
    if args.results_dir:
        result_dirs = [args.results_dir] + ([bringup_results_dir] if profile == "dev" else [])
    elif fixture:
        result_dirs = [args.gazebo_results_dir]
    counts, stale_xunit = read_xunit_dirs(result_dirs, args.since)
    accounted = accounting(workspace, packages, args.since) if not fixture else {"registered": 0, "with_a_result": 0, "problems": []}
    sidecars = fresh_sidecars(edir, args.since)
    check_list = (A6_CHECKS + EXTRA_CHECKS) if profile == "dev" else GAZEBO_CHECKS
    claimed = {i for i, _ in check_list}
    unclaimed = fold_unclaimed(counts, claimed)
    missing = missing_by_id(accounted["problems"], claimed)

    gazebo_src = os.path.join(repo, "ros2", "src", GAZEBO_PKG)
    gazebo_share = ev.package_share(GAZEBO_PKG) if profile == "gazebo" else None
    differ = trees_differ(pkg_src, share)
    gazebo_differ = trees_differ(gazebo_src, gazebo_share, names=("launch", "config", "worlds")) if gazebo_share else []
    checks = []
    for check_id, name in check_list:
        if check_id == "V01" and profile == "gazebo":
            result, details = "WARN", v01_details(8)
        elif check_id == "V01":
            result, details = "WARN", v01_details(8, last_launch="S4: robot_state_publisher + joint_state_publisher_gui + rviz2 started "
                                                                 "with no xacro or mesh errors (evidence/S4.json)")
        elif check_id == "I01" and profile == "gazebo":
            problems = sorted(gazebo_differ + [f"{ev.PKG}/{d}" for d in differ])
            result, details = ("FAIL", {"differing": problems, "hint": "run colcon build, then verify_gazebo.sh again"}) if problems \
                else ("PASS", {"compared": [f"{GAZEBO_PKG}: launch, config, worlds", f"{ev.PKG}: urdf, config, launch, meshes"]})
        elif check_id == "I01":
            result, details = ("FAIL", {"differing": differ, "hint": "run colcon build, then verify_a6.sh again"}) if differ \
                else ("PASS", {"compared": ["urdf", "config", "launch", "meshes"]})
        elif check_id == "H02":
            result, details = ("NOT_RUN", {"reason": f"{len(accounted['problems'])} registered test(s) left no usable result"
                                                     if accounted["problems"] else "no registered tests were found to account for",
                                           "problems": accounted["problems"][:20], "registered": accounted["registered"],
                                           "with_a_result": accounted["with_a_result"]}) if (accounted["problems"] or not accounted["registered"]) \
                else ("PASS", {"registered": accounted["registered"], "with_a_result": accounted["with_a_result"], "packages": list(packages)})
        elif check_id == "AUX" and profile == "gazebo":
            result, details = ("PASS", {}) if not counts.get("AUX") else combine("AUX", counts["AUX"], None, missing.get("AUX"))
        else:
            result, details = combine(check_id, counts.get(check_id), sidecars.get(check_id), missing.get(check_id))
        if check_id == "T02" and steps.get("check_urdf_matrix") not in (None, "0"):
            result, details["step"] = "FAIL", f"check_urdf_matrix.py exited {steps['check_urdf_matrix']}"
        if check_id == "AUX" and unclaimed:
            details["unclaimed_check_ids"] = {cid: {"tests": r["tests"], "failures": r["failures"]} for cid, r in unclaimed.items()}
        if check_id == "AUX":
            problems = {}
            for step in EXPECTED_STEPS[profile]:
                if steps.get(step) is None:
                    problems[step] = "step was not run"
                elif steps[step] != "0":
                    problems[step] = f"exited {steps[step]}"
            if stale_xunit:
                problems["stale_xunit"] = stale_xunit
            details["steps"] = steps
            if problems:
                result, details["step_problems"] = "FAIL", problems
        checks.append({"id": check_id, "name": (sidecars.get(check_id) or {}).get("name", name), "result": result,
                       "details": details})

    with open(os.path.join(share, "config", "datums.yaml")) as f:
        datums = yaml.safe_load(f)
    with open(os.path.join(share, "config", "mast_ids.yaml")) as f:
        mast_ids = yaml.safe_load(f)
    try:
        commit, commit_source = ev.upstream_commit_and_source(workspace)
        matrix = ev.expansion_matrix(mast_ids) if profile == "dev" else \
            [c for c in ev.expansion_matrix(mast_ids) + ev.profile_extras(mast_ids) if c[2] == "gazebo"]
        config_ids = {ev.expansion_label(*c): pci.config_id_of(share, commit, *c) for c in matrix}
    except ev.EvidenceError as err:        # no pinned commit means no config id: say so in the report, do not guess
        commit, commit_source, config_ids = None, None, {}
        for c in checks:
            if c["id"] in ("C01", "AUX"):
                c["result"], c["details"]["config_ids"] = "FAIL", f"not produced: {err}"

    provenance = defaultdict(int)
    for d in datums["datums"].values():
        provenance[d["status"]] += 1
    dirty, dirty_files = ev.git_dirty(repo)
    upstream = {"openarm_description": commit or "unknown"}
    ros2_head = ev.git_head(ev.upstream_checkout(workspace, "openarm_ros2"))
    if ros2_head:
        upstream["openarm_ros2"] = ros2_head
    manipulation = ev.git_head(os.path.join(workspace, "src", "openamrobot-manipulation"))
    if manipulation:
        upstream["openamrobot-manipulation"] = manipulation[:7]

    simulator = simulator_versions() if profile == "gazebo" else None
    now = datetime.datetime.now(datetime.timezone.utc)
    doc = {
        "schema": "openamr.evidence.v1",
        "repo": REPO,
        "git_hash": ev.git_head(repo),
        "dirty": dirty,
        "timestamp": now.strftime("%Y-%m-%dT%H:%M:%SZ"),
        "profile": profile,
        "evidence_class": PROFILES[profile],
        **({"simulator": simulator} if profile == "gazebo" else {}),
        "upstream": upstream,
        "datums_revision": datums["revision"],
        "checks": checks,
        "config_ids": config_ids,
        "provenance": {k: provenance[k] for k in ("placeholder", "provisional", "verified")},
        "limitations": (limitations(datums, sidecars) if profile == "dev" else gazebo_limitations(datums, sidecars, simulator)) + (
            [f"openarm_description commit {commit[:7]} was read from {commit_source}, not from a git checkout"]
            if commit_source == ev.METADATA_SOURCE else []),
    }
    if dirty:
        doc["dirty_files"] = dirty_files        # for the console summary only: a dirty report is never written
        return doc, None, sidecars
    path = os.path.join(edir, now.strftime("%Y%m%dT%H%M%SZ") + f"_{profile}.json")
    with open(path, "w") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")
    return doc, path, sidecars


def summary(doc, path, sidecars):
    lines = ["", f"{'Gazebo' if doc['profile'] == 'gazebo' else 'A6'} evidence  {doc['repo']} @ {doc['git_hash'][:7]}{' (dirty)' if doc['dirty'] else ''}  "
                 f"datums {doc['datums_revision']}  class {doc['evidence_class']}"]
    for c in doc["checks"]:
        extra = c["details"].get("reason") or ""
        if not extra and c["details"].get("failed_tests"):
            extra = "failing: " + ", ".join(c["details"]["failed_tests"][:3])
        tests = f"  [{c['details']['tests']} tests]" if "tests" in c["details"] else ""
        lines.append(f"  {c['result']:<4}  {c['id']:<4} {c['name'][:78]}{tests}{('  -- ' + extra[:90]) if extra else ''}")
    report = (sidecars.get("T10") or {}).get("details", {}).get("report")
    if report:
        lines += [""] + [f"  {line}" for line in report]
    constants = (sidecars.get("T13") or {}).get("details", {}).get("constants")
    if constants:
        lines += ["", "Visualization constants (DATUMS.md; visual-only, not datums):"]
        lines += [f"  {c['name']} = {c['value']:g}   in {c['file']}, {c['uses']} use(s)" for c in constants]
    lines += ["", "Config ids (full value is the id; a 12-hex prefix is shown for humans and is only a prefix)"]
    for label, cid in doc["config_ids"].items():
        lines.append(f"  {label:<26} {cid}   prefix {cid.split(':')[1][:12]}")
    counts = defaultdict(int)
    for c in doc["checks"]:
        counts[c["result"]] += 1
    if path is None:
        files = doc["dirty_files"]
        lines += ["", f"Evidence: NOT WRITTEN. {doc['repo']} has uncommitted changes ({len(files)}: "
                      f"{', '.join(files[:5])}{', ...' if len(files) > 5 else ''}), so a report would describe no commit.",
                  "          Commit, then run this again. Nothing above is a record; the evidence directory only holds clean-tree runs."]
    else:
        lines += ["", f"Evidence: {path}"]
    bad = counts["FAIL"] + counts[NOT_RUN]
    lines += ["Result: " + ("FAIL" if counts["FAIL"] else ("NOT_RUN" if counts[NOT_RUN] else "no FAIL")) +
             "  (" + ", ".join(f"{counts[r]} {r}" for r in RESULT_ORDER) + ")"]
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--since", type=float, help="epoch seconds: results older than this are stale (required to build)")
    parser.add_argument("--clear", action="store_true",
                        help="delete the previous run's sidecars of the chosen profile (G* for gazebo, the rest for dev) from the evidence "
                             "directory, print it, and stop")
    parser.add_argument("--step", action="append", default=[], metavar="NAME=RC",
                        help="return code of a step verify_a6.sh / verify_gazebo.sh ran: colcon_test, check_urdf_matrix, datums_report")
    parser.add_argument("--profile", choices=sorted(PROFILES), default="dev",
                        help="dev: the A6 report (static+mock). gazebo: the Gazebo report (simulation)")
    parser.add_argument("--gazebo-results-dir", default=None,
                        help=f"colcon's xunit directory for {GAZEBO_PKG} (G01, G02); default <workspace>/build/...")
    parser.add_argument("--repo", default=None, help=f"the {REPO} checkout (default <workspace>/src/{REPO})")
    parser.add_argument("--results-dir", default=None, help="colcon's xunit directory for the package")
    parser.add_argument("--bringup-results-dir", default=None,
                        help=f"colcon's xunit directory for {BRINGUP_PKG} (L01); default <workspace>/build/...")
    args = parser.parse_args(argv)
    if args.clear:
        directory = ev.evidence_dir()
        ev.clear_sidecars(directory, only=(lambda f: f.startswith("G")) if args.profile == "gazebo"
                          else (lambda f: not f.startswith("G")))
        print(directory)
        return 0
    if args.since is None:
        parser.error("--since is required")
    try:
        doc, path, sidecars = build(args)
    except ev.EvidenceError as err:
        print(f"build_evidence: {err}", file=sys.stderr)
        return 2
    print(summary(doc, path, sidecars))
    if any(c["result"] in FAILING for c in doc["checks"]):
        return 1
    return REFUSED_DIRTY if path is None else 0


if __name__ == "__main__":
    sys.exit(main())
