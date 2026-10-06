#!/usr/bin/env python3
"""T02: run `check_urdf` on every T01 expansion, `openarm` on `gazebo` and `real` at the release mast_id included.

    check_urdf_matrix.py

Also runs, labelled `extra`: the planar flag at the release mast_id for each arm package (the W02 reasoning says
check_urdf accepts the planar joint, and that should be a thing somebody ran), and `openarm` on `gazebo` and `real` at
mast_1300 (the first non-default mast_id), so the other profiles are checked at a second mast height. Prints one line per expansion, writes
`T02_check_urdf.json` to the evidence directory, and exits non-zero if any expansion is rejected or check_urdf is
missing (a check that cannot run is not a pass).
"""
import os
import shutil
import subprocess
import sys
import tempfile

import xacro
import yaml

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import evidence_common as ev  # noqa: E402


def run_check_urdf(urdf_xml, executable=None):
    """(returncode, output) of `check_urdf` on this URDF text. returncode None if the tool is not installed."""
    executable = executable or shutil.which("check_urdf")
    if executable is None:
        return None, "check_urdf not found on PATH (ros-<distro>-urdfdom / liburdfdom-tools)"
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:   # scratch input, not evidence
        f.write(urdf_xml)
    try:
        result = subprocess.run([executable, f.name], capture_output=True, text=True)
    finally:
        os.unlink(f.name)
    return result.returncode, (result.stdout + result.stderr).strip()


def run_matrix(share, executable=None):
    """-> list of {id, args, extra, returncode, summary}, one per expansion."""
    with open(os.path.join(share, "config", "mast_ids.yaml")) as f:
        mast_ids = yaml.safe_load(f)
    cases = [(c, "false", False) for c in ev.expansion_matrix(mast_ids)]
    cases += [(c, "true", True) for c in ev.planar_extras(mast_ids)]
    cases += [(c, "false", True) for c in ev.profile_extras(mast_ids)]
    rows = []
    for (mast_id, arm_package, hardware_profile), planar, extra in cases:
        label = ev.expansion_label(mast_id, arm_package, hardware_profile) + (" +planar" if planar == "true" else "")
        try:
            urdf = ev.expand_urdf(share, mast_id, arm_package, hardware_profile, planar)
            rc, output = run_check_urdf(urdf, executable)
        except xacro.XacroException as err:
            rc, output = 99, f"xacro failed: {err}"
        rows.append({"id": label, "args": {"mast_id": mast_id, "arm_package": arm_package,
                                           "hardware_profile": hardware_profile, "enable_planar_base": planar},
                     "extra": extra, "returncode": rc,
                     "summary": output.splitlines()[0] if output else ""})
    return rows


def main():
    share = ev.share_dir()
    rows = run_matrix(share)
    for row in rows:
        ok = row["returncode"] == 0
        print(f"  {'PASS' if ok else 'FAIL'}  check_urdf  {row['id']:<34} rc={row['returncode']}"
              f"{'  (extra: not in T01)' if row['extra'] else ''}")
        if not ok:
            print(f"        {row['summary']}")
    failed = [r["id"] for r in rows if r["returncode"] != 0]
    required = [r for r in rows if not r["extra"]]
    ev.write_sidecar("T02", "check_urdf", "check_urdf accepts every T01 expansion", "FAIL" if failed else "PASS", {
        "expansions": len(required), "extra_planar_expansions": len(rows) - len(required),
        "failed": failed, "rows": [{k: r[k] for k in ("id", "extra", "returncode")} for r in rows],
        "tool": shutil.which("check_urdf"),
    })
    print(f"T02: {len(rows) - len(failed)}/{len(rows)} expansions accepted by check_urdf")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
