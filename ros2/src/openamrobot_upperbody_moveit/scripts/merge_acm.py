#!/usr/bin/env python3
"""Merge the collision matrix the MoveIt Setup Assistant's collisions_updater wrote into config/openamrobot.srdf.

    merge_acm.py --generated TOOL_OUTPUT.srdf --srdf config/openamrobot.srdf --meta config/acm/acm_generation.yaml
                 --trials 10000 --mast-id mast_1350 --xacro-args "..." [--date YYYY-MM-DD]

Run by regenerate_acm.sh, which is the documented way to PROPOSE a new matrix. The checked-in SRDF is the contract (its fingerprint is
recorded in config/acm/acm_generation.yaml and test_acm_generation.py checks it), and the tool that generates the matrix is not seeded, so a
run gives a different matrix from the contract's: without `--write` this writes the merged SRDF to `--proposal` and prints what differs, and
changes nothing else. With `--write` it replaces the contract, which is a decision, not a verification step. It rewrites ONLY the text between the
`<!-- BEGIN GENERATED ACM -->` and `<!-- END GENERATED ACM -->` markers. A pair already listed in the SRDF outside the markers (the
imported upstream arm-internal pairs) is not repeated inside them, so every pair is disabled exactly once. The settings go into
config/acm/acm_generation.yaml, which test_acm_generation.py reads: it fails if the sample count is below 10000, or if the
generation did not enable the "never in collision", "adjacent" and "default" categories and left "always" off.
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET

import yaml

from openamrobot_upperbody_moveit.acm_split import acm_fingerprint, acm_pairs, diff_acm

BEGIN = "<!-- BEGIN GENERATED ACM -->"
END = "<!-- END GENERATED ACM -->"
REASONS = ("Never", "Adjacent", "Default")          # what the tool writes with --default and without --always


def pair_key(link1, link2):
    return tuple(sorted((link1, link2)))


def read_pairs(root):
    return {pair_key(d.get("link1"), d.get("link2")): d.get("reason") for d in root.findall("disable_collisions")}


def split_srdf(text):
    """(before, after, outside_text): the text up to and including BEGIN, the text from END on, and the two combined."""
    match = re.search(re.escape(BEGIN) + r".*?" + re.escape(END), text, flags=re.S)
    if not match:
        raise SystemExit(f"merge_acm: {BEGIN} ... {END} not found in the SRDF")
    return text[:match.start()], text[match.end():]


def merge(srdf_text, generated_root, settings):
    before, after = split_srdf(srdf_text)
    outside = read_pairs(ET.fromstring(before + after))
    generated = read_pairs(generated_root)
    unknown = sorted({r for r in generated.values()} - set(REASONS))
    if unknown:
        raise SystemExit(f"merge_acm: the tool wrote reasons {unknown}; expected only {REASONS} (was --always passed?)")
    new = {pair: reason for pair, reason in generated.items() if pair not in outside}
    missing = sorted(pair for pair in outside if pair not in generated)
    counts = {reason: sum(1 for r in new.values() if r == reason) for reason in REASONS}
    lines = [BEGIN,
             f"    <!-- Written by merge_acm.py from the output of {settings['tool']} (MoveIt {settings['moveit_version']}),",
             f"         generated on {settings['date']} at {settings['mast_id']}: {settings['trials']} samples, categories never in collision,",
             "         adjacent and default enabled, always-in-collision off. Settings: config/acm/acm_generation.yaml; the way to redo",
             "         it: scripts/regenerate_acm.sh. Pairs already disabled in section 1 are not repeated here. -->"]
    for (link1, link2), reason in sorted(new.items()):
        lines.append(f'    <disable_collisions link1="{link1}" link2="{link2}" reason="{reason}"/>')
    lines.append("    " + END)
    block = "\n".join(lines)
    return before + block + after, {"written": counts, "written_total": len(new), "imported_before_the_markers": len(outside),
                                    "imported_not_in_the_tool_output": [list(p) for p in missing]}


STATISTICS = {"Total Links": "total_links", "Total possible collisions": "possible_pairs", "Always in collision": "always_in_collision",
              "Never in collision": "never_in_collision", "Default in collision": "default_in_collision",
              "Adjacent links disabled": "adjacent_disabled", "Sometimes in collision": "sometimes_in_collision",
              "TOTAL DISABLED": "total_disabled"}


def tool_statistics(path):
    """The tool's own Statistics block (logged with --verbose), as {name: int}; {} when there is no log."""
    stats = {}
    if not path or not os.path.isfile(path):
        return stats
    with open(path) as f:
        for line in f:
            match = re.search(r"\]: (" + "|".join(map(re.escape, STATISTICS)) + r") : ([0-9.]+)\s*$", line)
            if match:
                stats[STATISTICS[match.group(1)]] = int(float(match.group(2)))
    return stats


def contract_record(srdf_root):
    """The `contract` block of acm_generation.yaml: what the checked-in matrix IS, as a fingerprint and counts, and what regenerating it means."""
    pairs = acm_pairs(srdf_root)
    return {"file": "config/openamrobot.srdf",
            "what": "every <disable_collisions> of the SRDF (section 1, imported from upstream, and the generated block). This file is the "
                    "contract: M03 validates it, and a change to it is a change to the contract",
            "fingerprint": acm_fingerprint(pairs),
            "pairs": {"total": len(pairs), **{reason: sum(1 for r in pairs.values() if r == reason) for reason in REASONS}},
            "generation_is_seeded": False,
            "regeneration": "approximate. moveit_setup_assistant's collisions_updater has no --seed option and samples with an unseeded generator "
                            "(two runs at the same settings gave 473 and 468 `Never` pairs at 3000 trials; three runs at 10000 gave 455, 456 "
                            "and 458). Re-running scripts/regenerate_acm.sh therefore proposes a new contract and is not a verification step; "
                            "scripts/validate_acm.py (M03) is the verification, and it validates THIS file"}


def print_diff(old, new):
    added, removed, relabelled = diff_acm(old, new)
    print(f"proposal vs the contract: {len(added)} pair(s) only in the proposal, {len(removed)} only in the contract, {len(relabelled)} relabelled "
          f"({len(new)} vs {len(old)} pairs)")
    for title, items in (("only in the proposal", added), ("only in the contract", removed)):
        for (a, b), reason in sorted(items.items()):
            print(f"  {title}: {a} <-> {b} ({reason})")
    for (a, b), (was, now) in sorted(relabelled.items()):
        print(f"  relabelled: {a} <-> {b}: {was} -> {now}")
    return added, removed, relabelled


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--generated", help="the SRDF collisions_updater wrote")
    parser.add_argument("--srdf", required=True, help="config/openamrobot.srdf: the contract (rewritten in place only with --write)")
    parser.add_argument("--meta", required=True, help="config/acm/acm_generation.yaml (written only with --write or --record-contract)")
    parser.add_argument("--write", action="store_true", help="replace the contract: rewrite the SRDF and the record. Without it, a proposal only")
    parser.add_argument("--proposal", help="where the merged SRDF is written when --write is not given")
    parser.add_argument("--record-contract", action="store_true", help="only refresh the `contract` block of --meta from the --srdf as it is")
    parser.add_argument("--trials", type=int)
    parser.add_argument("--mast-id")
    parser.add_argument("--xacro-args")
    parser.add_argument("--tool-version", default="unknown")
    parser.add_argument("--date")
    parser.add_argument("--tool-command", help="the exact command line, recorded verbatim")
    parser.add_argument("--tool-log", default=None, help="the tool's --verbose output; its Statistics block is recorded")
    args = parser.parse_args(argv)
    if args.record_contract:
        with open(args.meta) as f:
            meta = yaml.safe_load(f)
        meta["contract"] = contract_record(ET.parse(args.srdf).getroot())
        with open(args.meta, "w") as f:
            yaml.safe_dump(meta, f, sort_keys=False, width=140)
        print(f"recorded the contract of {args.srdf}: {meta['contract']['fingerprint']}, {meta['contract']['pairs']}")
        return 0
    missing = [name for name in ("generated", "trials", "mast_id", "xacro_args", "date", "tool_command") if getattr(args, name) is None]
    if missing:
        parser.error("required unless --record-contract: " + ", ".join("--" + m.replace("_", "-") for m in missing))
    if not args.write and not args.proposal:
        parser.error("give --proposal PATH (where the proposed SRDF goes) or --write (replace the contract)")
    settings = {"tool": "moveit_setup_assistant collisions_updater", "moveit_version": args.tool_version, "date": args.date,
                "mast_id": args.mast_id, "trials": args.trials}
    statistics = tool_statistics(args.tool_log)
    with open(args.srdf) as f:
        text = f.read()
    merged, stats = merge(text, ET.parse(args.generated).getroot(), settings)
    print_diff(acm_pairs(ET.fromstring(text)), acm_pairs(ET.fromstring(merged)))
    if not args.write:
        with open(args.proposal, "w") as f:
            f.write(merged)
        print(f"PROPOSAL written to {args.proposal}; the contract ({args.srdf}) and its record are unchanged. Re-running the tool is not a "
              f"verification step. To adopt this matrix as the new contract, re-run with --write and review the diff above.")
        return 0
    with open(args.srdf, "w") as f:
        f.write(merged)
    meta = {"schema_version": 1, **settings, "xacro_args": args.xacro_args, "command": args.tool_command,
            "categories": {"never_in_collision": True, "adjacent": True, "default": True, "always": False},
            "min_collision_fraction": "the tool's default; unused because the always category is off",
            "tool_statistics": statistics, **stats, "contract": contract_record(ET.fromstring(merged))}
    with open(args.meta, "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False, width=140)
    print(f"merged {stats['written_total']} generated pairs ({stats['written']}) into {args.srdf}; "
          f"{stats['imported_before_the_markers']} imported pairs left where they were. THE CONTRACT CHANGED: {meta['contract']['fingerprint']}")
    if stats["imported_not_in_the_tool_output"]:
        print("WARNING: imported pairs the tool did not list:", stats["imported_not_in_the_tool_output"], file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
