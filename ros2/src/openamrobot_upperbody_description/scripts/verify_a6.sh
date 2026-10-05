#!/usr/bin/env bash
# A6 verification. Runs colcon test for this package, the T02 check_urdf matrix, datums_report.py and the config ids,
# then writes <workspace>/evidence/<UTC timestamp>_dev.json (openamr.evidence.v1, evidence_class static+mock).
# Exit status is non-zero iff any check is FAIL.
#
#   source /opt/ros/jazzy/setup.bash && source install/setup.bash
#   ros2 run openamrobot_upperbody_description verify_a6.sh        # or run this file directly
#
# Run `colcon build` first: the tests and this report describe the INSTALLED package (check I01 fails if it is stale).
# The tests of openamrobot_upperbody_control and openamrobot_upperbody_bringup run too: L01 is the bringup launch test.
# OPENAMR_EVIDENCE_DIR overrides where evidence goes; OPENAMR_WS overrides the workspace root.
set -u

PKG=openamrobot_upperbody_description
BRINGUP=openamrobot_upperbody_bringup
CONTROL=openamrobot_upperbody_control
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

workspace_root() {
  if [ -n "${OPENAMR_WS:-}" ]; then echo "$OPENAMR_WS"; return; fi
  local d="$HERE"
  while [ "$d" != "/" ]; do
    if [ -d "$d/install" ] && [ -d "$d/src" ]; then echo "$d"; return; fi
    d="$(dirname "$d")"
  done
  return 1
}

WS="$(workspace_root)" || { echo "verify_a6: cannot find the workspace (no ancestor with install/ and src/); set OPENAMR_WS" >&2; exit 2; }
for tool in colcon xacro check_urdf python3; do
  command -v "$tool" >/dev/null 2>&1 || { echo "verify_a6: '$tool' not on PATH: source /opt/ros/jazzy/setup.bash and $WS/install/setup.bash first" >&2; exit 2; }
done
python3 -c "import ament_index_python.packages as p; p.get_package_share_directory('$PKG')" 2>/dev/null \
  || { echo "verify_a6: package $PKG is not on the ament path: source $WS/install/setup.bash" >&2; exit 2; }

export PYTHONDONTWRITEBYTECODE=1        # compiled caches would make the checkout look dirty
START="$(date +%s.%N)"

echo "== verify_a6: workspace $WS"
EVIDENCE_DIR="$(python3 "$HERE/build_evidence.py" --clear)" || exit 2
echo "== evidence directory: $EVIDENCE_DIR (previous sidecars cleared)"
rm -f "$WS/build/$PKG/test_results/$PKG/"*.xunit.xml "$WS/build/$BRINGUP/test_results/$BRINGUP/"*.xunit.xml 2>/dev/null

echo; echo "== colcon test ($PKG, $CONTROL, $BRINGUP)"
( cd "$WS" && colcon test --packages-select "$PKG" "$CONTROL" "$BRINGUP" --event-handlers console_cohesion+ )
RC_COLCON=$?
for p in "$PKG" "$CONTROL" "$BRINGUP"; do
  ( cd "$WS" && colcon test-result --test-result-base "build/$p" --verbose 2>&1 | tail -n 25 )
done

echo; echo "== T02: check_urdf on every T01 expansion"
python3 "$HERE/check_urdf_matrix.py"
RC_T02=$?

echo; echo "== datums report"
python3 "$HERE/datums_report.py"
RC_DATUMS=$?

echo; echo "== evidence report"
python3 "$HERE/build_evidence.py" --since "$START" \
  --step "colcon_test=$RC_COLCON" --step "check_urdf_matrix=$RC_T02" --step "datums_report=$RC_DATUMS"
exit $?
