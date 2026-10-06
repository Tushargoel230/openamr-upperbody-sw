#!/usr/bin/env bash
# Regenerate the self-collision matrix in config/openamrobot.srdf with the MoveIt Setup Assistant's own collision-matrix code.
#
#   source /opt/ros/jazzy/setup.bash && source <workspace>/install/setup.bash
#   ros2 run openamrobot_upperbody_moveit regenerate_acm.sh [--trials N] [--mast-id mast_1350] [--srdf PATH]
#
# collisions_updater is the command-line tool of moveit_setup_assistant: the same ComputeDefaultCollisions code the GUI's Self-Collisions
# page runs, so the sampling, the "never in collision" test and the adjacency rule are the Setup Assistant's. It was used instead of
# the GUI because the GUI cannot be driven from a non-interactive session; opening `moveit_setup_assistant` on the same model and
# pressing "Generate Collision Matrix" at 10000 samples gives the same kind of matrix (the sampling is random, so the exact set of
# "never" pairs can differ by a rare pair from run to run: that is what scripts/validate_acm.py is for).
#
# Settings (all recorded in config/acm/acm_generation.yaml):
#   --urdf        the installed combined_robot.xacro      --xacro-args  mast_id:=<mast> arm_package:=openarm hardware_profile:=mock
#   --srdf        the SRDF as it is (groups, virtual joint); its own disable_collisions are NOT kept (no --keep), the tool recomputes all
#   --trials      10000 or more                           --default     on: disable pairs that collide in the default state
#   --always      OFF: a pair that collides in every sample is a modelling question to look at, not something to disable
#   adjacent and never-in-collision pairs are always on in the tool.
# The merge keeps the pairs imported from upstream where they are and writes only the rest between the BEGIN/END markers.
set -eu

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRIALS=10000
MAST=mast_1350
SRDF=""
while [ $# -gt 0 ]; do
  case "$1" in
    --trials) TRIALS="$2"; shift 2 ;;
    --mast-id) MAST="$2"; shift 2 ;;
    --srdf) SRDF="$2"; shift 2 ;;
    *) echo "regenerate_acm: unknown argument $1" >&2; exit 2 ;;
  esac
done
[ "$TRIALS" -ge 10000 ] || { echo "regenerate_acm: --trials must be at least 10000 (A7 asks for that many samples)" >&2; exit 2; }

for tool in ros2 python3; do
  command -v "$tool" >/dev/null 2>&1 || { echo "regenerate_acm: '$tool' not on PATH: source /opt/ros/jazzy/setup.bash and the workspace install/setup.bash" >&2; exit 2; }
done
SHARE="$(python3 -c "from ament_index_python.packages import get_package_share_directory as g; print(g('openamrobot_upperbody_moveit'))")"
DESC="$(python3 -c "from ament_index_python.packages import get_package_share_directory as g; print(g('openamrobot_upperbody_description'))")"
# Edit the SOURCE tree's files, not the installed copies: the SRDF and the metadata are committed files.
SRC="${OPENAMR_MOVEIT_SRC:-$(cd "$HERE/.." && pwd)}"
[ -f "$SRC/config/openamrobot.srdf" ] || { echo "regenerate_acm: no $SRC/config/openamrobot.srdf: run this from the source tree or set OPENAMR_MOVEIT_SRC" >&2; exit 2; }
SRDF="${SRDF:-$SRC/config/openamrobot.srdf}"
OUT="$(mktemp -d)/tool_output.srdf"
XACRO_ARGS="mast_id:=$MAST arm_package:=openarm hardware_profile:=mock"
VERSION="$(python3 - <<'PY'
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory as g
print(ET.parse(g("moveit_setup_assistant") + "/package.xml").getroot().findtext("version"))
PY
)"
COMMAND="ros2 run moveit_setup_assistant collisions_updater --urdf $DESC/urdf/combined_robot.xacro --xacro-args \"$XACRO_ARGS\" --srdf $SRDF --output $OUT --default --trials $TRIALS"
LOG="$(dirname "$OUT")/tool.log"

echo "== $COMMAND --verbose"
eval "$COMMAND --verbose" > "$LOG" 2>&1 || true        # --verbose only adds the Statistics block that is recorded; it is not a setting
grep -E "Total Links|Total possible|Always in|Never in|Default in|Adjacent links|Sometimes in|TOTAL DISABLED|ERROR|FATAL|successfully loaded" "$LOG" | sed 's/^.*\]: //'

[ -s "$OUT" ] || { echo "regenerate_acm: the tool wrote no output (it could not load the model?)" >&2; exit 1; }

python3 "$HERE/merge_acm.py" --generated "$OUT" --srdf "$SRDF" --meta "$SRC/config/acm/acm_generation.yaml" \
  --trials "$TRIALS" --mast-id "$MAST" --xacro-args "$XACRO_ARGS" --tool-version "$VERSION" --date "$(date -u +%F)" \
  --tool-command "$COMMAND" --tool-log "$LOG"
