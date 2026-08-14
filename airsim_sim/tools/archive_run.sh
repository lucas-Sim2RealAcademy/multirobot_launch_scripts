#!/usr/bin/env bash
# Archive the just-finished radio4 run under its own label so the next run
# cannot overwrite it, then re-score it under that label.
#
# usage: archive_run.sh <label>
set -e
LABEL=$1
BASE=/home/lucas/hercules-sim
LOG=$BASE/e1_frames
DST=$LOG/runs/$LABEL
[ -n "$LABEL" ] || { echo "usage: archive_run.sh <label>"; exit 1; }
[ -e "$DST" ] && { echo "REFUSING: $DST already exists"; exit 1; }
mkdir -p "$DST"

# per-drone frame/meta dirs: radio4_<veh> -> <label>_<veh>
for d in "$LOG"/radio4_*/; do
  [ -d "$d" ] || continue
  veh=$(basename "$d" | sed 's/^radio4_//')
  cp -r "$d" "$DST/${LABEL}_${veh}"
done
# flat logs: *_radio4*.ext -> *_<label>*.ext
for f in "$LOG"/*radio4*; do
  [ -f "$f" ] || continue
  b=$(basename "$f")
  cp "$f" "$DST/${b//radio4/$LABEL}"
done
# host load snapshot for the record
if [ -f "$DST/metrics_${LABEL}.csv" ]; then
  python3 - "$DST/metrics_${LABEL}.csv" <<'PY' > "$DST/load_${LABEL}.txt"
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
l = [float(r['load1']) for r in rows if r.get('load1')]
if l:
    l2 = sorted(l)
    print('load1 n=%d min=%.1f median=%.1f max=%.1f mean=%.1f'
          % (len(l), l2[0], l2[len(l2)//2], l2[-1], sum(l)/len(l)))
PY
  cat "$DST/load_${LABEL}.txt"
fi

# re-score under the archive label
"$BASE/fidelity_scorecard.sh" "$LABEL" "$DST" > "$DST/scorecard_${LABEL}.txt" 2>&1 || true
grep -E "fleet_arena_cells|fleet_effort_m2|fleet_unique_m2|overlap_floor_pct|fleet_overlap_pct|fleet_overlap_concurrent|odom_divergence" \
  "$DST/scorecard_${LABEL}.txt" || true
echo "archived -> $DST"
