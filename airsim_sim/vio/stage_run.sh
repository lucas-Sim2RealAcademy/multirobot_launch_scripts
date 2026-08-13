#!/usr/bin/env bash
# One VIO-stabilization stage run: clear -> run_fleet_radio.sh -> archive under a label.
#
#   stage_run.sh R1 [secs]
#
# run_fleet_radio.sh hardcodes LABEL=radio<N>, so every run overwrites e1_frames/radio4_*
# and *_radio4_*.log.  This clears them first (so a short run cannot inherit stale metas
# from a long one) and archives the result under e1_frames/runs/<label>/ afterwards.
set -e
LABEL=$1; SECS=${2:-150}
[ -n "$LABEL" ] || { echo "usage: stage_run.sh <label> [secs]"; exit 1; }
BASE=/home/lucas/hercules-sim
LOG=$BASE/e1_frames
VEHS="ghost delta buckshee thunderstrike"

for v in $VEHS; do rm -f "$LOG/radio4_$v"/meta_*.json "$LOG/radio4_$v"/chase_*.png \
                          "$LOG/radio4_$v"/front_*.png 2>/dev/null || true; done
rm -f "$LOG"/*_radio4_*.log "$LOG"/ue_radio4.log "$LOG"/radio_radio4.log \
      "$LOG"/scorecard_radio4.txt 2>/dev/null || true

cd "$BASE"
"$BASE/run_fleet_radio.sh" 4 "$SECS"

n=0; for v in $VEHS; do n=$((n + $(ls "$LOG/radio4_$v"/meta_*.json 2>/dev/null | wc -l))); done
echo "stage_run: $n meta records captured"
[ "$n" -gt 100 ] || { echo "stage_run: RUN FAILED (too few metas) - not archiving"; exit 2; }
"$BASE/investigation/vio/archive_run.sh" "$LABEL" radio4
python3 "$BASE/investigation/vio/vio_metrics.py" "$LABEL" "$LOG/runs/$LABEL"
