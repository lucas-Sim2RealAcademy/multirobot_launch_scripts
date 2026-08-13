#!/usr/bin/env bash
# Archive one run's artifacts (metas + all logs + scorecard) under a distinct label so
# the next `run_fleet_radio.sh 4 150` (LABEL is hardcoded to radio$N) cannot overwrite it.
#
# usage: archive_run.sh <newlabel> [srclabel]
#   archive_run.sh baseline radio4     -> e1_frames/runs/baseline/{...radio4 artifacts}
set -e
NEW=$1; SRC=${2:-radio4}
LOG=/home/lucas/hercules-sim/e1_frames
DST=$LOG/runs/$NEW
[ -n "$NEW" ] || { echo "usage: archive_run.sh <newlabel> [srclabel]"; exit 1; }
mkdir -p "$DST"
shopt -s nullglob
for f in "$LOG"/*_${SRC}_*.log "$LOG"/*_${SRC}.log "$LOG"/*_${SRC}.csv \
         "$LOG"/scorecard_${SRC}.txt "$LOG"/ue_${SRC}.log "$LOG"/radio_${SRC}.log; do
  cp -a "$f" "$DST"/ 2>/dev/null || true
done
for d in "$LOG"/${SRC}_*/; do
  b=$(basename "$d")
  mkdir -p "$DST/$b"
  cp -a "$d"meta_*.json "$DST/$b"/ 2>/dev/null || true
done
# leave a copy of the labelled artifacts in place under the new label too, so
# vio_metrics.py <newlabel> works directly against e1_frames.
for f in "$DST"/*_${SRC}_*.log "$DST"/*_${SRC}.log "$DST"/*_${SRC}.csv \
         "$DST"/scorecard_${SRC}.txt; do
  [ -e "$f" ] || continue
  mv "$f" "$(dirname "$f")/$(basename "$f" | sed "s/${SRC}/${NEW}/")"
done
for d in "$DST"/${SRC}_*/; do
  [ -d "$d" ] || continue
  mv "$d" "$(dirname "$d")/$(basename "$d" | sed "s/^${SRC}_/${NEW}_/")"
done
echo "archived $SRC -> $DST"
ls "$DST" | head -40
