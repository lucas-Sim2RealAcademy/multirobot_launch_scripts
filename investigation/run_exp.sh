#!/usr/bin/env bash
# Guarded experiment runner for the overlap work.
#
# Wraps run_fleet_radio.sh with the three safeguards this investigation needs:
#   1. PREFLIGHT  -- refuse to start while a previous stack is still up, so a
#      stale teardown cannot SIGTERM the new run.
#   2. WIPE       -- clear e1_frames/radio4_*/ first.  meta_*.json is the ONLY
#      ground truth the scorecard reads; if a run dies early those files
#      survive from the previous run and the scorecard silently re-scores old
#      data and reports it as a new result (this happened once already).
#   3. VALIDATE   -- after the run, assert the meta files are fresh AND every
#      planner logged PLAN OK.  A run that fails validation is archived under
#      <label>_INVALID and must not be compared against anything.
#
# usage: [ENV=...] run_exp.sh <label> [seconds]
set -u
LABEL=${1:?usage: run_exp.sh <label> [seconds]}
SECS=${2:-150}
BASE=/home/lucas/hercules-sim
LOG=$BASE/e1_frames
[ -e "$LOG/runs/$LABEL" ] && { echo "REFUSING: runs/$LABEL exists"; exit 1; }

echo "=== PREFLIGHT ($LABEL) ==="
# NB: UnrealEditor is deliberately NOT part of the busy test.  run_fleet_radio.sh
# leaves UE running after a run and kills it itself at the start of the next one,
# so waiting for UE to exit here deadlocks forever.  What must be gone is the
# per-drone stack -- and airsim_realsense_node in particular, which ignores
# SIGTERM: every crashed run leaves 4 orphans that hold RPC slots against a dead
# AirSim and silently starve the next run's sensors.
for i in $(seq 1 60); do
  busy=0
  pgrep -f "[a]irsim_realsense_node" >/dev/null && busy=1
  pgrep -f "[s]imple_exploration_planner" >/dev/null && busy=1
  pgrep -f "[c]uvslam_sim_bridge" >/dev/null && busy=1
  [ $busy -eq 0 ] && break
  echo "  waiting for previous stack to finish teardown ($i)"
  [ "$i" -ge 6 ] && { echo "  forcing orphan cleanup"; \
      pkill -9 -f "[a]irsim_realsense_node" 2>/dev/null || true; }
  sleep 5
done
if [ $busy -ne 0 ]; then echo "PREFLIGHT FAIL: stack still up"; exit 1; fi
LOAD_PRE=$(cut -d' ' -f1 /proc/loadavg)
echo "  host clean, load1_pre=$LOAD_PRE"

echo "=== WIPE stale ground truth ==="
rm -rf "$LOG"/radio4_*/
echo "  cleared $LOG/radio4_*/"

# UE segfaults on roughly half of all cold starts on this host (Signal 11 a few
# seconds after "Engine is initialized", always with the AirSim vehicle spawn in
# flight).  It is not caused by anything in this experiment -- it happened on an
# unmodified run too -- so retry rather than report a fabricated result.
ok=0
for attempt in 1 2 3; do
  echo "=== RUN ($LABEL, ${SECS}s) attempt $attempt ==="
  rm -rf "$LOG"/radio4_*/
  # Hard timeout: when UE segfaults, one cuvslam_sim_bridge can hang forever in
  # confirmConnection() against the dead RPC server, and run_fleet_radio.sh's
  # `wait` then never returns -- the run sits there indefinitely with no output.
  # Budget = flight time + 5 min of startup/teardown.
  timeout -k 30 $((SECS + 300)) "$BASE/run_fleet_radio.sh" 4 "$SECS" \
      > "$LOG/run_${LABEL}.out" 2>&1
  rc=$?
  echo "  run script exited $rc"
  [ $rc -eq 124 ] && echo "  (timed out -- hung bridge after a UE crash)"
  pkill -9 -f "[c]uvslam_sim_bridge" 2>/dev/null || true

  echo "=== VALIDATE (attempt $attempt) ==="
  ok=1
  for v in ghost delta buckshee thunderstrike; do
    n=$(ls "$LOG/radio4_$v"/meta_*.json 2>/dev/null | wc -l)
    p=$(grep -ac "PLAN OK" "$LOG/planner_radio4_$v.log" 2>/dev/null)
    p=${p:-0}
    echo "  $v: meta=$n PLAN_OK=$p"
    [ "$n" -lt 100 ] && ok=0
    [ "$p" -lt 1 ] && ok=0
  done
  [ $ok -eq 1 ] && break
  if grep -q "Segmentation fault" "$LOG/run_${LABEL}.out" 2>/dev/null; then
    echo "  UE segfaulted -- retrying"
  else
    echo "  run invalid for a non-UE reason -- retrying once anyway"
  fi
  pkill -9 -f "[a]irsim_realsense_node" 2>/dev/null || true
  sleep 10
done
if [ $ok -ne 1 ]; then
  echo "VALIDATE FAIL -- archiving as ${LABEL}_INVALID, DO NOT COMPARE"
  LABEL="${LABEL}_INVALID"
fi

echo "=== ARCHIVE ==="
"$BASE/investigation/archive_run.sh" "$LABEL"
find "$LOG/runs/$LABEL" -name '*.png' -delete
echo "  load1_pre=$LOAD_PRE" >> "$LOG/runs/$LABEL/load_${LABEL}.txt"
cp "$LOG/run_${LABEL}.out" "$LOG/runs/$LABEL/" 2>/dev/null
echo "load1_pre=$LOAD_PRE"
[ $ok -eq 1 ] || exit 2
exit 0
