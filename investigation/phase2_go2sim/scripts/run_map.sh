#!/bin/bash
# PHASE-2 per-map runner.  Blocks binary + symlink graft + ?game= override, isolated on
# port 41471 so a sibling agent's UE on 41451 cannot contaminate the timing.
#   run_map.sh <label> <ue_map_or_"">
# UE segfaults on ~50% of cold starts, so the launch is retried.
set -u
LABEL=$1
MAP=${2:-}
SP=/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad
PROJ=/home/lucas/hercules-sim/HERCULES/Unreal/Environments/Blocks
SETTINGS=$SP/p2/settings-phase2-probe.json
PORT=41471
OUTDIR=$SP/probe_$LABEL
mkdir -p "$OUTDIR"

# never pkill a pattern that matches this script: match the project path + the port only
kill_mine() {
  for pid in $(pgrep -f "Blocks.uproject" 2>/dev/null); do
    if tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null | grep -q "settings-phase2-probe"; then
      kill "$pid" 2>/dev/null
    fi
  done
  for i in $(seq 1 20); do ss -ltn | grep -q ":$PORT " || break; sleep 1; done
}

kill_mine
MAPARG=()
[ -n "$MAP" ] && MAPARG=("${MAP}?game=/Script/AirSim.AirSimGameMode")

for attempt in 1 2 3; do
  echo "[run_map] $LABEL attempt $attempt map='${MAP:-<default Blocks>}' load1=$(cut -d' ' -f1 /proc/loadavg)"
  ( cd "$PROJ" && /home/lucas/UE5/UE5.2.1/Engine/Binaries/Linux/UnrealEditor \
      "$PROJ/Blocks.uproject" "${MAPARG[@]}" \
      -game -RenderOffscreen -windowed -ResX=1280 -ResY=720 \
      -settings="$SETTINGS" -log -stdout -unattended -nosplash \
      > "$SP/ue_$LABEL.log" 2>&1 ) &
  UEPID=$!
  up=0
  for i in $(seq 1 90); do
    if ss -ltn | grep -q ":$PORT "; then up=1; break; fi
    kill -0 $UEPID 2>/dev/null || break
    sleep 2
  done
  if [ $up = 1 ]; then echo "[run_map] up on $PORT after ${i}x2s"; break; fi
  echo "[run_map] failed to come up (attempt $attempt)"; kill_mine; sleep 3
done
[ $up = 1 ] || { echo "[run_map] GIVING UP on $LABEL"; exit 1; }
sleep 8

cut -d' ' -f1-3 /proc/loadavg > "$OUTDIR/load1_at_probe.txt"
/home/lucas/hercules-sim/venv/bin/python3 "$SP/p2/runtime_probe.py" "$LABEL" "$OUTDIR" \
    --port $PORT --burst 40 2>&1 | tee "$SP/probe_$LABEL.log"
RC=${PIPESTATUS[0]}
kill_mine
echo "[run_map] $LABEL done rc=$RC"
exit $RC
