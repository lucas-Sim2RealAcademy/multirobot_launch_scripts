#!/bin/bash
# PHASE-2 candidate sweep.  Sequential: two of my own UE processes would contend and the
# timing numbers would mean nothing.  BLOCKS is re-run at the END as a second control so
# the candidates are bracketed by two controls taken in the same session.
SP=/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad
cd "$SP" || exit 1
run () {
  echo "=================== $1 ($2) load1=$(cut -d' ' -f1 /proc/loadavg) $(date +%H:%M:%S)"
  ./run_map.sh "$1" "$2"
  echo "--- $1 rc=$?"
}
run HI_OVERVIEW  /Game/Home_Interior/Maps/Home_Interior_Overview
run MOON_DEMO    /Game/LowPolyMoonPack/Maps/Demo
run MOON_OVERVIEW /Game/LowPolyMoonPack/Maps/Overview
run TECHART      /Game/Maps/TechArt
run GO2_OVERVIEW /Game/Maps/Overview
run HOME_INTERIOR /Game/Home_Interior/Maps/Home_Interior
run DEMONSTRATION /Game/Maps/Demonstration
run JAPANFEST    /Game/Maps/JapanFest_Street
run BLOCKS2      ""
echo "SWEEP COMPLETE $(date +%H:%M:%S)"
