#!/usr/bin/env bash
# Sequential n>=3 A/B for the deadband fix.
#
#   BASE (stock)     : HERC_MIN_GOAL_DIST_M=0  ROTATE_IN_PLACE_M=0.6 (default)
#                      -> byte-identical behaviour to the pre-investigation tree
#   LADDER (fix)     : HERC_MIN_GOAL_DIST_M=1.0 HERC_ROTATE_IN_PLACE_M=0.25
#
# ROTATE_IN_PLACE_M=0.25 is part of the fix, not a separate variable: at the
# stock 0.6 every 0.5 m intermediate waypoint (waypoint_spacing) sits inside the
# bridge's rotate-only radius, so capping the GOAL distance alone would still
# leave the vehicle yawing instead of advancing between waypoints.
#
# usage: batch_ab.sh
set -u
R=/home/lucas/hercules-sim/investigation/run_exp.sh
LOGD=/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad

run () {   # run <label> <env assignments...>
  local label=$1; shift
  [ -d "/home/lucas/hercules-sim/e1_frames/runs/$label" ] && {
      echo "SKIP $label (already archived)"; return 0; }
  echo "############ $label : $* ############"
  env "$@" "$R" "$label" 150 > "$LOGD/${label}.log" 2>&1
  echo "  -> exit $? ; $(grep -c 'archived ->' "$LOGD/${label}.log" 2>/dev/null) archived"
}

run b2_base HERC_MIN_GOAL_DIST_M=0
run b3_base HERC_MIN_GOAL_DIST_M=0
run t1_ladder HERC_MIN_GOAL_DIST_M=1.0 HERC_ROTATE_IN_PLACE_M=0.25
run t2_ladder HERC_MIN_GOAL_DIST_M=1.0 HERC_ROTATE_IN_PLACE_M=0.25
run t3_ladder HERC_MIN_GOAL_DIST_M=1.0 HERC_ROTATE_IN_PLACE_M=0.25
echo "############ BATCH COMPLETE ############"
