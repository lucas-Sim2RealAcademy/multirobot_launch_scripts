#!/usr/bin/env bash
# Ladder A/B re-test at the corrected VIO operating point.
#
# Common to BOTH arms (the new defaults): HERC_CAPTURE=0 HERC_DEPTH_HZ=10.0
#   -> ~17.7-18.7 Hz stereo, and cuVSLAM stutter inside the field reference of
#      <=2 events/min.  The real gate is image_jitter_threshold_ms:=200.0
#      (run_fleet_radio.sh:176), not a nominal 30 Hz frame period.
#
#   BASE   : stock planner (min_goal_dist 0, ROTATE_IN_PLACE_M default 0.6)
#   LADDER : min_goal_dist 1.0 + ROTATE_IN_PLACE_M 0.25
#            (ladder: 0.25 rotate < 0.30 pos_tol < 0.50 wp_spacing < 1.0 goal)
#
# Arms are INTERLEAVED so host-load drift is shared, not confounded with arm.
set -u
R=/home/lucas/hercules-sim/investigation/run_exp.sh
L=/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad
COMMON="HERC_CAPTURE=0 HERC_DEPTH_HZ=10.0"
run () { local lbl=$1; shift
  [ -d "/home/lucas/hercules-sim/e1_frames/runs/$lbl" ] && { echo "SKIP $lbl"; return; }
  echo "######## $lbl : $* ########"
  env $COMMON "$@" "$R" "$lbl" 150 > "$L/${lbl}.log" 2>&1
  echo "  exit $? $(grep -c 'archived ->' "$L/${lbl}.log")"
}
run L1_base   HERC_MIN_GOAL_DIST_M=0
run L1_ladder HERC_MIN_GOAL_DIST_M=1.0 HERC_ROTATE_IN_PLACE_M=0.25
run L2_base   HERC_MIN_GOAL_DIST_M=0
run L2_ladder HERC_MIN_GOAL_DIST_M=1.0 HERC_ROTATE_IN_PLACE_M=0.25
run L3_base   HERC_MIN_GOAL_DIST_M=0
run L3_ladder HERC_MIN_GOAL_DIST_M=1.0 HERC_ROTATE_IN_PLACE_M=0.25
echo "######## BATCH COMPLETE ########"
