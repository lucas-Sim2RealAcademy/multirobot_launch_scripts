# HERCULES 4-Drone Stall — Final Synthesis (source-verified)

Synthesized 2026-08-12 from six source-reading agents over `/home/lucas/hercules-sim/src/{active_exploration, multi_drone_nvblox, px4_offboard, isaac_ros_nvblox, isaac_ros_visual_slam, isaac_ros_common, realsense-ros, multirobot_launch_scripts}` (sim branches, tips as of 2026-08-12). Legend: **CODE** = read in source; **INF** = inferred. Every claim cites file:line.

**Framing.** The symptom "Position → Offboard → explores → gets stuck with errors when coordinating over zenoh + LoRa" has two source-backed readings that are not mutually exclusive:

- **(A) Coordination is causal** — stale-peer frontier zeroing (F2), bridge churn (F3), compute/bandwidth burst (F6), depth-stale guard latch (F7).
- **(B) Correlation is incidental** — multi-drone coordination flights are the only open-field flights, and the team's own 2026-08-12 commits (95a5058, c92b35a) prove the open-field planner wedge fires there regardless of comms (F1). A source-level twist supports (B): in the flown tab6/tab7 config, the zenoh allowlist never bridged the loop-closure/octomap topics at all (F8) — "coordination on" added load and churn but likely exchanged nothing semantic.

---

## 1. Final ranked root-cause list

### F1. Open-field planner wedge family (absorbs R9) — **CONFIRMED, team's own diagnosis**

**Mechanism (CODE).** Four stacked defects in `simple_exploration_planner.py` + launch geometry:
- **1a Phantom ground obstacle:** with zero wall cells in the ESDF slice, the fallback used the raw 3D ESDF, which over open ground is distance-to-floor (~0.9–1.5 m at 1 m flight height) — below `collision_threshold=1.5`, so every known cell reads blocked; 422/422 A* failures (fix 95a5058: `simple_exploration_planner.py:513-527/522-534`).
- **1b Unbounded planning cycle:** sequential Python A* (`max_expansions=5000`, py:100-102) with no time bound inside a **single-threaded executor** (plain `rclpy.spin`, py:1402-1406) ran 105 s in one field cycle; the 3-strike stuck relaxation never fired because `consecutive_no_path` increments only after a cycle ends (py:1187). Fix c92b35a adds `plan_cycle_budget_s=10.0` checked between candidates (py:1074-1079) — still up to 10+ s of callback starvation per cycle.
- **1c FIS bbox never passed anywhere:** fleet passes only `flight_height` to FIS (`fleet_bringup.launch.py:126-129`); single-drone tab3 likewise (`launch_ghost_frontierexplore_tmux.sh:120-121`). FIS keeps default ±10 m (`fis.launch.py:12-17`; `frontier_info_structure_node.cpp:18-23`) as its nvblox ESDF AABB query (cpp:115-120) while the planner works a 35×35 shifted box (`planner_stage.launch.py:34-53`) — frontiers only ever exist within ±10 m of the drone's own origin; near-field exhausted → "No frontiers — DONE" (py:935-938, DONE logs **nothing** while idle).
- **1d Fence beyond valid ESDF:** fleet default `geofence_forward_m=35` (`fleet_bringup.launch.py:174`, commit c35e6d2) exceeds nvblox `workspace_bounds` ±25 m (`nvblox_base.yaml:94-100`); single-drone launchers deliberately used ±24 "1 m inside the nvblox workspace bounds" (`launch_ghost_frontierexplore_tmux.sh:132-135`).
- **1e Takeoff on the fence:** `geofence_behind_m=0` (fleet_bringup:180-182) puts `bbox_min_x=0` on the launch line; `_in_geofence` is strict with 0.3 m margin (py:651-654) so the takeoff point fails its own fence and waypoints there are skipped (py:710-715).
- **1f vehicle_id=0 in fleet mode:** `simple_planner_launch.py` has no vehicle_id arg (lines 9-36); planner defaults 0 (py:307), gating off coverage lanes and lower-id tie-break (py:956, 1042-1043) — two drones can chase the same frontier.
- **1g Scoring ignores geofence:** margin-band viewpoints win scoring (no `_in_geofence` in py:1012-1049), get skipped mid-EXECUTE, replan to the same goal — livelock at the box edge (no frontier blacklist exists anywhere).
- **1h flight_height mismatch:** planner default 1.0 m (py:288) vs FIS 2.0 m (cpp:31) — gain raycasts from a vantage the drone never occupies (cpp:619, 657-699).

**Evidence.** Commits 95a5058 + c92b35a (2026-08-12, field test: "both drones hovered indefinitely, NO PATH with 422/422 A* failures at walls=0"); c35e6d2 ("Previously the fleet planner include passed no bbox at all and silently used the ±10 m default"); all file:lines above.

**Field-log signature.** `NO PATH: tried N/M clusters, all failed (K no-path) ...` every 0.5 s (py:1189-1196); `ESDF drone=(...) ... walls=0 known=...` (py:546-551) — walls=0 is the phantom-ground fingerprint; guard bracketing: `No planner setpoint for X.Xs — holding position` (`reactive_depth_guard_node.cpp:265-272`); mode never drops (guard heartbeat unconditional, cpp:197-199). Silent DONE = FIS box exhausted.

**Fix.** Already landed: 95a5058 + c92b35a (unvalidated in flight). Still needed: pass team box (clamped to ±24) into FIS bbox; `geofence_behind_m≈2`; vehicle_id passthrough; geofence check at scoring time; MultiThreadedExecutor or worker-thread A*.

---

### F2. Stale-peer latch hard-zeroes all frontiers (NEW — the coordination-specific wedge) — **LIKELY**

**Mechanism (CODE).** `_peers_cb` overwrites `peer_claims/peer_positions` only when a `swarm/peers` message arrives — the planner has **no staleness expiry** (py:566-573). Every cycle it hard-zeros viewpoints within `r_keepout=3.0` m of a latched peer position (py:1030-1033) and within `r_claim=8.0` m of a latched lower-id claim (py:1042-1045), plus visited hard-zero (py:1035-1038; declared `visited_penalty=0.1` at py:407 is dead code). Peer expiry lives only upstream in `coordination_node` (`PeerTable.expire`, `coordination_logic.py:112-121`, 3 s), whose spin swallows **all** exceptions silently (`except Exception: pass`, `coordination_node.py:348-349`). **INF:** coordination starts → peers/claims exchanged → coordination_node dies silently or the LoRa link wedges (F4) → planner applies a frozen peer's exclusion discs forever → `scored=[]` → endless NO PATH; the A*-threshold relaxation (py:940-946) can never rescue score-zeroing.

**Field-log signature.** The **no-clusters variant**: `NO PATH: no clusters | took Xms` (py:1194-1196 — the only variant containing "took"), repeating at 0.5 s, with `swarm/peers` silent (`ros2 topic hz` zero) and no coordination_node error output.

**Fix.** Age out peer data in the planner (ignore SwarmPeers older than ~3 s local receive time); log-and-continue instead of `except: pass` in coordination_node.

---

### F3. Zenoh bridge lifecycle churn (R2) — **CONFIRMED**

**Mechanism (CODE).** (1) `launch_fleet_tmux.sh:53` prepends the ZENOH line **unconditionally** on every window run (fleet session guarded at :54; zenoh line not); the line is `tmux kill-session -t zenoh; pkill -f zenoh-bridge-ros2dds; sleep 1; new-session` (`fleet_ctl:156-161,215-220`) — re-attaching after an SSH drop (the documented recovery action) bounces the flying drone's mesh bridge. (2) Cleanup misses the tab8 mesh session: stop sweep greps `^(fleet|zenoh|rviz|loratest)` (`fleet_ctl:231-232`) but tab8's session is `gh_mesh_zenoh` (`launch_ghost_frontierexplore_tmux.sh:170`). (3) tab8 runs the **untracked** workspace-root v1 script via plain `sudo bash` (no `-E`, no domain env, tmux:155-170); v1 "FORCES ROS_DOMAIN_ID=0" with a `/drone.*` whitelist (`CURRENT_STATE.md:24`); if swapped to v2 it refuses to start without ROS_DOMAIN_ID (`launch_mesh_zenoh_v2.sh:71-89`). Only the fleet_ctl path pins domain=vehicle_id (`fleet_ctl:159,218`; `gen_zenoh_config.py:204-208` refuses 0). Restart amplification: BoW topic is RELIABLE+TRANSIENT_LOCAL depth 50 and full descriptors RELIABLE+TRANSIENT_LOCAL (`loop_closure_sp_node.py:333-346`) — every bridge restart triggers a replay storm; `_rebroadcast_bows` exists precisely because "Zenoh route setup can take 2-3s during which published messages are lost" (py:826-856).

**Field-log signature.** Coordination-topic gaps exactly at operator re-attach times; zenoh-bridge process restart in `ps`; burst of duplicate BoW/octomap receives after each restart.

**Fix.** Guard the ZENOH line with `tmux has-session -t zenoh ||`; include `*mesh_zenoh` in the stop sweep; delete/replace untracked v1.

---

### F4. XRCE/LoRa serial-port roulette (R1) — **CONFIRMED as failure class; mid-flight role unproven**

**Mechanism (CODE + one gap).** `coordination_stage.launch.py:50-53` launches `radiohive lora_bridge_node.py` with **zero parameters** at t=14 s of every default fleet bringup (`fleet_bringup.launch.py:112-123,152`; `connectivity_mode` default 'full', :166-167; `fleet_ctl:143-146` never overrides) — violating the repo's own rule "never launch it … without overriding serial_port" (`AGENT_HANDOFF.md:252,441-442`, documented default /dev/ttyUSB0@57600). The PX4 side documents the observed collision: commit 131f194 (2026-08-03) + `vio_bridge.launch.py:10-33` docstring — "ttyUSB0/ttyUSB1 assignment is enumeration-order roulette per boot. Talking XRCE to the LoRa modem fails silently … Position mode is refused (bench postmortem 2026-08-03, delta)". The FTDI auto-detect (idVendor 0403, fallback `/dev/ttyUSB0`, baud 921600 at :43-47) protects **only the agent side**; Linux ttys are non-exclusive, so the LoRa bridge opening the PX4 FTDI at 57600 reconfigures termios and corrupts XRCE framing (**INF**). No port-in-use check exists anywhere; preflight's "lora bridge" check is publisher-existence only (`preflight_check.py:90-91`) and passes with a dead radio. **Timing (CODE):** eager open at t=14 s presents at **bringup** ("no valid local position"), minutes before Offboard — for a mid-exploration stall R1 needs a lazy/retrying open in radiohive, **which is unreadable (repo absent from tree, `CURRENT_STATE.md:25`)** — the decisive missing evidence.

**Field-log signature.** Bringup: agent runs, `TermiosAgentLinux` never establishes a PX4 session / Position refused. Mid-flight (if lazy open): XRCE session drop + garbage on the agent console at the moment LoRa (re)connects; LoRa peers simultaneously absent.

**Fix.** Mirror the auto-detect for LoRa (pick CP210x idVendor 10c4, never 0403) as a `serial_port` param in coordination_stage; bind-mount `/dev/serial/by-id` into the container; add an fd-level cross-check (compare `/proc/<pid>/fd` of agent vs bridge) to preflight.

---

### F5. Offboard one-way trapdoor + no-respawn SPOFs (R3, narrowed) — **CONFIRMED as terminal amplifier**

**Mechanism (CODE).** Zero-hit grep for `VehicleCommand` across all 8 repos: **nothing arms, nothing (re)commands Offboard**. On nav_state loss the planner only regresses: "Lost offboard/armed — returning to INIT" (py:888-897; same `exploration_manager_node.cpp:575-582`) and waits for the RC pilot. The trigger set is **narrowed**: the guard's 30 Hz timer publishes `offboard_control_mode` unconditionally first (`reactive_depth_guard_node.cpp:95-98,197-199`) and every staleness branch still publishes a hold (cpp:246-282), so planner silence/ESDF/depth staleness **cannot** create the 1 s gap. The gap requires process/transport death: guard killed (no respawn, `reactive_guard.launch.py:32-54`), agent/vio_bridge death (no respawn, `vio_bridge.launch.py:50-57,60-65`), R11 pkill, tmux churn. Pre-first-position exception: `publishHold` early-returns if `!vehicle_pos_received_` (cpp:600) — heartbeat without setpoints. `debug_skip_arm_check` forces both armed_ok and offboard_ok true (py:343,889-897) — with `--debug` on (`launch_*_frontierexplore_tmux.sh:45,130`), a real demotion is **masked**.

**Field-log signature.** PX4: Offboard→POSCTL at COM_OF_LOSS_T=1 s; RC-loss→RTL at 1.5 m. ROS: planner INIT spam `INIT: odom=... armed=False offboard=False` (py:904-908) forever; or with --debug, a healthy-looking EXECUTE planner the vehicle ignores.

**Fix.** `respawn=True` on agent/vio_bridge/guard; a supervisor that re-commands Offboard (VehicleCommand MAV_CMD_DO_SET_MODE) after verifying setpoint stream health; split the debug flag; raise COM_OF_LOSS_T for field trials.

---

### F6. Coordination-onset compute/bandwidth burst (R4, mechanism refined) — **LIKELY**

**Mechanism (CODE + INF).** At coordination onset the Orin NX (RAM already 10.0/15.6 GB under the base stack, commit 2c4996f bench note) gains, simultaneously: GPU SuperPoint+LightGlue matching in the 1 Hz timer (`loop_closure_sp_node.py:706-724,1119-1126`; 154 ms + 71 ms on a bigger AGX, `AGENTS.md`) with load **growing** through the mission because `processed_remotes/rejected_bows` are cleared on every local keyframe (py:762-763, up to 200×200 re-matches); full merged-octree rebuilds on **every** remote octomap under the same mutex as the nvblox voxel callback (`octomap_exchange_node.cpp:448,479-506`); a RELIABLE traffic floor — 540 KB full descriptors RELIABLE+TRANSIENT_LOCAL (contradicting `COMMS_PROTOCOL.md:94-96`) plus all BoWs rebroadcast every 10 s (py:826-856). Downstream, the burst can only hurt via **dropout, not delay**: vio_bridge sends `timestamp=0` (`vio_bridge_node.cpp:105-106`) so PX4 stamps on arrival and EKF2_DELAY_MAX never trips (**graveyard**); but covariance is hardcoded 0.01/quality=100 (cpp:121-125) so EKF2 cannot deweight a degrading cuVSLAM — it either lies "perfect" or goes silent. Historical proof of the family: CUDA OOM killed nvblox mid-flight, launch teardown took cuVSLAM with it → VIO stopped → ALTCTL (flights 70/71, commits 2c4996f + 52af58c) — **fixed on sim branches; verify the drones fly builds containing both**. Residual fork burst: up to 21 GPU slice kernels/cycle at 10 Hz when no data at camera height (`nvblox_node.cpp:857-903`); integration distance doubled to 10 m (`nvblox_base.yaml:82`).

**Field-log signature.** cuVSLAM odom rate drop/stop with no error (silence is the failure mode, `visual_slam_impl.cpp:714-727`); guard `Depth image stale (X.Xs) — holding position`; EKF2 EV timeout in PX4 logs; historical: nvblox exit 99.

**Fix.** Forward covariance/quality in vio_bridge; cap rebroadcast + restore BEST_EFFORT per protocol doc; incremental octree merge; confirm 2c4996f/52af58c deployed per drone.

---

### F7. Guard↔planner split-brain + 0.2 s depth latch (NEW) — **LIKELY contributor**

**Mechanism (CODE).** The guard publishes `reactive_guard/blocked` for "the exploration manager" (`reactive_depth_guard_node.cpp:91-92,634-655`) but only the unused `exploration_manager_node.cpp:97` subscribes; the flown `simple_exploration_planner.py` subscription list (py:431-454) contains **no** guard topic. When the guard latches STOP (obstacle <1.0 m, cpp:397-410, or depth stale >`depth_timeout_s=0.2` s, cpp:38,256-263 — ~2 dropped D435i frames; never overridden by launch), the planner keeps believing EXECUTE progresses, waits out `replan_timeout_s=30`, replans through the same disputed spot (guard uses raw depth; planner uses nvblox horizontal ESDF) — permanent silent loop. Bonus hazard: planner hold can latch NED (0,0,0) before the first VehicleLocalPosition (py:866-872,375-379,774-775) → fly-to-origin the moment Offboard engages (**INF** on the operational window).

**Field-log signature.** Alternating `PATH BLOCKED: esdf=...` / guard `Obstacle`-hold + `Replan timeout (30.0s)` cycles at one location; or repeated `Depth image stale (0.2s)` holds clustered at coordination onset.

**Fix.** Subscribe planner to `reactive_guard/blocked` and penalize the corridor; raise `depth_timeout_s` to ~0.5 s; refuse hold-latch until `px4_received`.

---

### F8. Coordination is partly a silent no-op / silent misalignment (NEW family) — **CONFIRMED config facts**

**Mechanism (CODE).** (a) Default zenoh allowlist bridges `/d<i>/...` + `/loop_closures` + `/loop_closure_alignment` only (`gen_zenoh_config.py:36-48`); tab6/tab7 nodes publish bare `/loop_closure/keyframe_bow` and `/drone<i>/octomap` (`loop_closure_sp_node.py:386-398`; `octomap_exchange_node.cpp:65-68`) — bridged only with `--legacy-namespace` (LEGACY_NAMESPACE=1, `launch_mesh_zenoh_v2.sh:100-104`; never set by bench script or fleet_ctl). **In the flown tab config, loop-closure/octomap exchange likely never crossed drones — silently.** (b) `publish_tf:=True` is a dead parameter — node declares `legacy_tf_broadcast` (py:174); rclpy ignores unknown overrides — the refined alignment TF is never broadcast in the tab config. (c) octomap_exchange republishes the **YAML prior** on `/loop_closure_alignment` every 10 s (cpp:262-274,317-323) and alignment_manager treats any message there as a verified loop closure with "never downgrade" (alignment_manager.py:170-189) — a static prior can lock in as truth if stacks mix. (d) `wifi_up/map_ok/batt_pct` initialized and never updated (`coordination_node.py:104-106,270-280`) — claim flags permanently 0, no low-battery signalling. (e) No zenoh↔LoRa failover exists: RADIOHIVE.md's Link Manager is unchecked roadmap (RADIOHIVE.md:452-457); the two links die independently and silently. (f) fleet_bringup ran with **bench** alignment values (0.6 m/270°) for all field runs between 8/7 and 8/12 (swarm_alignment.yaml corrected in c35e6d2); `_geofence()` swallows yaml failures (`except: offsets={}`).

**Why it matters.** Reframes the symptom: "coordination" contributed churn + load (F3/F6) while its semantic layer was partly dead — and any conclusion that map alignment was active in the field is wrong.

**Fix.** One coordination stack per experiment; preflight check that zenoh allowlist matches what the nodes actually publish; loud failure on alignment-yaml errors; distinct topic for priors.

---

### F9. cuVSLAM silent-freeze modes: tracking loss + clock step (R5 recast intra-drone) — **PLAUSIBLE, unguarded**

**Mechanism (CODE).** On `CUVSLAM_TRACKING_LOST` the node warns and resets the pose cache only; **all publishing is inside `if (vo_status == CUVSLAM_SUCCESS)`** (`visual_slam_impl.cpp:714-727`) — total output silence, no invalid flag, and vio_bridge forwards nothing (no watchdog, `vio_bridge_node.cpp:58-137`). Non-monotonic input is swallowed: `MessageStreamSequencer` drops late IMU/images silently (`message_stream_sequencer.hpp:84-87,132-135`), `last_msg_ts_` never resets (`message_buffer.hpp:77-84`), only positive jitter checked (`visual_slam_impl.cpp:671-678`). **INF:** an NTP step on a 1969-booting Jetson (only ghost runs NTP) freezes cuVSLAM output with zero errors until wall clock re-passes pre-step stamps → EV starves → estimator failsafe. Fork delta here is zero (rviz + verbosity only). Related: fork enables IMU fusion but upstream registers every IMU sample at the image timestamp (`visual_slam_impl.cpp:687-688`). Map poisoning is permanent by config: all decay/clear off (`nvblox_base.yaml:21,23,25,52,115`) — any pose-glitch geometry persists all flight.

**Field-log signature.** `/visual_slam/tracking/odometry` rate → 0 with no error line; PX4 EV timeout; possibly correlated with an NTP sync event in journal.

**Fix.** `chronyd makestep` before launch on all four; odom-rate watchdog in vio_bridge; enable slow TSDF decay.

---

### F10. Opposite-sign 20° pitch compensation (NEW) — **CODE FACT, impact needs bench verification**

**Mechanism.** vio_bridge builds R_y(−20°) (`vio_bridge_node.cpp:9-18`; `scripts/vio_bridge.py:36`) while odom_correction builds R_y(+20°) (`odom_correction_node.cpp:9,20-28`) on the **same** input topic; `vio_bridge_node.hpp:23` claims they "must match". **INF:** PX4/EKF2 world and nvblox/planner world pitched relative to each other (up to ~40° worst reading) — planner plans in one frame, controller flies in the other; EKF2 EV misaligned with gravity → growing innovations with travel. (If cuVSLAM gravity-aligns via IMU, one node is a near-no-op and the other is wrong — bench decides.)

**Signature.** Systematic altitude error proportional to horizontal travel; EKF2 EV innovation growth on straight legs.

**Fix.** Bench test (fly 2 m forward in Position; compare EKF2 z vs nvblox TF z); single shared constant/parameter.

---

### F11. Stale/duplicate processes (R7) — **CONFIRMED**

Three live incidents fixed in-window: manually-started agents "hold the serial port against the next launch" (4859cd3); the stop sweep's pkill matched its own shell and died mid-sweep, "one straggler per drone after stop all" (2a43776); 7 stale fleet_tab sessions per drone attach next launch to a dead shell (2ba2258). `MULTI_DRONE_DEBUG_SUMMARY.md:84,163` documents duplicate publishers corrupting data flow. **Signature.** Two guards/agents in `ps`; USB busy; interleaved duplicate setpoints. **Fix.** Idempotent bringup preflight that inventories and refuses on stragglers.

### F12. Legacy domain-0 mesh bridge (R6) — **LIKELY historical**

The previously-flown untracked v1 mesh script forces ROS_DOMAIN_ID=0 against per-drone domains (`CURRENT_STATE.md:24`); v2/gen refuse 0 (`gen_zenoh_config.py:204-208`) and fleet_ctl pins domain=vid, but preflight (the only UXRCE_DDS_DOM_ID proxy check, `preflight_check.py:71-76`) is not auto-run by `fleet_ctl start`. **Fix.** Auto-run preflight in cmd_start; delete v1.

### F13. Bare `fleet_ctl stop` kills all four drones (R11) — **CONFIRMED code, operational hazard**

`vehicles()` returns the whole fleet with no args (`fleet_ctl:91-95`); cmd_stop pkill-9s ros2 launch, containers, `*_node.py`, lora_bridge, vio_bridge, MicroXRCEAgent + host zenoh per vehicle (`fleet_ctl:226-245`); `launch_fleet_tmux.sh:82-84` forwards `stop` → `all`. In-air consequence is R3's trapdoor (F5). **Fix.** Require explicit `all`; refuse stop for armed vehicles unless `--force`.

### F14. Unpinned agent DDS binding at mesh switch (R8) — **premise CONFIRMED, outcome untested**

Agent launched with no FastDDS profile/env/interface pinning and no respawn (`vio_bridge.launch.py:50-57`); TRANSIENT_LOCAL depth-5 vio publisher can replay up to 5 stale poses to a rejoining agent, stamped fresh because timestamp=0 (`vio_bridge_node.cpp:24-28,105-106`) — EV position jump on agent rejoin (**INF**). Bench zenoh path deliberately makes "ZERO network changes", so the mesh-switch case remains field-untested (f3236f9). **Fix.** Pin interfaces via profile; VOLATILE durability on the vio publisher.

### F15. GPS/EV fusion fight outdoors (R10) — **EV half CONFIRMED, fight unproven**

EV always claims 1 cm confidence/quality 100 regardless of tracking (`vio_bridge_node.cpp:121-125`), making EKF2_EV_QMIN meaningless; `VIO_QUICKSTART.txt:16-22` documents EKF2_EV_CTRL=15/HGT_REF=3. GPS-side params unread (not in these repos). **Fix.** Snapshot `param show EKF2_EV*`/GPS params per drone into debug archives; forward real covariance.

---

### Graveyard (refuted, one line each)

- **R5 as stated (cross-drone clock skew breaks coordination):** every hypothesized path refuted in code — pairing timeouts use local receive time (`loop_closure_sp_node.py:523,551,654-657`), `pcm.py` contains no timestamps at all, static TF is stamp-agnostic (py:413), octomap uses `tf2::TimePointZero` (cpp:410-411), LoRa claim packets carry no timestamp (`coordination_node.py:278-281`), and vio timestamp=0 insulates PX4 (`vio_bridge_node.cpp:105-106`). Survives only recast as intra-drone NTP-step freeze (F9).
- **R3's "planner silence → 1 s setpoint gap" trigger:** guard heartbeat is unconditional and every staleness branch publishes a hold (`reactive_depth_guard_node.cpp:197-199,246-282`).
- **R4's "EKF2_DELAY_MAX trips on pipeline latency" sub-mechanism:** timestamp=0 means PX4 stamps EV on arrival; latency is invisible (`vio_bridge_node.cpp:105-106`).
- **R2's "wrong ROS_DOMAIN_ID on the fleet_ctl bridge" sub-claim:** fleet_ctl pins `env ROS_DOMAIN_ID=<vid>` and the generator refuses 0 (`fleet_ctl:159,218`; `gen_zenoh_config.py:204-208`) — the domain bug is real only on the legacy tab8 path (F12).
- **R9's "FIS-vs-planner bbox divergence when both default" sub-claim:** both default to the identical ±10 box (py:284-287; cpp:18-23) — divergence exists only in the fleet/single-drone launch paths that enlarge the planner box (F1c/d).
- **"PCM time window off by ~56 years" (from STUCK-BUG-INVESTIGATION.md):** no such window exists in `pcm.py`.

---

## 2. What their own commits say (2026-07-20 → 08-12)

The team's implicit diagnosis, read from their own fixes:

- **2026-08-03 — "bench plumbing day" (six fixes, all silent-failure class):** XRCE agent talked to the LoRa modem via serial roulette (131f194); zenoh bridge had "no DDS presence at all on ghost" (c3c651c); ROS_LOCALHOST_ONLY=1 broke VIO because the agent ignores it — flip-flopped same day (28f2cdc→379dca9; 2f69427→063276f); keyframe TF lookups could never succeed in a single-threaded executor / listened on the wrong `/d<i>/tf` — "every frame silently skipped, found live" (c3c651c, f1a862d); half-namespaced split-brain and a missing agent (94de230); stop now kills stray agents holding the serial port (4859cd3).
- **2026-08-07:** the stop sweep killed itself mid-sweep leaving one straggler per drone (2a43776).
- **2026-08-12 — field-test morning (five fixes hours before this analysis):** tmux windows died on a host path inside the container (0ce7d64); stale sessions attached to dead shells (2ba2258); windowed/headless launch unified (7e71fe8); fleet geofence finally derived from alignment after "the fleet planner include passed no bbox at all" (c35e6d2); and the two planner root causes — phantom ground at walls=0, 422/422 NO PATH, both drones hovering (95a5058) and the 105 s unbounded A* cycle (c92b35a).
- **What they are NOT touching:** zero commits on COM_OF_LOSS_T/failsafe, EKF2 EV/GPS tuning, NTP/clock sync, or QoS — they are not currently fighting R3/R4/R5/R10 at the source level.
- **Chronic history:** "stuck" long pre-dates multi-drone — 03d94f1/ea3af91/40af19b/2bf7780/5f419e1 (March 2026) all describe stuck-at-obstacle/replanning.
- **Meta-fact:** the fleet launcher itself was launch-blocking-broken until the morning of 8/12 (0ce7d64, 2ba2258) — there has essentially never been a clean full-fleet field run on the current tooling; pre-8/12 field evidence conflates algorithm wedges with launcher artifacts.

**Their implicit ranking = this report's F1 (planner wedge) + F4/F3/F11 (serial/zenoh/stale-process plumbing).** They have not yet seen F2 (stale-peer latch), F7 (guard split-brain), F8 (allowlist no-op), or F10 (pitch sign).

---

## 3. Updated sim plan — HERCULES AirSim rig

Rig inventory (verified on host): `/home/lucas/hercules-sim/` has Cosys-AirSim source (`HERCULES/`), `PX4-1.15.2/` SITL, `settings-fleet-sim.json` (SimpleFlight multi-vehicle, ghost with front_center RGB+depth) and `settings-fleet-px4.json`, `ros2_ws/src/{active_exploration, nvblox_msgs, px4_msgs, px4_offboard}` already staged, `fly_and_capture.py` + `assemble_video.sh` for video, and the prior investigation in `investigation/`. Real nodes run from `/home/lucas/hercules-sim/src` on this Humble host.

Priority order — each experiment demonstrates a ranked cause failing **on video**:

**E1 (FIRST — top cause F1a/b, SimpleFlight, no PX4): phantom-ground A/B.**
Run the real `simple_exploration_planner.py` at the commit **before** 95a5058, vs the 8/12 tip. Fastest wiring: bypass nvblox entirely — a ~50-line synthetic publisher emitting the exact inputs the planner consumes: a flat-open-field 3D ESDF `PointCloud2` on `nvblox_node/static_esdf_pointcloud` where every known cell's value = distance-to-floor 0.9–1.5 m (py:345-347,432), plus `fis/frontier_viewpoints|centroids|gains` (py:434-439), odom, and a stubbed `/fmu/out/vehicle_status` with `nav_state=14, arming_state=2` at the planner's BEST_EFFORT+TRANSIENT_LOCAL QoS (py:424-429) — or set `debug_skip_arm_check:=true` (py:343). AirSim ghost (SimpleFlight, `settings-fleet-sim.json`) hovers over open grass while a small adapter maps `/planning/trajectory_setpoint` → AirSim `moveToPositionAsync` for the visual. Expected video: takeoff → brief exploration → permanent hover with overlay of `NO PATH: tried N/N clusters ... | walls=0` spam every 0.5 s (pre-fix), vs continued exploration (tip). This validates the team's own diagnosis and gives the baseline harness for everything below.

**E2 (F2, no PX4): stale-peer latch.**
Same harness, tip planner with `vehicle_id:=2`. A scripted `swarm/peers` publisher plants a lower-id peer claim within 8 m of the remaining frontiers, then **stops publishing** (simulating coordination_node's silent `except: pass` death or a LoRa wedge). Expected: planner transitions to endless `NO PATH: no clusters | took Xms` while the peer topic is silent — frozen exclusion discs on video (render the 3 m/8 m discs as markers). A/B against a 5-line age-out patch. This is the coordination-correlated wedge demo.

**E3 (F7, no PX4, uses real AirSim depth): 0.2 s depth-stale latch + split-brain.**
Run the real `reactive_depth_guard` on AirSim front_center depth + camera_info; a relay node injects 0.3–0.5 s depth gaps in bursts ("coordination onset"). Expected: guard latches `Depth image stale (0.3s) — holding position` while the planner (no `reactive_guard/blocked` subscription, py:431-454) keeps cycling EXECUTE → `Replan timeout (30.0s)` — drone frozen, planner oblivious, on video.

**E4 (F5, PX4 SITL required): the Offboard trapdoor.**
`PX4-1.15.2` SITL + `settings-fleet-px4.json` + real MicroXRCEAgent (UDP) + real guard + planner. Enter Offboard, mid-exploration `pkill -9 -f reactive_depth_guard` (exactly what `fleet_ctl stop` does, fleet_ctl:226-245). Expected: 1 s later PX4 demotes to POSCTL (COM_OF_LOSS_T=1); nothing re-commands (zero VehicleCommand publishers repo-wide); planner logs `Lost offboard/armed — returning to INIT` forever. Video shows the one-way trapdoor and motivates respawn + re-command supervisor.

**E5 (F3, no PX4): zenoh churn replay storm.**
Two ROS domains on this host bridged by two `zenoh-bridge-ros2dds` processes configured by the real `gen_zenoh_config.py`; run real `loop_closure_sp_node` pairs exchanging BoWs. Mid-run, execute the literal fleet_ctl zenoh_line (kill-session + pkill + restart). Measure/overlay: peer-topic gap, then the TRANSIENT_LOCAL replay burst (BoW depth 50 + 540 KB descriptors) on restart. Also demos F8 by flipping LEGACY_NAMESPACE to show the tab6/7 topics bridging nothing by default.

**E6 (F4, bench-style, PX4 SITL over pty): serial roulette.**
`socat` a pty pair; run SITL's XRCE over ptyA@921600 via the agent; a script opens ptyA at 57600 mid-session (emulating lora_bridge's default). Expected: termios reconfiguration corrupts XRCE framing, agent session drops on camera. (SITL normally uses UDP — the pty path is required to show the serial mechanism; the enumeration-roulette half is reproduced by swapping which pty "is" ttyUSB0.)

Cross-cutting sim notes: for E1–E3 keep `pose_source` default (cuvslam) but feed ground-truth odom and set the odom_correction pitch to 0 (or bypass it) to sidestep F10 unless explicitly testing it; if running real nvblox instead of the synthetic ESDF, use the fork (pool prealloc 2c4996f) on the 5090.

---

## 4. Answers digest (original questions, as returned)

| Q | One-line answer | Citation |
|---|---|---|
| Q1 | Yes — fleet bringup starts `radiohive lora_bridge_node.py` at t=14 s with **zero parameters** (defaults govern), minutes before manual Offboard entry. | coordination_stage.launch.py:50-53; fleet_bringup.launch.py:112-123,152,166-167 |
| Q2 | fleet_ctl passes no bbox/flight_height → planner gets a derived 35×35 forward box (beyond ±25 m ESDF), FIS gets flight_height only and keeps bbox ±10; defaults diverge (planner 1.0 m vs FIS 2.0 m). | fleet_ctl:143-146; fleet_bringup.launch.py:126-129,173-182; planner_stage.launch.py:34-53; fis.launch.py:12-22; nvblox_base.yaml:94-99 |
| Q3a | fleet_bringup runs a different coordination stack than the 9-tab flow — it never starts loop_closure_sp_node/octomap_exchange (those are tabs 6/7, legacy `drone` prefix); all staging is fixed TimerActions with no readiness gating. | fleet_bringup.launch.py:23-34,90-155; launch_ghost_frontierexplore_tmux.sh:137-153 |
| Q3b | Neither coordination node blocks synchronously on peers — async pending-BoW with 3 s local expiry, no retry; vanished peers leave stale state (keyframes, octrees, T_align) held forever but nothing wedges. | loop_closure_sp_node.py:158,547,652-663; octomap_exchange_node.cpp:100-103,382-393,486-491 |
| Q4 | A 1969-clock peer breaks nothing in these nodes: local receive-time everywhere, PCM has no time window, static TF is stamp-agnostic, octomap uses TimePointZero; only the shared-mapper path re-broadcasts peer stamps, and FIS/planner don't consume it. | loop_closure_sp_node.py:523,551,654-657; pcm.py (no time refs); octomap_exchange_node.cpp:410-411; keyframe_exchange.py:135; peer_map_integrator.py:202,222 |
| Q5 | tab8 runs the **untracked** v1 script via plain `sudo bash` (forces domain 0, `/drone.*` whitelist); v2 requires caller-exported ROS_DOMAIN_ID; only fleet_ctl derives domain=vehicle_id automatically; bench/mesh allowlists identical, differing in localhost-only, endpoints, and WiFi handling. | launch_ghost_frontierexplore_tmux.sh:155-170; CURRENT_STATE.md:24; launch_mesh_zenoh_v2.sh:71-89; fleet_ctl:149-161; gen_zenoh_config.py:36-48,98-121 |
| Q6 | The guard's 30 Hz heartbeat is unconditional; every staleness case (depth >0.2 s, planner >5 s, never-received) degrades to a published hold — the only no-setpoint path is "no PX4 position ever received". | reactive_depth_guard_node.cpp:95-98,197-199,246-282,600 |
| Q7 | Nothing in any repo publishes VehicleCommand (zero grep hits) — Offboard loss is detected only to regress to INIT and wait for RC; `debug_skip_arm_check` forces both armed_ok and offboard_ok true, masking demotion. | repo-wide grep; simple_exploration_planner.py:343,888-897; exploration_manager_node.cpp:575-582 |
| Q8a | Idle paths: INIT gate (armed+nav_state 14), stale-ESDF wait, empty-frontiers → silent DONE, all-zero scores → endless NO PATH (no blacklist, no give-up); 3-strike relaxation affects A* threshold only, never score-zeroing. | simple_exploration_planner.py:893-946,1035-1049,1187-1197 |
| Q8b | Planner waits on nothing multi-drone — local ESDF + local FIS only; peers only discount/zero scores; peer data latches forever with no staleness expiry. | simple_exploration_planner.py:345-347,432,566-573,1022-1045; frontier_info_structure_node.cpp:28-30,77-78 |
| Q8c | c92b35a caps a previously unbounded A* loop (105 s observed; single-threaded executor starved all callbacks incl. stuck detection); sibling 95a5058 is the actual unstick (walls=0 phantom ground). | git c92b35a/95a5058; simple_exploration_planner.py:100-102,297-302,522-534,1074-1079,1402-1406 |
| Q9 | Guard `/fmu/in` pubs are default RELIABLE/VOLATILE depth 10 (nonstandard but DDS-compatible); planner's PX4 subs are BEST_EFFORT+TRANSIENT_LOCAL (breaks if agent writers go VOLATILE → wedged INIT); vio_bridge pub is BEST_EFFORT+TRANSIENT_LOCAL depth 5 (stale replay on agent rejoin); coordination topics all default RELIABLE. | reactive_depth_guard_node.cpp:55-92; simple_exploration_planner.py:424-462; vio_bridge_node.cpp:23-34 |
| Q10 (returned as "Q-logs") | Full stuck-log inventory extracted — key discriminator: `NO PATH: tried N/M...` (A* failures) vs `NO PATH: no clusters | took Xms` (all scores zeroed → implicates peers/visited zeroing, not ESDF); DONE state logs nothing. | simple_exploration_planner.py:904-1196 passim; frontier_info_structure_node.cpp:103-186; reactive_depth_guard_node.cpp:242-270 |
| Q11 | `MicroXRCEAgent serial --dev <raw /dev/ttyUSBn> -b 921600`, no env, no FastDDS profile, no respawn; find_px4_serial picks the first idVendor-0403 tty, falls back to ttyUSB0, protects only the agent side. | vio_bridge.launch.py:10-33,43-47,50-57 |
| Q12 | vio_bridge sends timestamp=0 (Jetson clock never reaches PX4), drops covariance (hardcoded 0.01/quality 100), and applies R_y(−20°) while odom_correction applies R_y(+20°) to the same topic — parallel consumers, opposite signs. | vio_bridge_node.cpp:9-18,65-101,105-106,121-125; odom_correction_node.cpp:9,20-28; realsense_example.launch.py:211-224 |
| Q13 | radiohive source is absent from all 8 repos — default ttyUSB0@57600 is documentation-only; no port-in-use check exists; zenoh↔LoRa failover is unimplemented design (Link Manager roadmap unchecked); preflight's LoRa check passes with a dead radio. | CURRENT_STATE.md:25; AGENT_HANDOFF.md:252,441-442; RADIOHIVE.md:452-457; preflight_check.py:90-91 |
| Q14 | cuVSLAM (fork = upstream here) silently drops non-monotonic input with no reset path and goes fully silent on TRACKING_LOST; nvblox never decays/clears, so pose-glitch geometry poisons the map permanently. | message_stream_sequencer.hpp:84-87,132-135; message_buffer.hpp:77-84; visual_slam_impl.cpp:671-727; nvblox_base.yaml:21-25,52,115 |
| Q15 | Workspace bounds ±25 m confirmed; all staleness/decay disabled; the fork's moving ESDF slice searches ±4 m (up to 21 GPU kernels/cycle) and **still publishes an empty slice** on failure; use_sim_time correct for live flight but hardcoded True in namespaced_nvblox.launch.py (sim-only). | nvblox_base.yaml:94-100,18-25; nvblox_node.cpp:784-787,828-947; namespaced_nvblox.launch.py:70-350 |

---

## Decisive missing evidence (next reads)

1. **radiohive `lora_bridge_node.py`** — serial-open semantics (eager vs lazy/retrying) and true defaults: adjudicates F4's mid-flight role.
2. **Per-drone PX4 param snapshots** (`COM_OF_LOSS_T`, `EKF2_EV_*`, GPS noise) — F5/F15 quantification; VIO_QUICKSTART documents EV_QMIN=1 but vehicles reportedly run 0.
3. **Deployed branch per drone** vs sim tips — drones on pre-2c4996f/52af58c builds still carry the flights-70/71 OOM cascade; drones on pre-131f194 px4_offboard still have agent-side roulette.
4. **Field debug archives cross-matched against the exact log strings in Q10** — the two NO PATH variants split F1 from F2 conclusively.