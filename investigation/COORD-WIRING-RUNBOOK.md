# HERCULES 4-Drone Sim — Coordination-Layer Wiring Runbook (Run A / Run B)

All paths absolute. `[cf]` = code fact with citation; `[inf]` = inference; `[ver]` = verified by execution on this host during synthesis. Base stacks (cuVSLAM+odom_correction+nvblox+FIS+planner) are assumed already running per domain; this runbook adds only the coordination layer.

---

## 0. DECISIONS AND FLAGS — read before wiring

**FLAG-1 (the B-feasibility flag you asked for): Run B requires writing ONE new emulator file, ~120–180 lines of stdlib Python. It is NOT infeasible, but it exceeds the 100-line threshold.** `[cf]` No pty/socat/virtual-radio tooling exists anywhere in the four repos (grep over `*.py|*.sh|*.md` in radiohive, multi_drone_nvblox, active_exploration, multirobot_launch_scripts: zero hits for an emulator; the `test_radiohive_{pair,trio,mesh}.sh` scripts are 802.11s/Zenoh HARDWARE tests — `iw`/`nmcli`/`rmmod`/`docker exec`, no LoRa serial coverage). Reusable pieces that shrink the emulator (prefer these, per your instruction):
- `mdn_core.lora_packet.lora_airtime_ms()` — exact Semtech SX127x time-on-air formula (`/home/lucas/hercules-sim/src/multi_drone_nvblox/mdn_core/lora_packet.py:173-198`) and `slot_margin()` (`:201-232`) — import these for the slot-fit cap; zero formula code to write.
- `/home/lucas/hercules-sim/src/multi_drone_nvblox/test/test_lora_packet.py` — stdlib unit tests for the claim codec; run to sanity-check claim passthrough.
- Frame constants/builder to mirror: `/home/lucas/hercules-sim/src/radiohive/scripts/lora_bridge_node.py:330-335` (`_frame`), firmware parser `/home/lucas/hercules-sim/src/radiohive/firmware/lora_mesh_v7_2_ros2.ino:268-335`.

**FLAG-2 — name↔id conflict.** Task brief says ghost=1, delta=2, buckshee=3, thunderstrike=4. The repos disagree: `[cf]` `fleet_ctl:29-35` = delta=1, buckshee=2, ghost=3, thunderstrike=4 (test_radiohive_mesh.sh:11-14 and trio.sh:5-8 agree with fleet_ctl). **This runbook pins the TASK mapping (ghost=1 …) as labels only** — ids drive everything real (`ROS_DOMAIN_ID`, `/d<i>`, `127.0.0.<i>`); names are terminal labels. Record the deviation in the run log.

**FLAG-3 — planner `vehicle_id` gap is AS-FLOWN.** `[cf]` Neither `planner_stage.launch.py:43-53` nor `simple_planner_launch.py:44-68` passes `vehicle_id`; planner default is 0 (`simple_exploration_planner.py:307`), read once at init (no dynamic update). With 0: ModeState publishing and lane assignment are skipped (`:956`, `:974`) and the lower-id-wins tie-break is skipped (`:1042-1044`), while ClaimIntent publishing and claim/pose discounts still run (gated only on COORD_OK, `:726`, `:1160`, `:1022-1027`). **Run A: leave it (field behavior includes this gap). Run B: patch it (§3.4).**

**FLAG-4 — the embedded lora_bridge cannot take a serial port.** `[cf]` `coordination_stage.launch.py:50-53` launches `lora_bridge_node.py` with NO parameters → auto-detect, which only walks USB sysfs for CP2102 10c4:ea60 and never finds a pty (`lora_bridge_node.py:58-59,76-103,194`). Run B therefore uses `connectivity_mode:=wifi_only` (skips only the embedded bridge node — keyframe_exchange/peer_map_integrator are gated on `conn != 'lora_only'` and stay up, `coordination_stage.launch.py:50-73`) plus an external `radiohive/lora_bridge.launch.py` with explicit `serial_port` (`launch/lora_bridge.launch.py:15-33`). Zero repo edits. Yes, the mode name is ironic for the LoRa run — it only gates that one node.

**FLAG-5 — claim packet fits the TDMA slot: no blocker.** `[ver]` Computed with the repo's own formula: `slot_margin(19, base_packet=39)` (N=4 base) → 58 B total, **28.224 ms airtime, fits=True**; also ≤ the stricter firmware budget `SLOT_MS − 2·TX_MARGIN = 30 ms` (`.ino:43,59,388-392`). The codec docstring documents the same design intent at N=3 (57 B = 26.94 ms, `lora_packet.py:230-246`). Claims are never size-dropped.

**FLAG-6 — wire id contract (emulator MUST honor).** `[cf]` `coordination_node._on_peer_pose` maps `vehicle_id = PeerPose.node_id + 1` (`coordination_node.py:196-204`) → the emulator writes **0-based mesh id = vehicle_id − 1 (0..3)** in every 0x81 node_id byte. `[cf]` Claim packets carry `src_id = vehicle_id` (1-4) directly (TX `coordination_node.py:278`; RX uses `pkt['src_id']`, ignores the serial 0x82 src_node byte, `:206-214`) → claim payloads pass through opaque, no remap.

**FLAG-7 — host facts.** `[ver]` No CP2102 / no `/dev/ttyUSB*|ttyACM*` on this host → Run A degradation happens naturally. `socat` is NOT installed → prefer the `os.openpty()` emulator variant (or `sudo apt install socat`). Zenoh bridge binary exists: `/home/lucas/hercules-sim/bin/zenoh-bridge-ros2dds`. Workspace: source the overlay that installs the four packages (e.g. `/home/lucas/hercules-sim/ros2_ws/install/setup.bash` — adjust to the sim's actual overlay).

**FLAG-8 — stale in-repo items to ignore:** `radiohive/config/zenoh_drone{1,2,3}.json5` (eduroam IPs, old allowlist — do not use); `preflight_check.py:91` checks `/lora/peer_pose` at ROOT but the bridge lives under `/d<i>` → it will falsely report the bridge down `[cf+inf]`.

---

## 1. COMMON TO BOTH RUNS

### 1.1 Per-drone environment (every shell that launches coordination nodes)
```bash
# i = 1(ghost) 2(delta) 3(buckshee) 4(thunderstrike)   <- task mapping, see FLAG-2
export ROS_DOMAIN_ID=$i          # [cf] fleet_bringup takes it from env, never forces (fleet_bringup.launch.py:36-37)
source /home/lucas/hercules-sim/ros2_ws/install/setup.bash
# ROS_LOCALHOST_ONLY: match whatever the existing sim stacks use (decision rule in 1.3).
```

### 1.2 One-time host prep (root) — loopback DDS discovery for the bridges
```bash
sudo ip link set lo multicast on
sudo ip route replace 239.255.0.0/16 dev lo
# [cf] exactly what launch_bench_zenoh.sh:100-111 does on vehicles; SPDP multicast on lo.
```

### 1.3 Generate 4 zenoh configs — as-flown allowlist, loopback, NO post-processing
`[ver]` `--subnet` is an unvalidated string prefix (`gen_zenoh_config.py:31,89-92`), so distinct loopback ADDRESSES on one port replace the per-port hack; executed and confirmed for d1 (listen `tcp/127.0.0.1:7447`, connect `127.0.0.{2,3,4}:7447`, domain 1, nodename `zenoh_bridge_d1`, 22 allow patterns).
```bash
GEN=/home/lucas/hercules-sim/src/multi_drone_nvblox/scripts/gen_zenoh_config.py
for i in 1 2 3 4; do
  peers=$(for j in 1 2 3 4; do [ $j -ne $i ] && printf '%s ' $j; done)
  python3 "$GEN" --self-id $i --peer-ids "${peers% }" --domain-id $i \
      --subnet 127.0.0 --output /tmp/zenoh_sim_d$i.json5
done
```
Decision rule `[cf]`: if the sim stacks run `ROS_LOCALHOST_ONLY=0` (field-faithful per `fleet_ctl:73-87`), append `--no-localhost-only` to each call (`gen_zenoh_config.py:177-180`; records the 2026-08-03 live failure). If they run `=1`, keep the default. `--domain-id` must equal that drone's `ROS_DOMAIN_ID` (enforced, `:204-213`).

### 1.4 Launch the 4 bridges
```bash
ZB=/home/lucas/hercules-sim/bin/zenoh-bridge-ros2dds
for i in 1 2 3 4; do
  env CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="127.0.0.1"/></Interfaces></General><Discovery><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>' \
      RUST_LOG='zenoh=warn,zenoh_plugin_ros2dds=info' ROS_DOMAIN_ID=$i \
      "$ZB" -c /tmp/zenoh_sim_d$i.json5 > /tmp/zenoh_sim_d$i.log 2>&1 &
done
# MaxAutoParticipantIndex=200 is mandatory with 4 full stacks on one lo (launch_bench_zenoh.sh:115).
# Drop the <Interfaces> pin only in the ROS_LOCALHOST_ONLY=0 variant.
```

### 1.5 Sanity
```bash
ss -ltnp | grep 7447                       # four listeners 127.0.0.1-4:7447
grep -iE 'route|session' /tmp/zenoh_sim_d1.log
ROS_DOMAIN_ID=2 ros2 topic list | grep '^/d1/'
# EXPECT: /d1/alignment/*, /d1/keyframes/*, /d1/swarm/*, /d1/octomap*, /d1/loop_closure/*
# MUST NOT appear: /d1/lora/*, /d1/coordination/*  (not in allowlist, gen_zenoh_config.py:36-53)
```
Link split `[cf]`: zenoh carries SLAM/alignment/keyframes (+`/loop_closure_alignment`); the only cross-domain consumers are `alignment_manager.py:131-136` and `peer_map_integrator.py:137-143`. Coordination poses+claims ride LoRa ONLY (`/swarm/peers` is remapped to root and NOT bridged; PeerTable ingests exclusively from `lora/peer_*`, `coordination_node.py:116,130-136`). No double-delivery path exists.

---

## 2. RUN A — FIELD-FAITHFUL (no LoRa hardware → coordination degradation)

### 2.1 Launch (per drone, in its domain shell)
```bash
ros2 launch multi_drone_nvblox coordination_stage.launch.py \
    drone_id:=$i \
    alignment_yaml:=/path/to/swarm_alignment.yaml \
    connectivity_mode:=full          # as-flown; embedded bridge starts param-less
```
(If integrating estimation too, `fleet_bringup.launch.py drone_id:=$i alignment_yaml:=… pose_source:=cuvslam` stages the same nodes at t=14, `fleet_bringup.launch.py:112-155`.) Omit `alignment_yaml` only if you accept odom-frame peer poses + `alignment_ok=0` (`alignment_manager.py:197-199`; `coordination_node.py:179-194,274-275`).

### 2.2 Mechanism `[cf]`
Bridge auto-detect finds no CP2102 → `_open()` False → reader loop retries every 2 s forever; TX from coordination_node silently dropped (`lora_bridge_node.py:193-212,337-347`). radiohive msgs ARE installed → `RADIOHIVE_OK=True` → coordination_node keeps its lora pubs/subs and transmits into the void — the exact radio-unplugged field state. PeerTable stays empty; `/swarm/peers` publishes every 0.19 s with `self_id` set and zero peers (`coordination_node.py:153,288-322`); planner flies solo. Do NOT use `wifi_only` here — skipping the bridge node is a slightly different (also valid) degradation; radio-dead-with-bridge-running is the faithful one `[inf]`. Leave the planner unpatched (`vehicle_id=0` is as-flown, FLAG-3).

### 2.3 Verify
```bash
ros2 topic hz /swarm/peers                       # ~5.26 Hz (0.19 s cycle)
ros2 topic echo /swarm/peers --once              # peers: []  self_id: $i
ros2 topic hz /d$i/lora/peer_pose                # NO publications, topic exists
ros2 topic echo /d$i/swarm/link_stats --once     # lora_bridge_up: false (coordination_node.py:328)
# bridge log: no 'opened LoRa serial', reconnect backoff only
```

---

## 3. RUN B — DESIGN-FAITHFUL (virtual LoRa TDMA radio + real lora_bridge x4)

### 3.1 Serial virtualization — exact commands
Preferred (no deps, `socat` absent on host — FLAG-7): the emulator creates the ptys itself.
```python
mfd, sfd = os.openpty()                              # x4, one per drone
tty.setraw(sfd)                                      # raw; NO echo (echo corrupts framing)
os.symlink(os.ttyname(sfd), f"/tmp/hercules_lora/drone{i}")   # stable path, mkdir -p first
```
socat alternative (needs `sudo apt install socat`):
```bash
mkdir -p /tmp/hercules_lora
for i in 1 2 3 4; do
  socat -d -d pty,raw,echo=0,link=/tmp/hercules_lora/drone$i \
              pty,raw,echo=0,link=/tmp/hercules_lora/radio$i &
done   # emulator opens radio$i side
```
`[cf]` Viability: `SerialPort` opens any path with `O_RDWR|O_NOCTTY|O_NONBLOCK` + raw termios, VMIN=0 — works on a pty slave, baud is a no-op (`lora_bridge_node.py:106-125`). `serial_port` MUST be explicit (auto-detect never finds a pty). Bridge survives emulator restarts via the 2 s reconnect loop.

### 3.2 Bridge instances (per drone, in its domain shell)
```bash
ros2 launch multi_drone_nvblox coordination_stage.launch.py \
    drone_id:=$i alignment_yaml:=/path/to/swarm_alignment.yaml \
    connectivity_mode:=wifi_only            # skips ONLY the param-less embedded bridge (FLAG-4)

ros2 launch radiohive lora_bridge.launch.py \
    namespace:=d$i \
    serial_port:=/tmp/hercules_lora/drone$i \
    status_poll_hz:=1.0                     # optional: lora/mesh_status observability
# topic_prefix stays 'lora' so /d$i/lora/{peer_pose,peer_msg,tx_pose,tx_msg} match
# coordination_node's wiring (coordination_node.py:121-136).
```

### 3.3 Virtual radio emulator — behavioral spec (the ~120–180-line file, FLAG-1)
One process, opens all 4 ptys. May `sys.path` in `/home/lucas/hercules-sim/src/multi_drone_nvblox` and import `mdn_core.lora_packet` for airtime.

Constants (firmware at N=4): `NUM_NODES=4, SLOT_MS=40, GUARD_MS=30 → CYCLE_MS=190 ms, TX_MARGIN_MS=5, NODE_TIMEOUT_MS=3000` (`.ino:42-45,58-59`). Matches `cycle_period_s=0.19` default in coordination_node (`coordination_node.py:81-93`) — keep both at 190 ms.

Framing (both directions, little-endian): `[0xAA][type][len][payload…][chk]`, `chk = XOR(type, len, payload)`, sync excluded (`lora_bridge_node.py:330-335`; `.ino:199-208`). Resync = advance 1 byte past 0xAA on bad checksum (`lora_bridge_node.py:260-261`).

| Dir | Type | Payload | Emulator action |
|---|---|---|---|
| bridge→radio | 0x01 POSE_UPDATE | `<ffff` x,y,z,yaw_deg (16 B) | store node i pose, last-write-wins (`.ino:314-321`) |
| bridge→radio | 0x02 CUSTOM_MSG | dest(1B, 0xFF=bcast)+data≤160 | if pending: DROP silently (single-deep, `.ino:323-331`); else store |
| bridge→radio | 0x03 STATUS_REQ | empty | reply 0x83 on same pty: `[my_id][num_alive]` + per-peer `[id][rssi i16][snr f32]` (`.ino:245-263`) — optional |
| radio→bridge | 0x81 PEER_POSE | `<Bffffhf` node_id,x,y,z,yaw,rssi,snr (23 B) | in sender i's slot, write to every OTHER pty j; **node_id = i−1 (0-based, FLAG-6)**; synth rssi=−40, snr=8.0 |
| radio→bridge | 0x82 PEER_MSG | src(1B)+data | piggyback pending custom in sender's slot iff (dest==0xFF or dest==j's mesh id) AND fits (below); clear pending after slot even if dropped (`.ino:505-527,621-625`) |
| radio→bridge | 0x84 PEER_LOST / 0x85 PEER_JOINED | id / `<Bhf` | optional — **coordination_node never subscribes peer_event/mesh_status** `[cf]` (`coordination_node.py:130-151`); include only for debug |

TDMA loop: free-running clock; node i "transmits" at `cycle_start + i_mesh*40ms + 5ms`. One custom per node per cycle, max. Slot-fit rule (firmware-strict): drop custom iff `lora_airtime_ms(39 + custom_len) > 30.0` (`.ino:388-392`; import from `lora_packet.py:173`). The 19 B claim always passes (FLAG-5). **Log every custom drop with a counter** (feeds §4.3). Optional realism knobs `[inf]`: per-receiver drop probability, delivery delay = slot offset + airtime (~13–28 ms), distance-based partitioning with 3 s → PEER_LOST. Sequence numbers / neighbor_mask / cycle_clock never cross the serial boundary — do not emulate.

### 3.4 Planner `vehicle_id` patch (design-faithful only; 4 lines across 2 files, apply in the overlay)
```
/home/lucas/hercules-sim/src/multi_drone_nvblox/launch/planner_stage.launch.py:46  (launch_arguments dict)
    +            'vehicle_id': str(drone_id),
/home/lucas/hercules-sim/src/active_exploration/launch/simple_planner_launch.py
    +        DeclareLaunchArgument('vehicle_id', default_value='0'),      (args list, :8-37)
    +            'vehicle_id': LaunchConfiguration('vehicle_id'),          (parameters dict, :44-68)
```
Zero-edit alternative: bypass launch files and run the planner node directly with `--ros-args -p vehicle_id:=$i -p bbox_min_x:=… ` (compute bbox via `python3 -c "from mdn_core.geofence_logic import team_forward_box; print(team_forward_box('$YAML', $i, 35.0, 35.0, 0.0))"`). Runtime `ros2 param set` does NOT work — value cached at init (`simple_exploration_planner.py:307`). Enables ModeState/lanes (`:956-1008`) and lower-id tie-break (`:1042-1044`).

### 3.5 Verify B end-to-end
```bash
# per domain i:
ros2 topic hz  /d$i/lora/peer_pose      # ~15.8 Hz (3 peers x 5.26/cycle, .ino:617-618)
ros2 topic hz  /d$i/lora/peer_msg       # ~15.8 Hz (each peer claims every cycle; claim TX not gated on own pose [cf] coordination_node.py:268)
ros2 topic echo /swarm/peers --once     # 3 peers, age_s < ~0.4, delivery_ratio ~1.0, claim fields live
ros2 topic echo /d$i/swarm/link_stats --once   # lora_bridge_up: true
# bridge logs: 'opened LoRa serial: /tmp/hercules_lora/droneN'
python3 -m unittest discover /home/lucas/hercules-sim/src/multi_drone_nvblox/test   # claim codec sanity
```
Prerequisite gotcha `[cf+inf]`: own-pose TX requires `/fmu/out/vehicle_local_position` (`coordination_node.py:147-151,260`); claims TX regardless. A drone with dead PX4 VLP appears in peers' tables **at (0,0,0) with live claims** (claim-path `setdefault`, `coordination_logic.py:80,90`) — its keep-out radius then poisons frontiers near the origin. Confirm VLP is alive in all 4 domains before scoring runs.

---

## 4. BUG SIGNATURES TO WATCH

Baselines when healthy: `/swarm/peers` 5.26 Hz; per-peer `age_s` ≤ ~0.4 s; `delivery_ratio` ≥ ~0.95; peer expiry warn within 3 s of silence.

### 4.1 Stale-peer latch ("F2")
Note: "F2" is not an in-repo fault code (grep matched only "EKF2") — `[inf]` interpreted as stale peer/claim state latching past its lifetime. Two distinct latch surfaces:
- **Server side (PeerTable):** healthy = 3 s silence → `expire()` drops peer + its claims, log `peer N expired (>3.0s silent) - claims released` (`coordination_node.py:245-249`; `coordination_logic.py:112-121`). Latch signature = peer entry persists with `age_s` climbing unbounded on `/swarm/peers`, or `delivery_ratio` frozen while `age_s` grows. Test in B: SIGSTOP one bridge; expect the expiry warn in the other 3 domains within ~3.2 s and the entry gone.
- **Planner side (frozen snapshot) — the more dangerous latch `[cf]`:** `_peers_cb` overwrites `peer_claims/peer_positions` from the LAST message with **no staleness check** (`simple_exploration_planner.py:566-573`). If coordination_node dies (not merely empties), the planner keeps discounting/keeping-out against a dead snapshot forever. Signature: `ros2 topic hz /swarm/peers` → 0 while the planner still logs claim/keep-out rejections. Watch in BOTH runs (in A the snapshot is empty, so kill-tests must come from B).

### 4.2 Claim contention
- **Run A (as-flown):** no peers → no discounts, no tie-break, no lanes → all 4 drones plan independently; expect overlapping frontier coverage and colliding viewpoints. That overlap IS the field-faithful degradation measurement — record it, don't "fix" it.
- **Run B:** transient double-claim window is bounded by one TDMA cycle (190 ms) + planner FSM period (0.5 s). Signature of pathological contention: two domains' `claim set: (x, y) gain N` logs flapping between the same coordinates across consecutive plan cycles; `/swarm/peers` showing two peers with `claim_state≥1` at the same (x,y) for ≥2 s. With the §3.4 patch, the HIGHER id must yield (`loses_tie_break`, `simple_exploration_planner.py:1042-1044`); hysteresis prevents thrash — claim switches only after `claim_hold_s=4 s` unless released by reach ≤1 m / age 20 s / frontier-gone (`coordination_logic.py:204-248`; `coordination_node.py:223-256`). If you skipped the patch, tie-break is OFF and symmetric contention is expected — that invalidates B's design-faithfulness claim.
- Watch: `ros2 topic echo /coordination/claim_intent` per domain + coordination logs; cross-check both drones' `/swarm/peers` claim fields.

### 4.3 TDMA starvation
- Mechanism `[cf]`: coordination_node enqueues one claim per 0.19 s (`coordination_node.py:268-286`); firmware/emulator drains ONE custom per 190 ms cycle through a single-deep queue that silently drops the newcomer (`.ino:323-331`). Nominal rates match 1:1; timer drift makes occasional drops normal.
- Signature: receiver-side `delivery_ratio` (seq-gap based, `coordination_logic.py:64-66,91-99`) — expect ≥0.95; sustained <0.9 = starvation. Poses fresh but `claim_state/claim_gain` stale = customs starving while 0x81s flow. Emulator drop counter (§3.3) climbing steadily confirms it.
- Root causes to check, in order: emulator running N=3 timing (CYCLE_MS=150) or any cycle mismatch vs `cycle_period_s=0.19`; emulator slot-cap misconfigured with base ≠ 39 B (19 B claim must always fit — any size-drop of a 19 B custom is an emulator bug, FLAG-5); a second publisher flooding `lora/tx_msg`.
- Also watch bridge-side: `max_payload=160` truncation (`lora_bridge_node.py:353-355`) never triggers for claims; unknown frame types are silently ignored (`:327`) — a mis-built emulator frame vanishes without a log line, so verify §3.5 rates FIRST before interpreting higher-level silence.

---

## 5. RUN LOG DEVIATIONS TO RECORD
1. Name↔id mapping pinned to task brief, contradicts `fleet_ctl:29-35` (FLAG-2).
2. Run B uses `connectivity_mode:=wifi_only` + external bridge purely to inject `serial_port` (FLAG-4) — node graph is otherwise identical to `full`.
3. Planner `vehicle_id` patch applied in B only; A keeps the as-flown vehicle_id=0 gap (FLAG-3).
4. Emulator puts mesh id = vehicle_id−1 on the wire (matches firmware 0-based `NODE_ID`, `.ino:37-38`) and runs NUM_NODES=4 vs the compiled-for-3 firmware — deliberate, matches `cycle_period_s` default.
5. `wifi_up`/`map_ok`/`batt` broadcast as 0 always (never set in coordination_node, `coordination_node.py:104-105,270-277`) — `FLAG_LOW_BATT` can never assert; ignore those fields in analysis.
6. Emulator delivers instantly unless the delay knob is enabled; real slot airtime is 13–28 ms (`[inf]` from `lora_airtime_ms`).

Key files: `/home/lucas/hercules-sim/src/radiohive/scripts/lora_bridge_node.py`, `/home/lucas/hercules-sim/src/radiohive/firmware/lora_mesh_v7_2_ros2.ino`, `/home/lucas/hercules-sim/src/radiohive/launch/lora_bridge.launch.py`, `/home/lucas/hercules-sim/src/multi_drone_nvblox/scripts/coordination_node.py`, `/home/lucas/hercules-sim/src/multi_drone_nvblox/mdn_core/{lora_packet.py,coordination_logic.py}`, `/home/lucas/hercules-sim/src/multi_drone_nvblox/launch/{coordination_stage,fleet_bringup,planner_stage}.launch.py`, `/home/lucas/hercules-sim/src/multi_drone_nvblox/scripts/gen_zenoh_config.py`, `/home/lucas/hercules-sim/src/active_exploration/{scripts/simple_exploration_planner.py,launch/simple_planner_launch.py}`, `/home/lucas/hercules-sim/bin/zenoh-bridge-ros2dds`.