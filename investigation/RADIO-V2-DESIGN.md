All load-bearing citations verified against the sources (firmware serial state machine fw:268-310, custom ingest fw:323-331, sync fw:342-360, TX window fw:436-459, boot/airtime fw:382-406, bridge EOF blind spot br:215-222, full emulator). Producing the design doc as my return value.

# HERCULES Virtual LoRa Radio v2 — Design Doc

**Target file:** `/home/lucas/hercules-sim/virtual_lora_radio.py` (v1 = 183 lines, verified). Emulator plays the firmware+RF side of one pty per drone; the real `lora_bridge_node.py` opens the slave via symlink `/tmp/hercules_lora/droneN` (emu:56-64).

**Citation keys** (repo paths under `/home/lucas/hercules-sim/`):

| key | file |
|---|---|
| fw | src/radiohive/firmware/lora_mesh_v7_2_ros2.ino |
| br | src/radiohive/scripts/lora_bridge_node.py |
| pkt | src/multi_drone_nvblox/mdn_core/lora_packet.py |
| co | src/multi_drone_nvblox/scripts/coordination_node.py |
| cl | src/multi_drone_nvblox/mdn_core/coordination_logic.py |
| emu | virtual_lora_radio.py (v1) |
| FS | src/multirobot_launch_scripts/investigation/FINAL-SYNTHESIS.md |
| SB | src/multirobot_launch_scripts/investigation/STUCK-BUG-INVESTIGATION.md |
| AH | src/multirobot_launch_scripts/AGENT_HANDOFF.md |
| LB | src/multi_drone_nvblox/docs/LORA_BUDGET.md |

**Deployed-build note:** the committed firmware has `NODE_ID 0 / NUM_NODES 3` (fw:38,42, "CHANGE THIS FOR EACH NODE" fw:37); the fleet builds N=4 [inf — supported by fw's own v7.2 comment citing "~38 B" base (fw:531-533), `slot_margin` default `base_packet=38` (pkt:201,238), radiohive commit d09503e "Verified: 4-node mesh forms", CURRENT_STATE.md:50-53]. All v2 constants below are at N=4 unless noted. Hardware is an **SX1262** (Heltec Wireless Stick Lite V3, fw:2-3,34), not SX127x as the task brief said; at SF7/BW500 (no LDRO) the SX126x airtime equals pkt's SX127x formula — verified numerically equal for PL∈{37,38,39,57,58,64,65,198} against SX1262 datasheet §6.1.4 (https://cdn.sparkfun.com/assets/6/b/5/1/4/SX1262_datasheet.pdf).

---

## 1. Fidelity model summary — layer by layer

### L0 — Topology / transport (unchanged)
One pty pair per drone, master owned by emulator, slave symlinked at `/tmp/hercules_lora/droneN` (emu:56-64). Bridge opens it raw termios 8N1, no flow control (CRTSCTS cleared br:117-118 per research), `TCIOFLUSH` on every (re)open (br:125), reconnect on `OSError` only with 2.0 s backoff (br:207-229; param br:155). Wire node ids 0-based; claim payload src_id 1-based, opaque passthrough (co:196-204, emu:14-15).

### L1 — Serial link (CP2102 + USB) — NEW
| quantity | value | source |
|---|---|---|
| byte time @115200 8N1 | 86.8 µs/byte, 11,520 B/s per direction | fw:63, br:152 |
| frame wire times | POSE_UPDATE 20 B = 1.74 ms; CUSTOM claim 24 B = 2.08 ms; uplink burst 44 B = 3.82 ms; PEER_POSE 27 B = 2.34 ms; PEER_MSG 24 B = 2.08 ms | frame layout fw:11-17, br:10-12; claim = 19 B (pkt:249-250) |
| USB latency quantum | +U(0.1, 1.0) ms per direction per transfer (full-speed 1 ms frames; CP210x has no FTDI latency timer; measured RTT ~1-3 ms) | https://ftdichip.com/wp-content/uploads/2020/08/AN232B-04_DataLatencyFlow.pdf ; https://heavydeck.net/blog/measuring-serial-port-latency/ |
| device buffers | 576 B device→host, 640 B host→device (+256 B URB slack), **silent drop on overflow** (no flow control) | CP2102 datasheet https://www.silabs.com/documents/public/data-sheets/CP2102-9.pdf ; URB size https://lkml.iu.edu/hypermail/linux/kernel/1003.2/00951.html |
| end-to-end latency | bridge-write→fw-parse 2.5-5 ms; fw-write→bridge-publish 2-4 ms [inf on loop-poll term] | composition of above |
| steady-state load | uplink 44 B/190 ms, downlink ≤153 B/190 ms (3×27+3×24) ≈ 7% utilization — **congestion is impossible in steady state; do not invent it** | co:160,255-285; fw:209-262 |
| open-reset | bridge (re)open → DTR/RTS auto-reset of ESP32: 200-600 B boot-ROM spew 100-300 ms after open (survives the open-time flush), radio mute 1.5-3 s [inf — standard Heltec CP2102 wiring] | br:125; fw:2-8, NODE_TIMEOUT fw:58 |

v2 replaces v1's instant pty copy (emu:115-119, 70-76) with a per-direction token-bucket byte shaper + modeled 576/640 B buffers. This also removes v1's **uncontrolled artifact**: nonblocking `os.write` with ignored return/`OSError` (emu:60,117-119) silently truncates frames at the ~8-16 KB kernel pty buffer — wrong buffer size, unseeded, uncounted.

### L2 — Firmware serial protocol behavior — CORRECTED
- Framing `[0xAA][type][len][payload][XOR(type,len,payload)]` (fw:11-17,199-208; br:10-12). Emulator keeps this (emu:45-50).
- **Radio-side parser gets firmware semantics**, not bridge semantics: 5-state machine, no inter-byte timeout, **no rescan** — after 0xAA it blindly consumes type+len+payload+checksum; bad checksum silently discards the whole consumed span, eating any real 0xAA inside it (fw:268-310, verified). One corrupted len byte can swallow ~200 subsequent bytes including good frames. v1 wrongly uses the bridge's advance-1-byte rescan (emu:77-98) on the radio side — under-drops once corruption exists. XOR false-accept ≈ 1/256.
- `CUSTOM_MSG` accepted only if `len >= 2 && !custom_pending` (fw:324, verified) — dest-only frames ignored (v1 wrongly queues them, emu:103); ingest truncates to MAX_CUSTOM_DATA=160 (fw:327,79).
- `POSE_UPDATE` = last-write-wins register, `len>=16`, 4×f32 LE (fw:314-320). Matches emu:101-102.
- Single-deep keep-OLDEST custom queue; newer submissions silently dropped, no NACK, invisible to ROS2 (fw:323-331). Matches emu:103-107 — **preserve**.
- STATUS_RESP `{node_id u8, num_alive u8, [id u8, rssi i16, snr f32]×alive}` from the 3 s timeout table (fw:245-263), answered from loop() after wire+parse delay, not synchronously; polling defaults OFF (`status_poll_hz=0.0`, br:154).
- Emission order per received mesh packet: [PEER_JOINED 0x85 if reviving] → PEER_POSE 0x81 (always, even zero-pose) → PEER_MSG 0x82 iff `has_custom && custom_len>0 && (dest==NODE_ID || dest==0xFF)` (fw:609-625).

### L3 — Firmware TDMA / MAC — NEW (the core of v2)
**Constants** (fw:42-64): SLOT_MS=40, GUARD_MS=30, CYCLE_MS=N·40+30=**190 ms** @N=4, TX_MARGIN_MS=5, NODE_TIMEOUT_MS=3000, STATUS_MS=5000 (inert, fw:637-658 commented out), SERIAL_BUF_SIZE=220, MAX_CUSTOM_DATA=160.

**Packet**: packed LE struct = `node_id u8 | sequence u32 | uptime u32 | cycle_clock u32 | sync_source u8 | rssi_of[N] i8 | neighbor_mask u8 | 4×f32 pose | has_custom u8 | custom_dest u8 | custom_len u8 | custom_data[0..160]` (fw:84-101, verified). **BASE_PACKET_SIZE = 34+N = 38 B @N=4** (fw:106). v1's `BASE_PACKET=39` (emu:34) is off by one — live payload-cliff bug.

**TX rule** (fw:436-459, verified): transmit iff `slot==NODE_ID && !tx_done_this_cycle && time_in_slot >= 5 && time_in_slot + this_airtime <= 35` (integer ms), where `this_airtime` includes the pending custom's real size (fw:443-448). **No pose-only fallback**: a pending 19 B claim shrinks the launch window from [5,15] ms to [5,9] ms; missing it silences the node for the **entire cycle** (pose AND claim), custom persists to next cycle (cleared only inside doTx, fw:522). One TX/cycle via `tx_done_this_cycle` reset on cycle rollover (fw:430-434). `my_seq` increments on every TX attempt including lost ones (fw:490). v2 adds configurable loop-latency jitter (default U(0,2) ms + rare spikes) to exercise misses; v1 can never miss (emu:147, no upper bound).

**Custom attach** (fw:505-527): fits `max_tx_size` → attach whole; else **drop whole, never truncate**, silently; `custom_pending=false` either way. `max_tx_size` = largest size with `getTimeOnAir ≤ 30,000 µs` = **64 B → max custom 26 B** (fw:388-393, verified). Airtimes integer-truncated µs/1000 (fw:382-383, fallback 6 ms if 0).

**Sync protocol** (no beacon, no join handshake): boot free-running, `cycle_epoch=millis()`, `synced=true`, `my_sync_source=NODE_ID`, self alive (fw:395-406, verified). Per received packet: `auth = min(sender_id, sender_sync)`; accept iff `auth < my_sync_source` OR (`auth == my_sync_source && sender_id <= my_sync_source`) (fw:355-360, verified) — net effect: epoch moves only on packets heard **directly** from the authority [inf from fw:355-360 + fw:571-590]. Epoch snap:

```
new_epoch = (now − packet_airtime_ms) − (sender_id·SLOT_MS + TX_MARGIN_MS) − (p->cycle_clock / CYCLE_MS)·CYCLE_MS
```
(fw:571-590; integer division). Post-snap: `|correction| > SLOT_MS/2 (20 ms)` → `tx_done=false` (legal double-TX, fw:577-581); new elapsed already past `(NODE_ID+1)·40` → `tx_done=true`, own slot skipped silently (fw:586-589). `findNewSyncSource` = min over alive peers' **ids and gossiped sync_sources** (fw:342-353, verified).

**Modeled sync quirks** (all verbatim-reproduced):
1. **Fixed-airtime epoch error**: resync uses base `packet_airtime_ms`=20 ms regardless of actual length (fw:573) while TX used the real airtime (fw:443-448) → every claim-carrying authority packet (57 B = 26.944 ms) injects **≈ +6.9 ms** epoch error into every follower, corrected on the next pose-only packet — a claim-traffic-synchronized sawtooth [magnitude inf; arithmetic verified].
2. **Sender-launch-jitter error**: receiver assumes launch at exactly +5 ms into slot (fw:572); real launch lands anywhere in the window → up to ~10 ms extra error, plus RX-flag poll latency (fw:421-426) [inf].
3. **Node-0 reboot**: node 0 never syncs to anyone (`shouldSyncTo` can't pass for sender_id>0 when my_sync_source==0); the whole mesh re-phases to its new arbitrary epoch one node at a time, colliding meanwhile [inf from fw:342-360,402-404]. **Hidden-node sync lock**: a node locked to source 0 that loses RF only to node 0 re-elects... 0 (gossiped values still 0, fw:342-353) and free-runs indefinitely while believing it's synced [inf on consequence].

**Peer table** (fw:111-121): death = alive && last_seen>0 && age>3000 ms (wrap guard `age<0x80000000`, fw:474) → alive=false + serial PEER_LOST 0x84 + findNewSyncSource if it was my source (fw:469-480, verified); birth = any valid RX from dead peer → PEER_JOINED 0x85 before that packet's PEER_POSE (fw:609-615). Never-heard peers never emit LOST. missed += seq gap−1 (fw:593-594), never exported.

**RX validation** (fw:544-564): `38 ≤ len ≤ sizeof(Packet)`; `node_id < NUM_NODES && != self` (own-id silently dropped); exact-length check `len == BASE (+custom_len)` → **mixed-NUM_NODES builds reject 100% of each other's pose-only packets** (37≠38) — one-way-invisible node.

**Always-beacon**: radios TX every cycle from boot with zero-init pose registers (fw:131,486-498) — peers get (0,0,0,0) PEER_POSEs before the first POSE_UPDATE, and a dead coordination_node's radio keeps beaconing the last register (zombie beacon, prevents 3 s expiry). v1 wrongly stays silent until first pose (emu:151) and can deliver claim-without-any-pose (emu:160 vs 151) — impossible on hardware.

### L4 — PHY: airtime / path loss / PER / collisions / drift — NEW
**Airtime** (inherited: keep importing `pkt.lora_airtime_ms`, pkt:173-198, Semtech AN1200.13 form ≡ SX1262 DS §6.1.4, verified): T_sym=2⁷/500 kHz=0.256 ms; preamble 12.25 sym=3.136 ms. Table @SF7/BW500/CR4/5/pre8/CRC/explicit: **37/38/39 B = 20.544 ms** (same symbol block — the 39-vs-38 error is airtime-neutral for pose packets); **57 B = 26.944 ms** (matches pkt:242-245); **64 B = 29.504 ms**; 198 B = 79.424 ms (the v7.1 bug, fw:532-533).

**Atomic on-air unit**: pose+claim = ONE packet; single per-receiver loss draw; both delivered (0x81 then 0x82) at `tx_start + airtime` + serial pacing, or neither. Replaces v1's two independent rolls (emu:156,171).

**Half-duplex**: transmitting node deaf for its full airtime (`radio.transmit` blocking, `in_rx=false`, fw:454-457,535 + fw:171-176); ~0.5-2 ms re-arm gap after each RX; a frame is missed unless the receiver was armed within the first 3 preamble symbols (0.768 ms) — needs the last 5 of 8 symbols (Bor et al. Fig. 3, https://www.link-labs.com/hubfs/DOCS.linklabs.com/2017/01/lora-scalability_r254.pdf; SX1262 DS §6.2.2.1).

**Collision/capture per receiver** (A stronger than B by ΔP at that receiver): no overlap → both OK; ΔP ≥ **6 dB** and A starts ≤ B_start + 3·T_sym (0.768 ms) → A delivered, B lost; ΔP ≥ 6 dB but A later (receiver locked to B) → both lost; ΔP < 6 dB → both lost. 6 dB = LoRaSim ecosystem constant [inf — Bor's Pthreshold; same-chip DS CCR SF7 = 5 dB]. Lost frames vanish silently (PHY CRC, fw:378) → seq gap only (fw:593-594).

**Channel model** (per link, geometry-fed from the POSE_UPDATEs already parsed, emu:101-102):
```
RSSI(d) = 10 dBm − [31.7 + 10·n·log10(d/1m)] − X_σ − fade ;  n = 2.9 (2.7–3.0 near-ground open field)
X_σ ~ N(0, 3 dB) slow per-link process (coherence ~seconds);  fade ~ Rician K = 8 dB per packet
SNR = RSSI + 109.5 dB   (noise floor: −174 + 10·log10(500e3) + NF 7.5 = −109.5 dBm → sensitivity −117 dBm @SF7/BW500, matches DS table 3-5)
PER(SNR) = 1 / (1 + exp(2.2·(SNR + 7.5)))   (SF7 demod limit −7.5 dB, DS table 6-1; k≈2.2/dB from PER waterfall lit.)
```
Sources: SX1262 DS; https://rsisinternational.org/journals/ijriss/articles/experimental-evaluation-of-antenna-height-impact-on-lorawan-performance-in-the-915-mhz-ism-band/ ; https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9101881/ ; https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6111734/ ; https://www.researchgate.net/publication/362115325_On_the_Performances_of_Packet_Error_Rate_for_LoRa_Networks ; https://arxiv.org/pdf/1905.11252. Two-ray crossover d_c = 4πh₁h₂/λ = 38.4 m @1 m AGL, λ=0.3276 m (optional null dips beyond). Reported values quantized as hardware does: RSSI→i16, SNR f32 capped ~+10 dB (SnrPkt/4). **Headline conclusion the model must preserve: at 10-100 m the margin is 25-60 dB above the PER cliff — range loss is essentially never the field loss mechanism; loss must come from desync/collisions/half-duplex/queues/serial** [inf from budget].

**Burst loss overlay**: Gilbert-Elliott two-state per link (replaces uniform i.i.d. as default; `LORA_DROP_PCT` kept as debug knob). Motivation: peer expiry needs 15.8 consecutive lost cycles (3.0 s / 190 ms, cl:72, co:84); P ≈ p^15.8 ≈ 1e-11 at p=0.2 — uniform drop can never produce the field's expiry events.

**Clocks**: per-node ε_i ~ U(−10,+10) ppm fixed at boot (ESP32 40 MHz spec, https://www.esp32.com/viewtopic.php?t=2464), optional ±2 ppm slow walk; **1 ms millis() floor on every slot/sync/cycle_clock read** (all fw timing is millis(), fw:412-413,178-186). Drift 5-20 µs/s relative → eats the 5 ms margin in 250-1000 s unheard; adjacent-slot on-air overlap in 655-2620 s. The SX1262 TCXO (1.6 V, fw:372) stabilizes only the RF carrier, not slot timing.

---

## 2. Failure-injection API

Two surfaces: **boot env** (existing style) + **runtime control** (UDP text datagrams on `127.0.0.1:$LORA_CTRL_PORT`, default 47850, plus optional time-tagged schedule file `LORA_SCHEDULE`, lines `t=<s> <verb> ...`). Every applied event logged with node-clock time + counter snapshot; vocabulary is a superset of `mdn_core/fault_logic.py` (`kill/revive/partition/clear`) so behavior-level and phy-level injection can be A/B'd (commit e593b3e; its doc says physical-layer faults "remain manual bench options" — that layer is exactly this API).

**Boot env**: `LORA_N` (=4), `LORA_SEED`, `LORA_DROP_PCT` (debug), `LORA_MODE=faithful|ideal` (ideal = v1 behavior, regression escape), `LORA_PPM_MAX=10`, `LORA_LOOP_JITTER_MS=2`, `LORA_GE='p_gb,p_bg,per_good,per_bad'`, `LORA_CHANNEL=geom|fixed`, `LORA_PATHLOSS_N=2.9`, `LORA_SHADOW_SIGMA=3`, `LORA_RICIAN_K=8`, `LORA_NODE_CFG='mesh:NODE_ID:NUM_NODES,...'` (misconfig staging), `LORA_SERIAL_PACE=1`, `LORA_USB_JITTER=1`, `LORA_CTRL_PORT`, `LORA_SCHEDULE`.

**Runtime verbs → field-failure catalog**:

| verb | mechanism | field failure reproduced | source |
|---|---|---|---|
| `kill <id>` / `revive <id>` | radio dead / reboot (revive = fresh arbitrary epoch, seq=0, sync_source=self, collides until it hears authority) | sync-source (node 0) death → fleet-wide re-mastering + correlated loss burst; peer expiry → claims released | fw:342-360,402-404,469-480 |
| `reset <id>` | DTR-style reboot: boot spew 100-300 ms post-open + 1.5-3 s radio mute | bridge restart/reconnect cascades into mesh churn (PEER_LOST at 3 s → claim release → frontier churn) | [inf] Heltec wiring; br:125; fw:58 |
| `wedge <id> [rx\|tx\|both]` | pty stays open, forwarding silently stops (half-wedge = one direction) | **F2 prime stuck mechanism**: LoRa wedges → frozen exclusion discs → endless NO PATH; TX-only wedge = listener drone, asymmetric peer tables | FS:36 (F2); co:348-349 |
| `unplug <id> [T]` | close pty pair + remove symlink → bridge gets OSError(EIO), reconnect loop + TCIOFLUSH; recreate after T | mid-flight USB vibration/brownout; exercises 2 s backoff + flush-losses | br:207-229,125 |
| `eof <id>` | close master only → bridge `read()==b''` forever | **demonstrable bridge bug**: EOF is dead code (`pass`) → 100% CPU busy-spin, never reconnects, node looks alive while dead | br:215-222 (verified dead code) |
| `stall <id> [T]` | emulator stops reading its pty → bridge's unchecked nonblocking write short-writes / raises → silent uplink blackhole while downlink continues | firmware-wedge phenotype: drone hears everyone, transmits nothing; peers expire it | br:137-138,344-347 |
| `steal <id> <frac>` | second reader consumes a fraction of RX bytes / garbles baud | risk R1 non-exclusive open (ModemManager/XRCE-agent); no TIOCEXCL anywhere | SB:13-19; AH:252,441-442 |
| `roulette` | swap two drones' symlinks | 2026-08-03 delta enumeration roulette (XRCE opened the LoRa modem, silent) | FS:54-60,161; commit 131f194 |
| `corrupt <up\|down> <rate>` | bit flips; radio side uses the **no-rescan** parser (one bad len byte eats up to 255 B incl. good frames); 1/256 XOR false-accepts pass garbage | silent claim/pose loss with clean logs; wild-pose delivery into coordination | fw:268-310; br:240-268; co:206-210 |
| `garbage <id>` | ASCII/binary spew burst | boot-ROM spew / printStatus archetype; chance-0xAA parser stall | br:240-268; fw:631-659 |
| `mute_link <a> <b> [oneway]` | per-link blackhole, asymmetric allowed | **hidden-node sync lock**: node keeps inaudible source 0, free-runs, drifts into others' slots | fw:342-353,355-360 [inf] |
| `partition 'a b\|c d'` | pairwise blackhole, fault_logic grammar | A/B parity with behavior-level injection | mdn_core/fault_logic.py:1-120 |
| `freeze_sync <id>` / `jump <id> <ms>` / `drift <id> <ppm>` | stop epoch corrections / forced epoch step / set skew | scripted desync, drift-into-collision experiments | fw:571-590 |
| `claim_every_k <id> <k>` / `dup_claim <id>` | drain pending custom every k-th cycle / duplicate delivery | silent claim starvation under healthy pose flow; u8 seq-gap accounting | fw:323-331; cl:91-99; co:268-286 |
| `misconfig <id> node_id=<n> num_nodes=<m>` | per-node build constants; exact-length RX check does the rest | stale-firmware/mis-flash: one-way-invisible node, duplicate-ID slot collision | fw:37-42,544-564; d09503e |
| `interferer <duty%> <dBm>` | Poisson same-channel bursts | external 915 MHz ISM users [inf, optional] | — |
| `drop <pct> [a b]` | per-link uniform override | debug only | emu:36 legacy |
| `clear` | all faults off | — | fault_logic parity |

---

## 3. Prioritized implementation plan

Single file stays single-file (standalone sim tool). v1 = 183 lines; v2 estimate **≈ 1,150-1,350 lines**. Order chosen so every later item lands on the per-node-clock substrate.

**P0 — critical (unlocks everything; ~430 lines added/changed)**
1. **Event core + per-node clocks** (~120 ln): wall-anchored `NodeClock` (boot epoch, ppm skew, 1 ms millis() floor), heap of scheduled deliveries, poll tightened to ~0.5 ms. Keyed RNG `hash(seed, purpose, sender, rx, cycle)` so draws are order-independent → real reproducibility (v1's seed is wall-clock-coupled and not reproducible, emu:37 + audit).
2. **Verbatim v7.2 sync protocol** (~90 ln): shouldSyncTo/findNewSyncSource/epoch-snap equation, quirks 1-3, tx_done reset/skip rules (fw:342-360,571-590). Enables the entire desync failure class — impossible in v1's shared clock (emu:127,136).
3. **Atomic on-air packet + always-beacon** (~70 ln): one AirFrame carrying pose+seq+sync fields+optional custom; single loss draw; zero-pose beaconing from boot; JOINED→POSE→MSG emission order. Fixes both WRONGs (independent rolls emu:156/171; silent-until-pose emu:151).
4. **Runtime fault API + schedule engine** (~130 ln): UDP listener, schedule file, verb dispatch, event log. Unlocks the whole field-failures area (v1 has zero runtime surface).
5. **Constant fixes** (~10 ln): `BASE_PACKET=38` (34+N), integer size-based `max_tx_size=64` gate (max custom 26 B not 25), integer-ms airtimes for window math, `CUSTOM_MSG len>=2` rule, docstring corrections (emu:16-18,34,103,162 vs fw:84-106,324,382-393).

**P1 — high (~370 lines)**
6. **TX window + whole-cycle miss** (~40 ln): [5, 35−airtime] with pending-custom airtime, no fallback, custom persists, loop-jitter knob (fw:436-459 vs emu:147).
7. **Airtime-delayed delivery + serial pacing both directions** (~90 ln): deliver at tx_start+airtime, then 86.8 µs/byte + U(0.1,1) ms USB jitter; removes the ~25-35 ms optimistic skew; lets the bridge's back-to-back pose/claim frames straddle the slot boundary.
8. **Peer tables + 0x84/0x85 + real STATUS_RESP + seq/missed accounting** (~80 ln): 3 s timeout, wrap guard, JOIN-before-POSE, table-driven STATUS_RESP with modeled rssi/snr (fw:469-480,609-615,245-263; v1 fakes all of it, emu:108-113).
9. **Collision + capture + half-duplex** (~70 ln): overlap detection on scheduled AirFrames, 6 dB/3-symbol rules, TX deafness + re-arm gap + preamble-arming.
10. **Channel model + GE burst loss** (~90 ln): geometry RSSI/SNR/PER per L4, slow shadowing state, per-link GE overlay; replaces constant −40/8.0 (emu:112,154).

**P2 — medium (~200 lines)**
11. **Modeled CP2102 buffers** (~40 ln): 576/640 B caps with silent mid-frame overflow; replaces the unchecked-short-write artifact (emu:117-119).
12. **Firmware-parser semantics + corruption hooks** (~60 ln): 5-state no-rescan machine on radio side (fw:268-310), bit-flip/garbage/truncation injectors, 1/256 false-accept path, byte loss during blocking TX [inf].
13. **Reconnect-reset emulation** (~50 ln): detect slave close/reopen via EIO transitions on the master [inf — Linux pty semantics], then boot-spew + 1.5-3 s mute.
14. **Extended counters** (~50 ln): per-link delivered/lost/collided, window-miss skips, GE state occupancy, queue/air drops, per-node seq — reconcilable against cl:91-99 delivery accounting.

**P3 — low (~60 lines)**
15. **Misconfig knobs** (~30 ln): per-node NODE_ID/NUM_NODES, exact-length RX validation, duplicate-ID staging (fw:544-564).
16. **lora_only mode validation + docs** (~20 ln): 32 B legacy full packet passes the corrected gate (70 B = 32.064 ms ≤ 30 ms budget FAILS policy but fits slot per LB:37-41 — document precisely); claim-latency-invisibility note (no t_ms in claim pkt:248-293; `PeerEntry.latency_ms` never written, cl:62).
17. **Two-ray/interferer options** (~10 ln): off by default.

---

## 4. Validation plan

1. **Airtime/codec inheritance**: keep importing `pkt.lora_airtime_ms` (emu:28-29); run their `test_lora_packet.py` unchanged (13 B @SF7/BW125 = 46.336 ms vector, ≥30% slot-margin assertion, pkt:173-232). Add emulator unit tests: golden airtimes {38→20.544, 57→26.944, 64→29.504, 198→79.424 ms}; gate accepts 26 B custom, drops 27 B (the v1 off-by-one regression test).
2. **Frame-level golden tests**: canned POSE_UPDATE/CUSTOM_MSG/STATUS_REQ in → byte-exact 0x81/0x82/0x83/0x84/0x85 out, parsed with the real bridge's parser code (br:240-268, format cross-check br:291-307); emission-order assertion (JOINED before POSE); `len>=2` custom rule; dest filter dest==id||0xFF, no self-delivery.
3. **Cadence checks**: per-peer PEER_POSE inter-arrival at each pty = 190 ms ± modeled jitter; aggregate per bridge ≈ 3 × 5.26 Hz = 15.8 Hz, matching the bench-verified ~16 Hz 4-node figure (radiohive commit d09503e). TX launch offsets within [5,15] ms (pose) / [5,9] ms (claim-laden) of slot start; zero TX in guard (elapsed ≥ 160 ms).
4. **Latency distribution**: timestamp POSE_UPDATE-write → peer PEER_POSE-read; assert p50/p95 against the analytic chain (slot-wait U(0,190) + launch 5-15 + airtime 20.5-26.9 + serial 2.3-4.4 + USB jitter); end-to-end with coordination ticks ≈ 320/610 ms typical/worst. Harness must timestamp at the pty and at `swarm/peers` — **no in-stack metric exists** (claim carries no timestamp pkt:248-293; latency_ms never written cl:62).
5. **Desync experiment (flagship)**: seed ±10 ppm; `kill 0` at t=60 s → assert followers coast then PEER_LOST(0) on all bridges at +3.0 s; re-mastering to node 1 (findNewSyncSource); `revive 0` at t=90 s → collision-storm window while old/new phases overlap, one-node-at-a-time re-lock (direct-hear rule); signature = correlated multi-link seq-gap burst + LOST/JOINED storm + recovery. Also: hidden-node lock via `mute_link 0 <x> oneway` → x free-runs, drifts, collision bursts against the slot it drifts into. Determinism: two runs, same seed+schedule → identical counters and event logs.
6. **Peer-expiry reachability**: 30 min at uniform 20% drop → **zero** expiries (p^15.8 ≈ 1e-11, cl:72); GE bursts (mean bad-state ≥3 s) → expiries occur. Proves the burst model is what unlocks the field class.
7. **Window-miss / claim-coupling**: claims every cycle + loop jitter → measured whole-cycle blackout rate matches P(launch lands past the [5,9] ms window); peers see 380 ms pose gaps; quirk-1 sawtooth (~+6.9 ms follower epoch error on authority claim cycles) visible in launch-offset traces.
8. **A/B vs behavior-level faults**: same scenario via `fault_logic` topic vs via phy verbs → compare `swarm/peers` signatures (partition, kill/revive).
9. **Regression**: `LORA_MODE=ideal` reproduces v1 behavior for existing sim runs; `eof` verb demonstrates the bridge busy-spin bug (br:215-222) as a documented repro, not a regression.

---

## 5. Deliberately NOT modeled — and why it cannot matter

| omission | why it can't matter for the fleet's bugs |
|---|---|
| RF carrier frequency offset / CFO | SX1262 TCXO powered at 1.6 V (fw:372) → ±2 ppm class carrier; negligible vs BW500 tolerance. Not a loss mechanism here. |
| Inter-SF / adjacent-channel interference | Single fixed channel 915.0 MHz, single SF7 fleet-wide (fw:48-54); no other SFs exist in the mesh. |
| LoRa PHY CRC false-accept | 2⁻¹⁶ ≈ 1.5e-5 per corrupted frame — dwarfed by the serial XOR's 1/256 path, which we DO model (fw:378). |
| FCC duty-cycle limits | 915 MHz ISM has no duty-cycle constraint relevant at 20-27 ms per 190 ms (fw airtimes). |
| Serial congestion beyond wire pacing | Link runs at ~7% of 11,520 B/s (44 B up + ≤153 B down per 190 ms, co:255-285, fw:209-262); steady-state congestion is physically impossible — modeling it would invent sim-only failures, violating the no-false-modes requirement. |
| Range-driven PER as a loss driver | 25-60 dB margin above the −117 dBm cliff across the 10-100 m deployment envelope [inf from link budget]; the PER curve is implemented but documented as inert at deployment ranges — field loss must come from timing/collision/queue/serial mechanisms. |
| RSSI-driven coordination behavior | Nothing in the coordination path consumes rssi/snr (co:196-214; fault gating is scripted, not RSSI-driven) — channel-model realism is for tooling parity and future thresholds only. |
| ESP32 internals (task scheduling, WiFi coexist) | Observable effect is TX-launch jitter and stale `now` in loop() (fw:412-424) — fully subsumed by the loop-jitter knob. |
| Firmware 220 B buffer overrun's memory corruption | Real effect of len 221-255 is writes past `ser_buf` into parser globals (fw:64,291-297) — unpredictable board-specific corruption; we model the frame-eating/parser-scramble consequence, not fabricated memory contents. |
| Antenna patterns / polarization / Doppler | Folded into net-0 dBi assumption + 3 dB shadowing + Rician fade; drone speeds are trivial vs symbol rate at BW500. |
| External interferers by default | Unmodelable without site data [inf]; available as the optional `interferer` knob only. |
| Preamble-detector config variants | Fixed 8-symbol preamble at SF7 (fw:54); DS long-preamble advice applies to SF5/6 only (SX1262 DS §6.2.2.1). |
| GNSS semantics of lat/lon fields | Repurposed as team-frame x/y/z/yaw floats (LB:21-41); pure payload passthrough. |

**Bottom line:** v2 models every mechanism the field symptom can ride on — per-node clocks + verbatim (buggy) sync, atomic piggybacked packets, real TX windows, half-duplex/collisions, burst loss, timeout-driven peer events, paced/corruptible serial with real buffer sizes, and a scriptable fault API in the team's own vocabulary — and explicitly refuses to model anything that could generate sim-only failure modes.