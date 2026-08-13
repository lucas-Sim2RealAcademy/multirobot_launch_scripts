# HERCULES Fleet Coordination Redesign

**Status:** implementation spec. Every repo claim below was re-verified against source and against the `radio4` artifacts; numbers I reproduced myself are marked ✓. Estimates are marked [inf].

---

## 0. Scope correction (read first — it changes what to build)

Zhou et al. 2022 (*Sci. Robot.* 7(66):eabm5954) contains **no goal, frontier, or task-assignment mechanism**. Its coordination layer is reciprocal *collision avoidance* only; goals are exogenous ("A user or software gives a global goal position", p.13) and the Discussion explicitly defers assignment to an outside module (p.12). The claim/assignment layer — the one producing our 26% — is precisely the layer that paper disclaims.

Two consequences:

1. **The transferable idea is real but narrower than the framing implies.** "Two agents may use the same space at different times" resolves *collision* conflicts. It does not resolve *coverage* conflicts: ground swept twice is wasted whether the sweeps are 2 s or 200 s apart. So spatial-temporal sharing cannot by itself reduce coverage overlap. What it does is **remove false exclusions** — the hard 8 m disc that suppresses candidates a peer will have vacated — which gives the assignment layer headroom it does not currently have.
2. **The assignment source is RACER** (Zhou, Xu, Shen et al., IEEE T-RO 39(3):1816-1835, 2023; arXiv:2209.08533), same lab, same lineage. Its structural lesson: *ownership decides WHERE, the avoidance penalty decides HOW to get there.* HERCULES conflates them — `r_claim=8 m` avoidance geometry is currently making assignment decisions (`/home/lucas/hercules-sim/src/active_exploration/scripts/simple_exploration_planner.py:1012-1049`). Separating them is the real structural change.

Frame the work as **"EGO-Swarm's spatial-temporal deconfliction + RACER's region ownership"**, not "EGO-Swarm alone."

---

## 1. Diagnosis

In run `radio4` the fleet covered 2245 unique 1 m cells out of 3030 cells of effort (25.9% overlap ✓, reproduced from `radio4_*/meta_*.json` with the scorecard's own 4 m-disc union); 33 claims were set and 21 released, 17 by `max_age` and 4 by `reached`, 0 by `frontier_gone` ✓ — the signature that motivated this work — but the release timer (`claim_max_age_s=20.0`, `/home/lucas/hercules-sim/src/multi_drone_nvblox/scripts/coordination_node.py:86`) is only the visible end of a four-part failure, and the radio is not part of it at all: the virtual LoRa link delivered 11335 of 11370 packets (99.69%), executed 3790 of 3790 TDMA transmit opportunities with **zero** window misses, dropped exactly one custom payload, and expired **zero** peers in 180 s (`radio_radio4.log` t=180.006 ✓). What actually broke is (a) **the claim channel was silent ~40% of the time and stale the rest** — `claim_hold_s=4.0` silently rejects a new claim with no signal back to the planner (`mdn_core/coordination_logic.py:204-221`), so ghost turned 50 `PLAN OK` events into 9 claim sets, and after every release there is a median **10.9 s** gap before the next claim lands ✓ (n=20, planner cadence, not claim logic); (b) **claims are broadcast in the wrong frame** — own pose is transformed through `T_world_self` (`coordination_node.py:180-194`) but the claim is packed raw from `msg.viewpoint` (`:234-236` → `coordination_logic.py:256-264`), and `config/swarm_alignment.yaml` carries priors for only 2 of 4 vehicles under an id map (`1=delta, 3=ghost`) that contradicts `run_fleet_radio.sh:40,121` (`ghost=1, delta=2, buckshee=3, thunderstrike=4`) ✓, so peer coordinates are displaced 3-9 m against radii of 3/5/8 m — the exclusion field is mis-aimed, which is worse than absent; (c) **the fleet barely flew** — 4 of 91 `PLAN OK` reached `Path complete` (4.4%), 92 of 106 `NO PATH` events were "no clusters", median planned goal distance was **6.3 m over 2 waypoints** ✓, i.e. `r_claim=8 m` exceeds the median distance to the thing being claimed; and (d) **the metric cannot resolve any of this** — all four ground-truth tracks begin at (0,0) in their own frames, one drone (buckshee) ran to (150.2, 88.0) m inside a ±10 m geofence and alone contributes 1705 of 3030 effort cells (56%) of entirely uncontested transit, four stationary drones at the configured spawns already score **39.8% overlap** ✓, and at the in-arena effort actually flown (751 cells into a 630-cell arena) the **minimum arithmetically achievable overlap is 16.1% — the <10% target is impossible** ✓ until either the arena grows or redundant effort falls.

### Verification table

| Claim | Source | Result |
|---|---|---|
| 17 `max_age` / 4 `reached` / 0 `frontier_gone`, 33 sets | `coord_radio4_*.log` | ✓ exact |
| unique 2245 / effort 3030 / overlap 25.9% | recomputed from `meta_*.json` | ✓ (brief's 2807 is a different snapshot) |
| radio 99.69% delivery, `miss=0`, `qdrop=1`, `LOST/JOIN=0/12` | `radio_radio4.log` | ✓ exact |
| 91 `PLAN OK`, 4 `Path complete`, 106 `NO PATH` (92 no-clusters) | `planner_radio4_*.log` | ✓ |
| median goal dist 6.3 m, median 2 wps, max 5 | `planner_radio4_*.log` | ✓ |
| replan cadence: ghost/thunderstrike 0.5 s, delta 10 s, buckshee 31 s | `planner_radio4_*.log` | ✓ |
| 19 B → 26.944 ms → 4 ms TX window; 26 B → 29.504 ms → **1 ms** | their `lora_airtime_ms()` | ✓ (brief's "26 B = 4 ms" is off by one block) |
| per-drone claim-active fraction 46-72% | reconstructed claim timelines | ✓ |
| `peer_map_integrator` `ok=0` for every peer, whole run | `*radio4*.log` | ✓ shared-map channel delivered nothing |
| `BOUSTROPHEDON` never engaged | 0 hits in 4 planner logs | ✓ do not tune it |
| arena union 630 cells; <10% needs 676 unique at effort 751 | computed | ✓ impossible |

**Sequencing consequence:** Stage A is not "warm-up." Until the frames agree and the metric is honest, **no A/B test of Stage B or C is interpretable** — you would be measuring a mis-registered exclusion field through a metric dominated by one drone's odometry.

---

## 2. STAGE A — minimum-change fixes (zero wire change)

Everything in Stage A leaves the 19 B packet, its airtime, and `test_lora_packet.py` untouched. Roughly 120 lines plus config.

### A1. Tie claim lifetime to plan lifetime and measured progress, not wall clock

This is the requested "ETA/progress instead of fixed 20 s," but the measured data says the fixed timer is the *third* problem, behind hysteresis rejection and the dead `reached` path. Fix all three together or the change does nothing.

**A1a — honor `reached` directly.** `coordination_node.py:223-232` currently re-derives arrival from a distance test whose two operands are in different frames (`coordination_logic.py:238`, `reach_radius_m=1.0` vs a 3-9 m frame offset).

```python
# coordination_node.py  _on_intent(), replaces :224-232
if not msg.valid:
    if msg.reached:
        self.claim.release()                 # planner is authoritative on arrival
        self.get_logger().info('claim released: reached')
    elif not msg.frontier_exists:
        self.claim.release()
        self.get_logger().info('claim released: frontier_gone')
    return
```

**A1b — a new plan updates the claim; it is never silently rejected.** `coordination_logic.py:204-221` returns `False` with no signal to the planner. Replace the time hysteresis with a *displacement* hysteresis, which is what the anti-oscillation intent actually was:

```python
# coordination_logic.py  ClaimLifecycle.request(), replaces :204-221
def request(self, x, y, z, gain, now_s, path_len_m=None, eta_s=None):
    moved = _hypot2(x, y, self.x, self.y)
    if self.state == CLAIM_NONE or moved >= self.claim_move_eps_m:   # new param, 0.5
        self._set(x, y, z, gain, now_s, path_len_m, eta_s)
        return True
    self.gain = gain                       # same goal, refresh utility + deadline
    self._refresh(now_s, path_len_m, eta_s)
    return False
```

Retire `claim_hold_s`; add `claim_move_eps_m` (default 0.5, matching `waypoint_spacing`). Every `PLAN OK` now reaches the wire — ghost's 50 plans instead of 9.

**A1c — deadline from ETA, watchdog from progress.**

```python
# coordination_logic.py  ClaimLifecycle.update(), replaces :232-248
def update(self, own_x, own_y, frontier_still_exists, now_s, remaining_m=None):
    if self.state == CLAIM_NONE:
        return None
    if not frontier_still_exists:
        self.release(); return 'frontier_gone'
    # progress watchdog (WiSER-X: commit on path-length progress, not clock)
    if remaining_m is not None:
        if remaining_m < self.best_remaining_m - self.progress_eps_m:   # 0.5 m
            self.best_remaining_m = remaining_m
            self.t_progress_s = now_s
        elif now_s - self.t_progress_s > self.stall_timeout_s:          # 6.0 s
            self.release(); return 'stalled'
    # ETA deadline: generous multiple of the planner's own estimate
    if self.deadline_s is not None and now_s > self.deadline_s:
        self.release(); return 'eta_expired'
    if now_s - self.t_claimed_s > self.claim_max_age_s:                 # raise to 60.0
        self.release(); return 'max_age'                                # backstop only
    return None
```

`deadline_s = t_claim + eta_margin * eta_s + eta_floor_s` (defaults 2.5 and 4.0). `reached` (A1a) is now the normal exit; `max_age` becomes a pure liveness backstop and should drop below ~10% of releases.

**A1d — get a speed estimate (this is the missing input, and it is free).** There is no velocity anywhere in the stack: `_publish_sp` NaNs velocity/accel/jerk (`planner:874-883`) and `_vlp_cb` receives `VehicleLocalPosition` and **drops `msg.vx/vy/vz`** (`planner:585-591` ✓). Measured ground speeds spanned 0.21-0.93 m/s — a 4x spread — so a nominal-speed constant would be wrong by up to 4x. Read the fields that are already arriving:

```python
# planner _vlp_cb(), after :589
self.ned_vx, self.ned_vy = msg.vx, msg.vy
v = math.hypot(msg.vx, msg.vy)
self.v_est = 0.9 * self.v_est + 0.1 * v if self.v_est > 0 else max(v, 0.15)
```

**A1e — publish path length and ETA.** `_resample_path` already accumulates `seg_len` into `accum` (`planner:1199-1229`) and discards it. Return the total; add `path_length` and `eta_s` to `msg/ClaimIntent.msg` (additive fields, existing ones untouched); populate at `planner:1160-1168`; republish a progress intent from the waypoint-advance block at `planner:717-724` so `remaining_m` reaches `ClaimLifecycle.update`.

### A2. Fix the frames (prerequisite; zero bytes)

- `config/swarm_alignment.yaml`: entries for **all four** vehicles with the real spawn offsets from `settings-fleet-4drone.json` (ghost −7,−7 / delta −7,−4 / buckshee −7,−1 / thunderstrike −7,+2), and correct the id↔name comment block — today vehicle 3 (buckshee) is flying a prior written for ghost, at the wrong magnitude (3.5 m vs a true 6 m separation) ✓.
- `coordination_node.py:234-236`: transform the incoming claim through `self.T_world_self` with `tr.se3_apply`, exactly as own pose already is at `:185-186`.
- `coordination_node.py:260-267`: when `T_world_self is None`, **do not transmit** pose or claim (or set a `FLAG_UNALIGNED` bit and have receivers ignore that peer's geometry). Broadcasting raw odom labelled "team frame" is worse than silence.
- Symmetry matters: the receiving planner never transforms back either (`planner:566-573` → `:1023/:1026/:1031`). Fixing only senders halves nothing.

### A3. Retune `r_claim` — mandatory companion to A1/A2

With claims currently absent ~40% of the time and mis-aimed the rest, `r_claim=8.0` (`planner:308`) is harmless. Once A1+A2 make it live and accurate, an 8 m disc over a **median 6.3 m goal distance** suppresses a large share of every peer's reachable candidate set — the claimer's own body sits inside its own claim disc. Drop `r_claim_m` to **4.0** and `r_pose_m` stays 5.0. Both are already ROS params, so this is A/B-testable with no code change.

> Explicitly rejected: widening `r_pose` to 12-17 m. Over a ±10 m candidate set with peers reported near each receiver's own origin, that degenerates into a uniform radial push away from own-launch-point applied identically by all four drones — it does not discriminate *between* peers, it just makes everyone run outward faster, which is the observed failure amplified.

### A4. Soften the two hard zeros that Stage A will arm

`planner:1035-1038` hard-zeros any candidate within `visited_radius=1.5 m` for 60 s, while `self.visited_penalty = 0.1` sits declared and unused at `:407` ✓. Today `visited_positions` is written only by `Path complete` (`planner:724`), which fired **4 times fleet-wide** — so this is dormant. Every Stage A fix works by making arrivals happen, and FIS regenerates viewpoints at 1-2 m, i.e. *inside* the blackout band. Ship this in the same patch or the coordination fix will present as a regression:

```python
score *= self.visited_penalty        # planner:1037, replaces  score = 0.0
```

### A5. Clamp FIS viewpoints to the planner geofence

`bbox_max_x=10.0` with `geofence_margin=0.3` means `_in_geofence` requires `x < 9.7` (`planner:651-654`), while FIS emits viewpoints on the ±10 m boundary — the outer 0.3 m ring is permanently unreachable. 3 of 33 claims sat there and were guaranteed 20 s `max_age` (ghost re-claimed (10.0, −8.6) at t=109.0 having released it at t=108.6 ✓). Under A1 these become long-lived dead zones instead of 20 s ones. Clamp in the FIS node or hoist the planner bbox so both agree.

### A6. Fix the metric (blocking for all validation)

`/home/lucas/hercules-sim/fidelity_scorecard.sh:240-264`. Four changes:

1. **Apply spawn offsets** from `settings-fleet-4drone.json` before unioning. (Recomputed: as-measured 2245/3030/25.9% → spawn-corrected 2277/3029/24.8% ✓.)
2. **Divergence gate.** Any drone whose ground track leaves its own geofence by >2× reports `FAIL(odom_divergence)` as a **separate row** and is excluded from the overlap denominator. Today buckshee's runaway to (150, 88) *lowers* the reported overlap — dropping it moves the headline from 25.9% **up** to 35.2% ✓. A VIO fix currently looks like a coordination regression.
3. **Report the floor.** Add `overlap_floor_pct = max(0, 1 − arena_cells/effort_cells)` and grade against `floor + margin`, not a fixed 10%. At the observed effort the floor is 16.1% ✓; four parked drones already score 39.8% ✓.
4. **Add `fleet_overlap_concurrent_pct`** — cells covered by ≥2 drones within Δt = 10 s. This is the coordination-attributable share and it is the number Stages A-C actually move. The existing spatial-only number conflates "poor assignment" with "launched from the same line."

### A7. Fix the emulator's TX-window model (blocking for Stage B sizing)

`/home/lucas/hercules-sim/virtual_lora_radio_v2.py:870-903`. `prev_in_slot` is derived from `self._last_tx_poll`, which is only written inside the node's own-slot branch — so on the first poll of each cycle it carries a value from the *previous* cycle, making `prev_in_slot` ≈ `time_in_slot − 190` and `window_covered` unconditionally true whenever `airtime ≤ 30 ms`. The miss branch is therefore unreachable for **every payload the 26 B `max_tx_size` gate lets through** — confirmed by `miss=0` over 3790 transmits ✓. The clamp is deliberate (the in-code comment says it compensates for poll granularity), but its side effect is that **payload-size risk is not modeled at all**: the sim will green-light a 26 B design that has a 1 ms launch window on hardware. Initialize `_last_tx_poll` at slot entry rather than carrying it across cycles, and add a regression asserting that 26 B produces measurably more window misses than 12 B.

### A8. Optional — suppress the payload when no claim is active

`coordination_node.py:268` sends 19 B **unconditionally every cycle** ✓ (`custom=3651/3790`), including the ~40% of the run with `claim_state == NONE` where `as_packet_fields()` returns all zeros. Skipping the attach drops those cycles from 26.944 ms to 20.544 ms airtime and from a 4 ms to a 10 ms window.

**Caveat, and why this is optional:** it makes airtime *variable*. The resync path assumes a fixed 20 ms airtime while TX uses the real value, so the sync authority injects `(airtime − 20)` ms of epoch error into every follower — a constant 6.944 ms today, but a 6.4 ms square wave if size alternates. Gate this to non-authority nodes, or skip it: **Stage B supersedes it** by making the payload a constant 12 B, which is both smaller and jitter-free.

### Stage A: expected effect

| Effect | Reasoning |
|---|---|
| `max_age` share of releases: 81% → <10% | `reached` becomes reachable (A1a+A2); ETA deadline replaces the flat 20 s (A1c) |
| Claim-active fraction: 46-72% → >95% | hysteresis no longer rejects (A1b); no 10.9 s re-claim gap after a spurious release |
| Claim positional error: 3-9 m → ~0 | frame fix (A2) |
| Claims reaching the wire: 33/91 plans → ~91/91 | A1b |
| `fleet_overlap_concurrent_pct` | **first meaningful measurement** — the mechanism was inoperative before |
| `fleet_overlap_pct` (corrected metric) | modest, direction uncertain [inf] |

I will not put a number on Stage A's overlap reduction, and neither should the sprint plan. Stage A turns a non-functioning mechanism *on*; whether a correctly-aimed 4 m disc reduces or increases measured overlap depends on the transit-vs-assignment tradeoff (a partition-like rule forces drones to reject near frontiers and fly farther, converting assignment overlap into transit overlap). **The honest deliverable of Stage A is a valid experiment**, not a number. Anyone promising 26% → 15% from Stage A is guessing.

---

## 3. STAGE B — spatial-temporal intent on 12 bytes

**Gate before building:** Stage A merged; `max_age` releases <10%; ≥50% of plans reaching `Path complete`; A7 landed. If the fleet is still completing 4% of its paths, Stage B encodes a 2-waypoint, 6.3 m segment replanned every 0.5 s — a polyline shorter than the exclusion radius it replaces. **Do not build B against the current flight behavior.**

### B0. Byte-budget ground truth (reproduced with their own `lora_airtime_ms`)

Airtime is a **step function** in 4-byte blocks; bytes *within* a block are free, so every payload must be sized to a block ceiling.

| custom | total | airtime | window span | admissible launch ms |
|---:|---:|---:|---:|---:|
| 0-2 B | 40 B | 20.544 ms | 10 ms | 11 |
| 3-5 B | 43 B | 21.824 ms | 9 ms | 10 |
| 6-9 B | 47 B | 23.104 ms | 7 ms | 8 |
| **10-12 B** | **50 B** | **24.384 ms** | **6 ms** | **7** |
| 13-16 B | 54 B | 25.664 ms | 5 ms | 6 |
| **17-19 B (today)** | 57 B | 26.944 ms | **4 ms** | 5 |
| 20-23 B | 61 B | 28.224 ms | 2 ms | 3 |
| 24-26 B | 64 B | 29.504 ms | **1 ms** | 2 |
| ≥31 B | 69 B | 32.064 ms | — | 0 (impossible) |

**The 26 B cap is a firmware attach limit, not a usable budget.** Cap the encoder at 19 B in code and design to 12. Note `test_lora_packet.py:198-205` asserts ≥30% slot slack, which 19 B passes (32.6%) and 20 B fails — 12 B keeps that test green with margin.

### B1. Wire format — SPACETIME INTENT, fixed 12 B, every cycle

Replaces the 19 B claim packet entirely. **Fixed size is mandatory**, for two verified reasons: the firmware's custom queue is single-deep **keep-OLDEST** (a newer submission is silently discarded), so alternating frame types means a missed TX destroys the *other* type and can phase-lock a node onto one; and variable airtime modulates the resync epoch error across the whole fleet.

The sender's pose is **not** in the payload — it rides free in the 38 B base packet (x/y/z/yaw as 4 floats). Every spatial field chains from it.

```
byte  bits  field        encoding
────────────────────────────────────────────────────────────────────────────
  0   7:6   ver          = 2   (bump; unpack_intent rejects mismatch → clean
                                mixed-fleet failure, peer ignored not misdecoded)
      5:3   fmt          = 1   (1 = intent; 2..7 reserved for Stage C)
      2:0   mode         MODE_* (coordination_logic.py:25-28)

  1   7:0   seq          u8, wraps — preserves the seq-gap delivery accounting
                         in PeerTable.update_claim (coordination_logic.py:85-110)

  2   7:6   n_hop        0..3  valid hops. 0 = no active intent.
      5:4   state        0=NONE 1=ENROUTE 2=ON_STATION 3=rsvd
                         (resurrects the dead CLAIM_HOLDING distinction —
                          mark_holding() at coordination_logic.py:228-230 has
                          zero callers today, so peers cannot tell "15 s out"
                          from "on station")
      3:0   lane_id      0..14, 15 = unassigned  (was a whole byte at 255)

  3   i8    dx1          0.25 m/LSB, −32.00 .. +31.75 m   hop 1 from SENDER POSE
  4   i8    dy1
  5   i8    dx2                                            hop 2 from node 1
  6   i8    dy2
  7   i8    dx3                                            hop 3 from node 2 = GOAL
  8   i8    dy3

  9   u8    t_arr        0.2 s/LSB, 0 .. 50.8 s. Time from THIS PACKET'S TX
                         instant to the final node. 255 = unknown.

 10   7:4   t_dwell      0..15 s, 1 s/LSB — planned on-station time at the goal
      3:2   clr          advertised clearance index → {1.5, 3.0, 5.0, 8.0} m
                         (EGO-Swarm's per-drone des_clearance: a degraded or
                          position-uncertain node asks for more room, no
                          renegotiation)
      1:0   conf         ETA confidence 0=none 1=low 2=med 3=high

 11   u8    crc8         poly 0x07, init 0x00, over bytes 0..10
────────────────────────────────────────────────────────────────────────────
12 B → 50 B PHY → 24.384 ms → 6 ms window, 7 admissible launch instants.
vs today: +50% window span, +40% launch instants, −2.56 ms airtime, constant size.
```

**Fields deleted and why (all verified dead):** `src_id` (the radio frame already prefixes the sender node id, surfaced as `PeerMessage.src_node`; `coordination_node.py:206-212` ignores it and re-reads the payload copy) · `ref_id` (hardcoded 0 at `:278`, read by nobody) · `batt` (`self.batt_pct = 0` at `:106`, never written — no battery subscription exists, so `FLAG_LOW_BATT` is structurally unreachable at `:276`) · `claim_z` (always `flight_height`, `planner:1165`; every consumer is 2-D) · `claim_gain` (packed, stored, republished, and discarded as `_gain` by both `discount_claim:163` and `loses_tie_break:183`) · 1 CRC byte (CRC8 over a link that already has a PHY CRC and a serial XOR) · 6 bits/axis of position (i16 cm gives ±327 m at 1 cm against a ±10 m arena, a 0.2 m voxel and a 0.3 m position tolerance).

**Clock skew is designed out, not tolerated.** Every time field is relative to the sender's TX instant and the receiver stamps `t_rx` locally. Clock *offset* drops out entirely — no NTP, no PTP, no timestamp exchange, and no dependence on the firmware's `cycle_clock`, which is not exported to ROS anyway. Only rate skew survives (~±100 ppm [inf] → 2 ms over 20 s → 4 mm at 2 m/s). The one residual is one-way latency `L ≈ 24.4 ms airtime + ~1.5 ms serial + ~0.5 ms USB ≈ 26 ms`; subtract it as a constant (`t_ref = t_rx − L_NOMINAL`), worth ~2.6 cm at 1 m/s, 1/10 of a quantization cell.

### B2. Encoder

```python
HOP_LSB = 0.25;  T_LSB = 0.2;  MAX_HOPS = 3;  HOP_LIM = 31.75

def encode_intent(pose_xy, waypoints, eta_s, dwell_s, state, mode,
                  lane_id, clr_idx, conf, seq):
    # waypoints: world-frame [(x,y,yaw)] already LOS-pruned by simplify_path
    # (planner:1145-1147) and resampled (planner:1199-1229).
    pts = douglas_peucker([(w[0], w[1]) for w in waypoints], tol=0.5)[:MAX_HOPS+1]
    hops, prev_hat = [], pose_xy          # prev_hat = RECONSTRUCTED previous node
    for p in pts[1:]:
        dx = clamp(round((p[0]-prev_hat[0])/HOP_LSB), -128, 127)
        dy = clamp(round((p[1]-prev_hat[1])/HOP_LSB), -128, 127)
        hops.append((dx, dy))
        # ERROR FEEDBACK: chain from what the DECODER will see, not from truth.
        # Naive chaining accumulates to ±0.5 m over 3 hops — comparable to
        # r_keepout — and would silently poison deconfliction. This bounds the
        # error at ±0.125 m per node forever, independent of chain length.
        prev_hat = (prev_hat[0] + dx*HOP_LSB, prev_hat[1] + dy*HOP_LSB)
    n_hop = len(hops)
    while len(hops) < MAX_HOPS: hops.append((0, 0))

    b = bytearray(12)
    b[0] = (2 << 6) | (1 << 3) | (mode & 0x07)
    b[1] = seq & 0xFF
    b[2] = ((n_hop & 3) << 6) | ((state & 3) << 4) | (lane_id & 0x0F)
    for i, (dx, dy) in enumerate(hops):
        b[3+2*i] = dx & 0xFF;  b[4+2*i] = dy & 0xFF
    b[9]  = 255 if eta_s is None else clamp(round(eta_s/T_LSB), 0, 254)
    b[10] = (clamp(round(dwell_s), 0, 15) << 4) | ((clr_idx & 3) << 2) | (conf & 3)
    b[11] = crc8(b[0:11])
    return bytes(b)
```

If the planned path exceeds 3 hops (observed: median 2, p90 3, max 5 ✓), **truncate** and set `t_arr` to the ETA of the 3rd node. A partial-horizon commitment is correct — peers only need the near-term intent, and the horizon still exceeds the 0.5 s replan interval.

### B3. Decoder and peer prediction — the part that kills `max_age`

```python
def decode_intent(buf, peer_pose_xy, t_rx):
    if len(buf) != 12 or crc8(buf[0:11]) != buf[11]: raise ValueError('crc')
    if (buf[0] >> 6) != 2: raise ValueError('version')          # mixed-fleet safe
    n_hop = buf[2] >> 6
    nodes, prev = [peer_pose_xy], peer_pose_xy
    for i in range(n_hop):
        prev = (prev[0] + i8(buf[3+2*i])*HOP_LSB,
                prev[1] + i8(buf[4+2*i])*HOP_LSB)
        nodes.append(prev)
    eta = None if buf[9] == 255 else buf[9]*T_LSB
    return dict(nodes=nodes, t_ref=t_rx - L_NOMINAL, eta_s=eta,
                dwell_s=buf[10] >> 4, clr=CLR_TABLE[(buf[10] >> 2) & 3],
                conf=buf[10] & 3, state=(buf[2] >> 4) & 3,
                lane_id=buf[2] & 0x0F, mode=buf[0] & 7, seq=buf[1])

def predict(peer, t):
    """Peer's predicted position at absolute local time t, plus a confidence
    weight in [0,1]. NEVER returns None and NEVER expires — this is the
    structural fix for 17/21 max_age releases."""
    nodes = peer['nodes']
    if len(nodes) < 2 or peer['eta_s'] in (None, 0):
        return nodes[0], 1.0                      # pose-only fallback
    segs = [dist(nodes[i], nodes[i+1]) for i in range(len(nodes)-1)]
    L = sum(segs) or 1e-6
    dt = t - peer['t_ref']
    t_end = peer['eta_s'] + peer['dwell_s']

    if dt <= peer['eta_s']:                       # in transit: arclength interp
        return lerp_along(nodes, segs, L * dt / peer['eta_s']), 1.0
    if dt <= t_end:                               # on station at the goal
        return nodes[-1], 1.0

    # PAST THE HORIZON — degrade continuously, never delete.
    # EGO-Planner-v2 (swarmGradCostP) extrapolates a peer at its terminal
    # velocity rather than dropping it; there is no max-age deletion anywhere
    # in that loop. Our peer parks at its goal, so the correct analogue is to
    # hold position and decay CONFIDENCE, not to inflate the exclusion.
    stale = dt - t_end
    return nodes[-1], max(0.0, 1.0 - stale / T_DECAY_S)   # T_DECAY_S = 8.0
```

**This is the single highest-value line of the whole redesign.** A representation with a hard lifetime can always produce "the claim died while its owner was still en route." One that fades cannot. Liveness is handled where it belongs: **expire the PEER, never the CLAIM** — the existing 3.0 s `peer_timeout_s` (`coordination_logic.py:112-121`) is correct and needs 15.8 consecutive losses to fire (measured: 0 in 180 s ✓). RACER has no orphan-reclaim rule at all; ours is already there.

### B4. Receiver-side scoring — replaces the binary disc hard-zeroing

Current block, `planner:1012-1049`: `gain/(dist+eps)` × static 8 m disc × static 5 m disc, then **three hard zeros** — `r_keepout=3` co-location, `visited_radius=1.5`, and `loses_tie_break` (a 201 m² veto granted per lower-id peer on seniority).

```python
# planner:1012-1049 replacement
t_now = self.now_s()
v_me  = max(self.v_est, 0.15)

for k in range(n_vp):
    vx, vy, vyaw = self.viewpoints[k]
    dist  = math.hypot(vx - self.flu_x, vy - self.flu_y)
    score = (self.gains[k] if k < len(self.gains) else 0.0) / (dist + self.score_eps)

    # WHEN would I be there, and for how long?
    t_f    = t_now + dist / v_me
    w_lo   = t_f - self.tau_before_s          # 2.0
    w_hi   = t_f + self.dwell_est_s           # 4.0

    for peer in self.live_peers:
        # min separation over MY occupancy window — space AND time
        d_min, w_conf = 1e9, 1.0
        for t in frange(w_lo, w_hi, self.t_sample_s):     # 0.5 s → ~13 samples
            p, c = predict(peer, t)
            d = math.hypot(vx - p[0], vy - p[1])
            if d < d_min: d_min, w_conf = d, c

        # EGO-Swarm clearance shell: SUM of both advertised clearances, ×1.5
        C = (self.my_clearance + peer['clr']) * 1.5
        if d_min >= C:
            continue                          # exactly zero — no cliff, no thrash
        # cubed hinge (Zhou Eqs.17-19): C²-continuous, zero gradient outside C
        pen  = ((C*C - d_min*d_min) / (C*C)) ** 3
        score *= 1.0 - (1.0 - self.w_peer) * pen * w_conf

        # SAFETY keep-out, evaluated at MY arrival time, not at t_now.
        # Avoidance re-shapes the path; it must not decide the goal (RACER VI-B3).
        p_at_arrival, _ = predict(peer, t_f)
        if math.hypot(vx - p_at_arrival[0], vy - p_at_arrival[1]) < self.r_keepout:
            score = 0.0; break

    for pvx, pvy, pt in self.visited_positions:
        if math.hypot(vx - pvx, vy - pvy) < self.visited_radius:
            score *= self.visited_penalty     # A4: soft, not 0.0
            break

    # lower-ID-wins survives, but only as a genuine CONFLICT test:
    # both space AND time must actually collide, not merely "within 8 m".
    if (COORD_OK and score > 0.0 and self.vehicle_id > 0
            and conflicts_in_spacetime(self.vehicle_id, (vx, vy), t_f,
                                       self.live_peers, C)):
        score = 0.0
    if score > 0.0: scored.append((score, k))
```

What changes, mechanically:

- **`discount_claim` and `discount_peer_pose` merge into one mechanism.** A peer's current pose is just `predict(peer, t_now)` — the t=0 point of its own polyline. Two discs become one time-indexed shell.
- **"Same space at different times" becomes real.** A peer that clears a corridor at t=3 s contributes exactly zero to a candidate I reach at t=15 s. Under the current code that ground is suppressed for the full 20 s claim life.
- **No score cliffs.** The cubed hinge has zero value *and zero gradient* at `d = C`, so candidate scores change smoothly as peers move. The current smoothstep has a discontinuous derivative at `r_claim` and the three hard zeros are outright cliffs — a direct target-thrash source.
- **`loses_tie_break` is preserved as the conflict resolver.** Keep it: it is the only rule that stays consistent when two nodes hold different subsets of the peer table, which is the normal case on a broadcast medium. Only its *trigger* narrows, from "any lower-id peer within 8 m" to "an actual space-time collision."

### Stage B: expected effect

| Effect | Reasoning |
|---|---|
| `max_age`-class releases | **eliminated by construction** — no representation with a lifetime remains; peer knowledge decays over `T_DECAY_S` and liveness is a peer lease |
| Falsely-suppressed candidates | 201 m² of blanket veto per lower-id peer replaced by a ~4.5 m time-gated shell (`clr` index 0 → `C = (1.5+1.5)×1.5`); at a median 6.3 m goal distance this is the difference between "most of my reachable set is vetoed" and "only genuine conflicts are" |
| TX window | 4 ms → **6 ms** (+50%), airtime −2.56 ms, size constant |
| Fleet channel occupancy | 56.7% → 51.3% |
| `fleet_overlap_concurrent_pct` | direct reduction — this is the term the penalty targets |
| `fleet_overlap_pct` (total) | **modest** [inf] — see §0: time-separated coverage is still redundant coverage. B removes false exclusions; C is what removes true overlap |

**Do not sell Stage B as the overlap fix.** It is the *precondition* for the overlap fix: it takes the avoidance layer out of the assignment decision so Stage C can own that decision cleanly.

---

## 4. STAGE C — region ownership by deterministic partition

**Gate:** Stage B merged and measured; `fleet_overlap_concurrent_pct` still above `floor + 5 points`.

RACER's real contribution is that **the task unit is a region carrying workload, not a frontier point** — "a large unexplored area may lie behind a small frontier cluster, and vice versa, thus coordinating robots by frontier clusters usually leads to unbalanced partitioning of workloads" (Sect. VIII-A1). Their measured value for the task-unit change alone: Office scene 47.5 s → 35.4 s (Tab. I). And ownership acts as a **filter** on which frontiers a drone may consider (Sect. VI, lists `F_a` / `F_s`), not as a score penalty — overlap is prevented by construction rather than discouraged.

### C1. What NOT to build from RACER

- **Not the pairwise request/response transaction (Alg. 2).** Each node transmits once per 190 ms with a single-deep **keep-OLDEST** custom queue and no back-channel. A round trip is 2-3 cycles (380-570 ms), 5-6 with the recommended k=3 blind repeat (~1.1 s/trade, ~6 s for a fair round over 6 pairs) — and every transaction packet **displaces an ownership snapshot in that one-deep queue**. Today `qdrop=1/3790` precisely because there is exactly one payload type ✓.
- **Not the CVRP partitioning messages.** RACER's Parti+Traj traffic is "<50 kB/s" (Sect. VIII-C, Fig. 14c) against our 26 B / 190 ms = 137 B/s — **365× over budget**, and its hgrid cell set *grows* during exploration, so it is unbounded by construction.
- **Not hgrid's online octree subdivision.** It exists to bound cell count in large scenes and is the source of the "inconsistent views" failure that `PairOptResponse.msg` exists to report.
- **Not a shared global cell-ID space, yet.** Frame offsets are 3-9 m and odometry drift moved ghost's odom→world map >20 m and buckshee's >140 m within 160 s ✓. A grid index derived from a frame that translates 20-140 m per sortie does not name the same ground twice, and durable ownership would integrate that error *forever* instead of self-clearing every 20 s. Also note the researchers sized bitmaps for 1225 m² and 2304 m² arenas; `radio4` passed no bbox params, so the real arena was the ±10 m default — a **630-cell union** ✓, 4-5 B of bitmap, not 16-26 B.

### C2. What to build instead — deterministic partition, zero new bytes

Every node computes the **same** partition from the **same** input: the sorted set of live peer ids. This is already the pattern `assign_lane` uses (`coordination_logic.py:271-279`) with zero payload. It is legitimate here because the peer set is agreed: 99.69% delivery, 15.8 consecutive losses needed to expire, **0 expiries in 180 s** ✓.

```python
def my_region(self_id, live_ids, arena):
    ids = sorted(live_ids | {self_id})
    k, n = ids.index(self_id), len(ids)
    return contiguous_split(arena, n)[k]     # CONTIGUOUS, not interleaved
```

- **Contiguous, not interleaved.** `assign_lane:271-279` gives vehicle *k* lanes *k, k+n, k+2n* — at `lane_spacing_m=4.0` over a 20 m span with 4 drones, each lane transition is a 16 m traverse crossing all three peers' active lanes, a guaranteed overlap corridor. The docstring's fault-tolerance rationale (a dead drone's gaps spread evenly) is real, so keep both behind a param and measure. *Caveat: `BOUSTROPHEDON` never engaged in `radio4`* ✓ *— 0 hits across all four planner logs — so this is untested territory, not a tuning exercise.*
- **Ownership filters, it does not penalize.** Candidates outside my region are dropped from `scored` unless my region yields no viable candidate — then fall back to global scoring. This is MinPos's non-starving property, and it matters because this planner already has a starvation path (A4).
- **Ownership never expires.** It changes only when the live-peer set changes — the correct trigger, and it recomputes automatically on peer loss, so RACER's missing orphan-reclaim rule never arises.
- **The `lane_id` nibble (byte 2 bits 3:0) carries the sender's *own* region index** as a consistency check, not as authority. On disagreement the receiver adopts the advertised value (the sender is authoritative about itself) — idempotent snapshot semantics, resynchronized by any single received packet, in one cycle.
- **No handshake, no transaction, no ACK, no double-check race, no queue contention.** The entire negotiation is deleted, and the byte cost is zero.

### C3. C2 — workload weighting (gated, do not build yet)

Adopt RACER's **capacity constraint** (Eq. 6: per-vehicle unknown-voxel demand bounded by `α_ρ` × total) rather than its path-length objective — their ablation shows the balance term, not the path term, is what stops one drone finishing early and re-entering a peer's region (Tab. I: H+mTSP 41.7 s vs full 35.4 s). **Blocked on the shared map**: `peer_map_integrator` logged `ok=0` for every peer for the entire run ✓ — the Zenoh/WiFi map channel delivered nothing, so each drone's 300+ frontier clusters are entirely private. Ownership can only be asserted over unknown-to-me space, and two drones will confidently claim regions the other has already mapped. **Fix the map channel first** — that is where the redundancy information actually lives, and it is a stronger candidate for the next sprint than any wire-format work.

Also worth measuring independently: RACER's coverage-path *visiting order* within owned cells (Sect. VI). Their single-drone ablation — no coordination effect at all — is 120.7 s / 155.9 m without it vs 96.9 s / 125.1 m with (Tab. II), a 20% saving purely from finishing a region before moving on. HERCULES may be paying that cost on top of the overlap.

### Stage C: expected effect

Region ownership is the only stage that removes overlap *by construction*. Its ceiling is set by the arena arithmetic, not the algorithm: at the in-arena effort flown (751 cells into a 630-cell arena) the floor is 16.1% ✓ and <10% is unreachable. Reaching the stated target requires **lower redundant effort** (more of the 91 plans completing) or **a larger arena** (raise the ±10 m bbox), and both are outside the coordination stack. Expected: `fleet_overlap_concurrent_pct` → near the floor [inf]; `fleet_overlap_pct` → floor + a few points, where the floor must be reported alongside it.

---

## 5. Files and line ranges

Paths are absolute. **`/home/lucas/hercules-sim/src/active_exploration_prefix/scripts/simple_exploration_planner.py` is a diverged near-duplicate** (coordination lines offset ≈ −23) — every planner change must land in both copies or the two packages will disagree on the wire protocol.

| Stage | File | Lines | Change |
|---|---|---|---|
| A1a | `.../multi_drone_nvblox/scripts/coordination_node.py` | 223-240 | honor `reached` → `release()` directly |
| A1b/c | `.../multi_drone_nvblox/mdn_core/coordination_logic.py` | 193-248 | displacement hysteresis; ETA deadline; progress watchdog; new fields in `__init__` |
| A1c | `.../scripts/coordination_node.py` | 84-87, 243-256 | retire `claim_hold_s`, add `claim_move_eps_m`/`stall_timeout_s`/`eta_margin`; raise `claim_max_age_s`→60; pass `remaining_m` |
| A1d | `.../active_exploration/scripts/simple_exploration_planner.py` | 585-591 | read `msg.vx/vy`, EWMA `v_est` |
| A1e | `.../simple_exploration_planner.py` | 1199-1229, 1160-1168, 717-724 | return path length; publish `path_length`+`eta_s`; progress intents |
| A1e | `.../multi_drone_nvblox/msg/ClaimIntent.msg` | — | **additive**: `float32 path_length`, `float32 eta_s`, `float32 remaining_m` |
| A2 | `.../multi_drone_nvblox/config/swarm_alignment.yaml` | all | 4 vehicles, real spawn offsets, correct id↔name map |
| A2 | `.../scripts/coordination_node.py` | 234-236, 260-267 | transform claim via `T_world_self`; refuse TX when `None` |
| A3 | `.../simple_exploration_planner.py` | 308-314 | `r_claim_m` 8.0 → 4.0 |
| A4 | `.../simple_exploration_planner.py` | 1035-1038, 406-407 | `score = 0.0` → `score *= self.visited_penalty` |
| A5 | FIS `frontier_info_structure_node.cpp` | bbox init ~18-23 | clamp viewpoints to `bbox ± geofence_margin` |
| A6 | `/home/lucas/hercules-sim/fidelity_scorecard.sh` | 240-264 | spawn offsets; divergence gate; `overlap_floor_pct`; `fleet_overlap_concurrent_pct` |
| A7 | `/home/lucas/hercules-sim/virtual_lora_radio_v2.py` | 870-903 | init `_last_tx_poll` at slot entry; regression on 26 B vs 12 B |
| B1-3 | `.../multi_drone_nvblox/mdn_core/lora_packet.py` | 248-293 | add `pack_intent`/`unpack_intent`; **keep `pack_claim` for rollback**; `INTENT_VERSION = 2` |
| B3 | `.../mdn_core/coordination_logic.py` | 41-66, 85-110, 130-142 | `PeerEntry.__slots__` gains `nodes[4]`, `t_ref`, `eta_s`, `dwell_s`, `clr`, `conf`; `predict()` |
| B4 | `.../mdn_core/coordination_logic.py` | 149-190 | cubed-hinge shell replaces `smooth_discount`+`discount_claim`+`discount_peer_pose`; narrow `loses_tie_break` |
| B4 | `.../simple_exploration_planner.py` | 1012-1049, 566-573 | scoring rewrite; widen `_peers_cb` tuple to carry `age_s` and the polyline |
| B | `.../msg/PeerState.msg`, `SwarmPeers.msg` | — | polyline + timing fields; **rebuild `multi_drone_nvblox` and all dependents** (`CMakeLists.txt:17-22`) |
| B | `.../scripts/coordination_node.py` | 268-286, 288-322 | 12 B TX; `PeerState` assembly |
| C | `.../mdn_core/coordination_logic.py` | 271-279 | contiguous vs interleaved partition behind a param |
| C | `.../simple_exploration_planner.py` | 1012-1049 | region filter with global fallback |
| tests | `.../multi_drone_nvblox/test/test_lora_packet.py` | 179-205 | 12 B round-trip; keep the ≥30% slack assertion (12 B passes wide) |
| tests | `.../test/test_coordination_logic.py` | 19, 73-141 | discount shape, tie-break, hysteresis, `as_packet_fields` all break by design |

---

## 6. Validation

Measure with the **A6-corrected** scorecard only. Every run reports five numbers: `fleet_unique_m2`, `fleet_overlap_pct`, `overlap_floor_pct`, `fleet_overlap_concurrent_pct`, `odom_divergence[d]`.

**Baselines to re-establish after A6/A7 (the current 32%/26% are not comparable):**

| Config | Purpose |
|---|---|
| radio-dead | coordination off |
| radio-live, stock code | today's behavior on the corrected metric |
| radio-live, Stage A | frames + lifetime + metric |
| radio-live, Stage A, `r_claim=8` vs `4` | isolates A3 |
| radio-live, Stage B | spatial-temporal |
| radio-live, Stage B+C | ownership |

**Per-stage exit criteria:**

- **A:** `max_age` share of releases <10% (from 81%); `reached` >50%; claim-active fraction >95%; claim positional error vs ground truth <0.5 m; `odom_divergence` reported as its own row; ≥3 seeds. *Gate to B:* ≥50% of `PLAN OK` reaching `Path complete` (from 4.4%) and <20% of `NO PATH` being "no clusters" (from 87%). **If this gate fails, the bug is in FIS/VIO, not coordination — stop and fix that instead.**
- **B:** zero lifetime-class releases; measured TX window ≥6 ms in the fixed emulator; `fleet_overlap_concurrent_pct` down vs Stage A at equal `fleet_unique_m2`; **no reduction in unique coverage** (guards against over-suppression); target-switch rate per minute down (thrash proxy).
- **C:** `fleet_overlap_pct` within 5 points of `overlap_floor_pct`; no drone starved (every drone has a viable candidate ≥95% of ticks); region reassignment on induced peer loss completes within 3 s.

**Unit tests worth writing before the integration run:** encoder error-feedback bound (chained reconstruction error ≤0.125 m per node over 10⁴ random paths); `predict()` continuity across the transit/dwell/decay boundaries; `predict()` never returns `None` at any `t`; decoder rejects wrong version, bad CRC, out-of-bbox polylines, and implied speeds >5 m/s; scoring is exactly unchanged when all peers are >C away.

---

## 7. What NOT to copy from Zhou et al.

| Element | Verdict | Reason |
|---|---|---|
| `MINCOTraj.msg` verbatim | **Kill** | 127 B at M=2, 143 B at M=3, 175 B at M=5, 223 B at M=8 — **4.9-8.6× the 26 B cap**. One M=3 packet would occupy ~1.05 s of a node's entire channel. |
| `start_v`/`start_a`/`end_v`/`end_a` | **Kill** (−48 B) | The fleet is a waypoint follower with a yaw gate; `_publish_sp` NaNs velocity/accel/jerk (`planner:874-883`). Higher derivatives carry no actionable information. |
| Per-piece `duration[]` | **Kill** (−15 B) | MINCO optimizes time allocation per piece; this planner resamples at a fixed 0.5 m spacing and flies a commanded cruise. One scalar ETA is the honest encoding. |
| Absolute `time start_time` | **Kill** (−7.4 B) | Requires synchronized clocks. Relative-to-TX + local `t_rx` stamping removes clock offset from the design entirely, and the firmware's `cycle_clock` is not exported to ROS anyway (`FMT_PEER_POSE = '<Bffffhf'`). |
| `des_clearance` as float32 | **Adapt** | The *idea* is excellent — each drone advertises how much room it wants and peers sum both, ×1.5. Ship it as 2 bits (`clr`). |
| Their loss handling | **Kill** | No ACK, no retransmit, no sequence-gap detection, no dropout eviction. They lean on WiFi ("TP-LINK TL-XVR6000L" / "AzureWave AW-CB375NF", p.15). Our loss tolerance must come from idempotent snapshots + the peer lease, which we already have. |
| Constant-velocity extrapolation past horizon | **COPY — highest value** | `swarmGradCostP` never deletes a peer trajectory. This is the structural answer to 17/21 `max_age`. |
| Cubed-hinge soft penalty (Eqs. 17-19) | **COPY** | Zero value *and* gradient outside `C_w`; peers are a gradient, never a hard-zeroed exclusion. Directly opposite to our `r_claim`/`r_keepout` hard-zeroing. |
| Time-indexed distance evaluation | **COPY** | Own trajectory at `t_i` vs peer position at the corresponding instant. This is the "same space at different times" mechanism. |
| Deadlock handling | **Do not expect one** | "naturally mitigated... owing to real-world randomness, asynchronously triggered planning" (p.12). No detection, no resolution. Practical implication: keeping replan triggers **unsynchronized** is load-bearing, not incidental — the TDMA schedule already staggers nodes, which helps. |
| Independent safety monitor | **COPY** | Soft penalties are explicitly not trusted as the safety layer: a postcheck runs after every plan, plus a background collision check that can trigger an emergency stop (p.15). A LoRa port loses more packets than WiFi, so this matters *more* for us. |
| Their assignment layer | **Does not exist** | See §0. Take assignment from RACER. |

---

## 8. Risks and rollback

| # | Risk | Severity | Mitigation | Rollback |
|---|---|---|---|---|
| R1 | **Stage A arms the visited-position blackout.** `Path complete` fired 4× fleet-wide, so the 60 s / 1.5 m hard-zero at `planner:1035-1038` is dormant. Every Stage A fix works by making arrivals happen; FIS regenerates viewpoints at 1-2 m, inside the blackout band. | **High** | Ship A4 in the *same* patch. Assert "viable candidates ≥1" on every tick. | Revert A4 alone; it is one line. |
| R2 | **Correctly-aimed claims over-suppress.** `r_claim=8 m` > median goal distance 6.3 m. Today's mis-aimed, absent claims are accidentally harmless. | **High** | A3 (`r_claim` → 4.0) ships with A1/A2; both are ROS params, A/B-testable with no code change. | Param revert, no rebuild. |
| R3 | **Stage A looks like a regression on the current metric.** Fixing buckshee's divergence moves the headline 25.9% → 35.2% ✓. | **High** | A6 lands **before** any A/B. Report `overlap_floor_pct` and `odom_divergence` as separate rows. | n/a — this is a reporting fix. |
| R4 | **Emulator green-lights an unflyable payload.** The window-miss branch is unreachable for every attachable size (`miss=0`/3790 ✓). | **High** | A7 before Stage B. Cap the encoder at 19 B in code regardless. | 12 B is 7 ms of margin below the 19 B baseline; even a broken model cannot make it worse than today. |
| R5 | **Message/format change breaks the diverged planner copy.** `active_exploration_prefix` is a real second package. | Medium | CI check that both copies' coordination blocks are identical; `INTENT_VERSION = 2` makes a mixed fleet fail *clean* (peer ignored, `_on_peer_msg:206-210` already swallows `ValueError`) rather than misdecode. | Keep `pack_claim`/`unpack_claim` in place; a `use_intent_v2` param selects the codec at runtime. |
| R6 | **Bad ETA poisons the time gate.** `conf` low, speed estimate noisy at 0.21 m/s, or PX4 `vx/vy` invalid. | Medium | `t_arr = 255` (unknown) → receiver falls back to a pose-only, time-agnostic disc, i.e. today's behavior. Widen the occupancy window by `conf`. | Set `conf = 0` fleet-wide; the system degrades to Stage A semantics. |
| R7 | **Soft penalties cannot guarantee separation** under stale peer data — the exact reason Zhou et al. run an independent monitor (p.15). | Medium | Keep the hard `r_keepout` check, evaluated at `predict(peer, t_f)`. Add a background separation monitor outside the planner. | `w_peer = 0.0` restores hard exclusion at the shell. |
| R8 | **Stage C ownership never repairs.** Frame offsets 3-9 m plus 20-140 m/sortie drift; durable ownership integrates that forever, where the 20 s timer accidentally self-clears it. | **High** | C is gated on A2 landing *and* on drift bounds being measured. Ownership recomputes on peer-set change, so there is always a repair path. | C is a param-gated filter over the existing scorer — disable and the planner returns to Stage B scoring. |
| R9 | **Stage C over-partitions in a 20 m arena.** A hard partition forces rejecting a near frontier to fly farther inside your own region, converting assignment overlap into *transit* overlap (an independent model showed hard Voronoi at 27.8% vs 23.8% for the current policy at 0.4 m/s [inf] — and measured speeds here were 0.21-0.93 m/s, the regime where it loses). | Medium | Filter-with-global-fallback, not hard exclusion. Measure `fleet_unique_m2` alongside overlap — a partition that lowers both is not a win. | Param off. |
| R10 | **Target `<10%` is unreachable and the programme is judged against it.** At effort 751 into a 630-cell arena the floor is 16.1% ✓. | **High** | Renegotiate the target *now*: grade against `overlap_floor_pct + margin`, and track `fleet_overlap_concurrent_pct` as the coordination KPI. | n/a — this is a target definition, and it must be fixed before the sprint, not after. |

**Rollback posture overall:** Stage A is behavior-only with zero wire change — every item is independently revertible and most are ROS params. Stage B keeps `pack_claim`/`unpack_claim` alive behind a `use_intent_v2` param, so a fleet can be rolled back without a firmware or message rebuild. Stage C is a param-gated filter with a global fallback path. There is no point in the plan where a rollback requires reflashing.

---

## 9. Recommended sequencing

1. **A6 + A7 first** (metric + emulator). Neither touches flight code. Without them nothing downstream is measurable.
2. **A2** (frames + config). Zero code risk, unblocks every peer-aware quantity.
3. **A1 + A3 + A4 + A5** as one patch, then re-run `radio4` × 3 seeds.
4. **Gate check:** if `Path complete` is still <50% of `PLAN OK` or `NO PATH: no clusters` is still >20%, **stop**. The remaining waste is in FIS and VIO, not coordination — and the map channel (`ok=0` for every peer, entire run ✓) is the highest-value target in the whole system, because that is where redundancy information actually lives.
5. **Stage B** only after the gate passes.
6. **Stage C** only if B leaves `fleet_overlap_concurrent_pct` more than 5 points above the floor.

**Bottom line for the sprint plan:** the paper's principle is sound and the 12 B encoding is real and buys a wider radio window than the fleet has today — but `radio4` shows the link delivered 99.69% of packets with zero missed transmit windows and zero peer expiries, so the 26% was never a wire problem. It was a frame-registration problem (peer coordinates displaced 3-9 m against 3-8 m exclusion radii), a metric problem (39.8% geometric floor, 56% of the denominator from one drone's odometry runaway, and a <10% target that is arithmetically impossible at the effort flown), and a flying problem (4 of 91 plans completed, 92 of 106 planning failures were "no frontier clusters at all"). Stage A is where the value is this sprint. Stage B is the right *second* move.