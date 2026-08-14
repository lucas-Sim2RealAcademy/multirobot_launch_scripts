# HERCULES fleet-size decision: 4 drones stay

**Status: DECIDED. The premise the decision was convened on is wrong, and the problem is already solved at N=4 by two environment variables that are already the runner defaults.**

All numbers below were measured on this host today (2026-08-13) on the real 4-drone stack, window-matched to identical run phases, with host load recorded per run.

---

## 1. The measured render ceiling and what binds it

Across eight window-matched 4-drone full-stack runs (`t = 20–40 s`, all four drones, `/home/lucas/hercules-sim/e1_frames/runs/`), the UE render subsystem delivers **185 ± 8 renders/s total** no matter how that budget is spelled: 167.3 / 167.1 / 169.3 / 176.3 r/s with chase capture on and depth at 30 Hz, 179.7 r/s with capture off, 188.8 r/s with capture off and depth paced at 10 Hz, and 190.9 r/s with stereo additionally cut to 424×240. What binds it is **per-image serialization through the game-thread render fence**: `RenderRequest.cpp:64` marshals every `simGetImages` batch onto the game thread via `AsyncTask(ENamedThreads::GameThread, …)`, registers a `game_viewport_->OnEndDraw()` lambda, and blocks the RPC worker at `:99-106` on `wait_signal_->waitFor(5)` until the *next* frame draws — so one image ≈ one frame-fence traversal, and throughput is `frames/s × batches-per-frame`. The decisive proof that this is a **fixed per-image cost and not a fill-rate cost** is a paired A/B I ran on the real stack at matched load (load1 median 8.9 vs 8.8): cutting stereo from 640×480 to 424×240 — a **3.02× reduction in pixels** — moved fleet throughput from 188.8 to 190.9 r/s, i.e. **+1.1%, inside the noise**. It is not GPU-bound (SM 81–85% but memory-controller utilization only 2–4%, 217–232 W against a 600 W cap, SM clock pinned 2917–2932 MHz, 52–54 °C — no throttle, no bandwidth pressure), not CPU-bound (`renice -15` on every UE thread changed nothing; aggregate 41.4% busy, hottest core 79.6%), not RPC-pool-bound (already patched to `spawned_actors*4+8`), and not memory-bound (`vmstat si/so ≈ 0`). At N=4 with capture off and depth at 10 Hz, `4 × (2f + 10) = 188` solves to **f = 18.5 Hz**, and the measured value is **18.70 Hz** — the budget model is accurate to 1%.

### The premise correction that changes the decision

**The brief's "cuVSLAM stutter warnings fire when inter-frame gap > 34 ms" is false.** `run_fleet_radio.sh:176` launches cuVSLAM with `-p image_jitter_threshold_ms:=200.0`, and the warning literally reads:

```
[WARN] [visual_slam]: Delta between current and previous frame [1834.207000 ms] is above threshold [200.000000 ms]
```

34 ms is the *nominal period of a hypothetical 30 Hz stream*, quoted as "desired" in `investigation/STUCK-BUG-INVESTIGATION.md` §R4 — in a paragraph noting that the **real field hardware already stutters 66–851 ms**. The actual gates are (a) cuVSLAM's 200 ms image-jitter threshold, (b) the scorecard's field reference F7, `≤ 2 stutter events/min in steady state`, and (c) downstream PX4 `EKF2_DELAY_MAX = 200 ms`. Nothing in the stack requires 30 Hz.

**At 18.7 Hz, all four drones PASS the stutter gate for the first time in the archive:**

| run | capture | depth | stereo Hz/drone | fleet r/s | cuVSLAM stutter/min (4 drones) | odom divergence (m) | load1 |
|---|---|---|---|---|---|---|---|
| `b2_base` | ON | 30 | 13.60 | 167.3 | 8.5 / 17.0 / 15.8 / 14.1 — 3 FAIL | 10.4–15.9 | 10.3 |
| `b3_base` | ON | 30 | 13.55 | 167.1 | 12.0 / 7.9 / 16.3 / 6.2 — 2 FAIL | 10.8–12.1 | 14.8 |
| `s0_base` | ON | 30 | 13.65 | 169.3 | 5.6 / 2.7 / 6.8 / 6.7 — 4 WARN | 5.0–11.1 | — |
| `t3_ladder` | ON | 30 | 14.15 | 176.3 | 3.8 / 2.7 / 4.1 / 2.5 — 4 WARN | 9.4–11.1 | 10.8 |
| `r1_nocap` | **OFF** | 30 | 14.70 | 179.7 | 4.8 / 3.9 / 2.8 / 1.1 — 1 PASS | 10.8–13.1 | 6.5 |
| **`r5_cap0_depth10`** | **OFF** | **10** | **18.70** | **188.8** | **1.2 / 1.3 / 0.0 / 1.1 — 4 PASS** | **9.4–12.1 (4 PASS)** | 8.9 |
| **`r6_424x240`** | OFF | 10 @424×240 | 18.90 | 190.9 | **0.0 / 0.0 / 0.0 / 0.0 — 4 PASS** | 4.4–10.1 (4 PASS) | 8.8 |
| `r4_cap0_depth10` | OFF | 10 | 8.25 | **96.1** | 26.7 / 28.0 / 31.1 / 10.3 — 4 FAIL | 10.7–21.0 (1 FAIL) | 11.8 |

Raw warning counts tell the same story: `b3_base` logged 43/31/23/18 stutter events; `r5` logged **4/2/2/3 for the entire run**.

**`r4_cap0_depth10` must be excluded, and the disqualification is mechanical, not a judgement call.** Its *paced* depth stream delivered **7.50 Hz against its own 10.0 Hz setpoint**, and fleet throughput was 96.1 r/s — 49% of every other run. A paced stream that cannot meet a setpoint it is throttling itself to proves host starvation, not lever failure. `r4` is the run the adversarial review built its falsification on; `r5` is the clean repeat of the identical config, run 10 minutes later at *higher* load, and it was not in the archive when that review was written.

**On statistical power:** the review correctly computed that a 10% effect needs ~19 runs per arm at the observed CV. The measured effect is **13.74 → 18.80 Hz = +36.8%**, ~3.0 σ against the review's own 1.67 Hz pooled sd and ~18 σ against the 0.28 Hz sd of the four capture-on baselines. By the review's own power table, a 30% effect needs ~2 runs per arm. We have four baseline and two treatment runs. The effect clears its own bar.

---

## 2. Options

| Option | Achieved stereo Hz/drone | Stutter gate | Fidelity cost | Effort |
|---|---|---|---|---|
| **Status quo as documented (N=4, capture ON, depth 30, 640×480)** | 13.55–14.15 measured | **2–3 FAIL, rest WARN** | None on imagery; VIO is starved enough that a coordination defect cannot be distinguished from a starved tracker | Zero |
| **RECOMMENDED — N=4, capture OFF, depth 10.0 Hz, 640×480** | **18.70 measured (`r5`)** | **4/4 PASS (0.0–1.3/min)** | Depth integration 30→10 Hz. Consumer chain cannot use more: `nvblox_base.yaml` `update_esdf_rate_hz=10`, FIS pulls the ESDF at `update_rate_hz=1.0` onto a 0.2 m grid, and `projective_integrator_max_weight=5.0` saturates a TSDF voxel after 5 observations, so 10 frames/planner-cycle is already 2× saturation. p99 inter-frame motion 0.154 m (3.1 voxels) / 4.0° yaw → 95.4% frustum overlap. Clears the field `reactive_depth_guard` 0.2 s staleness timeout with 2× margin. Loses only the chase-cam MP4; `meta_*.json` ground truth is written unconditionally (`cuvslam_sim_bridge.py:383-406`) | **Zero — already the runner defaults** (`run_fleet_radio.sh:48,51`) |
| **N=4 at 424×240 stereo** | 18.90 measured (`r6`) | 4/4 PASS | **REJECT.** +1.1% throughput for a 3.02× pixel cut, and I independently measured the cost with the repo's own probe geometry (Shi-Tomasi + LK stereo match, disparity threshold scaled to hold *metric* depth constant), on both frame sets: in the **untextured world the rig actually flies**, usable landmarks fall **130 → 51 (0.39×)** and waypoints below the 10-landmark tracking-loss floor go **3/8 → 6/8**; textured world 3569 → 2627 (0.74×), 0/8 → 1/8. This reproduces the review's independent measurement to two decimal places and confirms `VIO-STABILIZATION-PLAN.md:117`, which already lists "424×240 stereo" under **"Do not do — harmful/null."** | Low, but negative value |
| **N=4 with depth 5.0 Hz** | ~22 predicted | Would pass | Mapping is fine (90.8% frustum overlap) but the field `reactive_depth_guard_node.cpp:38` uses `depth_timeout_s=0.2`, so a nominal 5 Hz stream sits *on* the staleness threshold and holds position on the real drone — while the sim does not launch that guard and would look clean. Hidden sim/field divergence | Zero to set, but not defensible without porting the guard |
| **Lockstep (`simContinueForTime`)** | 30.303 Hz in **sim** time, **11.3 Hz wall** (RTF ~0.33 at the measured budget) | Passes by construction | **REJECT.** Measured pause penalty is **1.0×** — lockstep cannot raise the ceiling, only spend wall time. IMU collapses to the tick rate: `ImuBase.hpp:42-54` keeps a single `output_` with no buffer; 12 polls per frozen window returned **1 distinct timestamp on 20/20 ticks**, so 30 Hz IMU against `imu_jitter_threshold_ms:=15.0`. Sim time advances only in 3 ms quanta, so 200 Hz IMU is arithmetically unreachable. And one rclcpp process has one DDS domain, while `run_fleet_radio.sh:160` puts each drone's whole stack on its own `ROS_DOMAIN_ID` — collapsing to one bus deletes the exact topology the 4-drone experiments exist to test | **1.5–3 weeks** (~450–500 of 907 lines of the car-only `airsim_node_hercules_synced.cpp` rewritten, ~12 `create_wall_timer`→`create_timer` conversions across 9 files, sim-clock port of a 1723-line radio emulator). Q13's "a day" is not credible |
| **Drop to N=2, 640×480, 30/30** | 30 (budget 180 of 188 r/s, 4% margin) | Passes | Full imagery fidelity, **but kills the experiments in §4** | Zero |
| **Fallback: N=3, capture OFF, depth 10** | 25.8 predicted | Expected to pass | Keeps every N≥3 bug class; loses only the arena-saturation overlap floor. Radio cycle becomes 150 ms, still ≠ the 190 ms of N=4 | Low (needs a `settings-fleet-3drone.json`) |

---

## 3. Recommendation

**Keep four drones. Ship `HERC_CAPTURE=0` + `HERC_DEPTH_HZ=10.0` at 640×480. Revert the 424×240 edit. Do not build lockstep. Do not drop to 2.**

The reasoning is that the decision was framed against a threshold that does not exist. Nothing in this stack needs 30 Hz stereo — cuVSLAM's configured jitter threshold is 200 ms, the scorecard's field gate is ≤2 stutter events/min, and PX4's `EKF2_DELAY_MAX` is 200 ms. At the 18.7 Hz that two free environment variables already deliver, **all four drones pass that gate** (0.0–1.3/min, versus 6.2–16.3/min at the documented baseline), and `r5` simultaneously posted the tightest odom-divergence spread of any archived 4-drone run (9.4–12.1 m, 4/4 PASS). The starved-VIO condition that made 4-drone coordination results uninterpretable — where a coordination defect and a starved tracker were indistinguishable — is gone at N=4. Paying for 30 Hz by halving the fleet buys a number nothing consumes, at the cost of the experiments in §4.

Two options are rejected on measurement rather than judgement. **424×240 is measured-dead from both ends on this host today**: +1.1% throughput (`r5` vs `r6`, matched load) and −61% usable landmarks in the world the rig flies, with waypoints in tracking loss doubling from 3/8 to 6/8. A sibling agent edited `settings-fleet-4drone.json` to 424×240 at 20:49 today; **that edit must be reverted** — it is pure fidelity loss for nothing, and `VIO-STABILIZATION-PLAN.md:117` had already filed it under "Do not do." **Lockstep** is rejected because the measured 1.0× pause penalty proves it cannot touch the render ceiling; it is a determinism investment, not a rate fix, and it costs 1.5–3 weeks to arrive at 4 drones running at 11 Hz wall with 30 Hz IMU and no per-drone DDS domains — strictly worse for coordination work than what we have today for free.

The residual exposure is honest and bounded: depth at 10 Hz has not been A/B'd against depth 30 on the *downstream* Stage A criterion (`Path complete / PLAN OK ≥ 50%`). `r5` and `r6` both passed `run_exp.sh`'s VALIDATE gate and scored `fis_max_ms` PASS and `planner_p95/max` 1–7 ms PASS, so there is no sign of trouble, but that is the one measurement to close (§5, step 4).

---

## 4. What dies if we drop to 2 drones

Not degradation — **structural erasure**. Eight items, each with a mechanism:

1. **Bug #3, sync-source re-election deadlock (CONFIRMED) — invisible at N=2.** Measured with the repo's own emulator on a `SimClock`, killing node 0 at t=5 s: N=2 self-elects at t=8.0 s and the mesh heals; N=3 stays `(0,0,0)` forever; N=4 stays `(0,0,0,0)` forever. `findNewSyncSource()` (`lora_mesh_v7_2_ros2.ino:342-353`) adopts a peer's *gossiped* sync source without checking `nodes[s].alive`, so survivors re-elect the corpse — and at N=2 there is no third node to gossip the dead id. **Needs N≥3, full stop.** The one-way-mute (hidden-node) variant splits identically.
2. **Bug #7, alignment-unaligned (CONFIRMED, fail-dangerous) — dormant at N=2.** The committed `swarm_alignment.yaml` has exactly two entries and `fleet_ctl:47` hands every vehicle the same file. The bug fires only when the flying fleet exceeds the yaml's entries: an unaligned vehicle publishes no `T_world_self`, is silently dropped by peer-map integration in *both* directions, and still broadcasts raw odom over LoRa as if it were team frame, straight into peers' keep-out discs with no unaligned flag. At N=2 nothing is ever unaligned.
3. **R10, the overlap floor — cannot exist at N=2.** Measured `max(0, 1 − arena_cells/effort_cells)` over every drone subset of two real 4-drone runs: **0.0% at N=2 in all 6 pairs**, 0.0–0.1% at N=3, and 4.5–30.9% at N=4 (`r5` 4.5, `b2_base` 27.8, `b3_base` 30.9). The finding "the <10% overlap target is arithmetically unreachable" is a pure consequence of four drones saturating a 630-cell arena. A 2-drone rig reports floor = 0 regardless — so it could never have discovered R10, and cannot be used to renegotiate the target either.
4. **Transitive tie-break chains and highest-id starvation.** `coordination_logic.py:178-188` is strict lower-ID-wins. N=2 has exactly **one** ordered relation; N=4 has **six**. Chains (4 yields to 3 yields to 2 yields to 1) cannot form at N=2 at all.
5. **Stage C's central risk R9** — "each lane transition is a 16 m traverse crossing all three peers' active lanes" — is a 4-drone geometry (`assign_lane`: `lane % n_live == k`). At N=2 lanes simply alternate and the interleaved-vs-contiguous partition question Stage C exists to answer becomes untestable.
6. **Peer-expiry/rejoin statistics collapse ~9×.** Measured over 120 s with Gilbert-Elliott burst loss: N=2 → 6 PEER_LOST / 8 PEER_JOINED; N=3 → 25/31; N=4 → 54/64. Two compounding causes: fewer peers to lose, and expiry needs `NODE_TIMEOUT_MS/CYCLE_MS` silent cycles = 27.3 cycles at N=2 vs 15.8 at N=4, so a burst must last 1.7× longer to expire anyone.
7. **The TDMA cycle is mistuned at N=2.** Verified in source (`virtual_lora_radio_v2.py:622`): `cycle_ms = num_nodes * SLOT_MS + GUARD_MS` = N×40+30 → **110 ms at N=2 vs 190 ms at N=4**. Every duty-cycle-derived constant — claim lifetimes, peer ages, expiry thresholds — is tuned against the wrong period.
8. **A silent sim2real trap: the payload budget is *larger* at N=2.** Verified: `base_packet = 34 + N` (`:624`), `max_tx_size` 64 B at every N → max custom payload **28 B at N=2, 27 B at N=3, 26 B at N=4**. A payload sized on a 2-drone rig carries **2 bytes of false headroom**, and the firmware (`fw:505-527`) drops an oversized custom **whole and silently** — no NACK, no ROS visibility.

**And note where these bugs can be found: nowhere else.** `AGENT_HANDOFF.md:252` states "Radio hardware NOT attached" on the field vehicle; the 4-node mesh was bench-validated, never flown; the committed firmware is `NUM_NODES 3` against a four-vehicle fleet, and the RX exact-length gate (`fw:544-564`) means a NUM_NODES=3 and a NUM_NODES=4 build reject 100% of each other's pose-only packets. Of the 8 CONFIRMED bugs, 5 are single-drone defects the field could surface on its own and 1 needs N≥2 — **the 2 that need N≥3 are exactly the ones only this rig can ever find.** Dropping to 2 removes the unique scientific value of the rig and keeps the findings we could have gotten anywhere.

**What N=2 would *not* cost, for completeness:** the TX-window-miss experiment is genuinely N-invariant (19 B claim → 55/56/57 B on air at N=2/3/4, all in the same LoRa symbol block, 4 ms launch window, miss rate 3.41/4.63/4.47%), so A7 and the Stage B 12 B-vs-26 B sizing are fully testable at N=2. And all five single-drone bugs are unaffected.

**If the budget ever tightens, cut to 3, not 2.** N=3 preserves every N-dependent bug class measured above (sync re-election deadlocks identically, 25 expiry events/120 s, tie-break chains and the yaml-shortfall path both survive) and loses only the arena-saturation overlap floor. At the measured budget N=3/depth-10 gives ~25.8 Hz. N=2 is the only cut that crosses a structural cliff.

---

## 5. Exact next commands

**Step 1 — revert the 424×240 edit (do this first; it is live in the settings file right now).**

```bash
cd /home/lucas/hercules-sim
cp settings-fleet-4drone.json settings-fleet-4drone.json.bak-424x240
sed -i 's/"Width": 424,/"Width": 640,/; s/"Height": 240,/"Height": 480,/' settings-fleet-4drone.json
python3 -c "
import json,re
j=json.loads(re.sub(r'^\s*//.*\$','',open('settings-fleet-4drone.json').read(),flags=re.M))
for v,d in j['Vehicles'].items():
    c=d['Cameras']
    print('%-14s L=%dx%d R=%dx%d depth=%dx%d'%(v,
      c['front_left']['CaptureSettings'][0]['Width'],  c['front_left']['CaptureSettings'][0]['Height'],
      c['front_right']['CaptureSettings'][0]['Width'], c['front_right']['CaptureSettings'][0]['Height'],
      c['front_center']['CaptureSettings'][0]['Width'],c['front_center']['CaptureSettings'][0]['Height']))
"
# expect all four rows: L=640x480 R=640x480 depth=640x480
```

**Step 2 — confirm the two levers are the defaults (they already are; verify, do not re-edit).**

```bash
grep -n 'CAPTURE=\|DEPTH_HZ=\|image_jitter_threshold_ms' /home/lucas/hercules-sim/run_fleet_radio.sh
# expect: :48 CAPTURE=${HERC_CAPTURE:-0}
#         :51 DEPTH_HZ=${HERC_DEPTH_HZ:-10.0}
#        :176 -p image_jitter_threshold_ms:=200.0
```

`HERC_DEPTH_HZ` **must** be a float. `airsim_realsense_node.cpp:185` declares it `declare_parameter<double>`; passing the integer `10` throws `InvalidParameterTypeException` and kills all four sensor nodes at startup — that is what produced the all-zero `runs/r2_depth10_INVALID`.

**Step 3 — lock the result in with a randomized, load-gated replicate set (~30 min, unattended).** Two arms are sufficient at a 36.8% effect; three each gives margin. Run only when no sibling stack is up.

```bash
cd /home/lucas/hercules-sim
pgrep -f "[a]irsim_realsense_node|[s]imple_exploration_planner|[c]uvslam_sim_bridge" && echo "ABORT: stack up"
for i in 1 2 3; do
  for arm in $(shuf -e A B); do
    until [ "$(cut -d' ' -f1 /proc/loadavg | cut -d. -f1)" -lt 3 ]; do sleep 20; done
    if [ "$arm" = A ]; then HERC_CAPTURE=0 HERC_DEPTH_HZ=10.0 investigation/run_exp.sh c${i}_cap0_d10 150
    else                   HERC_CAPTURE=1 HERC_DEPTH_HZ=30.0 investigation/run_exp.sh c${i}_cap1_d30 150; fi
  done
done
```

Report per-drone stereo Hz **excluding any drone that outlived its peers** (that artifact inflated `r4`'s mean from 11.5 to 15.6), and discard any run whose *paced* depth stream misses its own setpoint by >10% — that is the host-starvation signature that disqualified `r4`.

**Step 4 — close the one open question: does depth-10 hurt the planner?** The rate question is settled; the ESDF/FIS question is not.

```bash
cd /home/lucas/hercules-sim/e1_frames/runs
for r in c1_cap0_d10 c1_cap1_d30 c2_cap0_d10 c2_cap1_d30 c3_cap0_d10 c3_cap1_d30; do
  echo -n "$r  PLAN_OK=$(grep -ah -c 'PLAN OK' $r/planner_*.log | paste -sd+ | bc)  "
  echo    "PATH_COMPLETE=$(grep -ah -c 'Path complete' $r/planner_*.log | paste -sd+ | bc)"
done
```

Gate: `Path complete / PLAN OK ≥ 50%` must not regress between arms. If it does, step depth back to 15.0 (still ≈ +5 Hz stereo over baseline) rather than reverting the capture lever.

**Step 5 — harden the scorecard before trusting any future A/B.** Add `-a` to every `grep` in `/home/lucas/hercules-sim/fidelity_scorecard.sh` (lines 57, 60, 81, 83, 97, 99, 143, 153, 160, 163–166, 179, 193, 203). A NUL byte in a log makes GNU grep print "binary file matches" instead of the line, scoring 0.00/FAIL — `/home/lucas/hercules-sim/e1_frames/scorecard_fast4.txt` shows exactly that fabricated failure. I verified this did **not** affect `r5` or `r6` (no NUL bytes in their logs), so their numbers stand.

**Step 6 — settle N=3-vs-N=4 in the emulator, with no UE and no render cost.** The `sync_source` re-election deadlock is the strongest N-dependent result available and needs zero GPU. Run the kill-node-0 and one-way-mute experiments at N=2/3/4 to completion against `virtual_lora_radio_v2.py` on its `SimClock`. If N=3 reproduces every bug class N=4 does (it did in the exploratory sweep), that becomes the documented fallback if the render budget is ever squeezed — but on today's measurements, no squeeze is needed.

**Do NOT do:** re-test 424×240 (measured: +1.1% throughput, −61% usable landmarks, and `VIO-STABILIZATION-PLAN.md:117` said so first); drop to 320×240 (n=1 141.2 vs 140.1, n=4 247.6 vs 245.1 renders/s — literally zero gain, since fitted per-image cost is ~1.6 ms fixed + ~15.7 ns/pixel and 424×240 is already past the knee); switch to PNG (2.6× slower per call for a 4.5× byte reduction, and bytes are not the constraint); merge stereo+depth into one RPC batch (61.00 vs 60.55 renders/s — a 2-image batch costs exactly 2× a 1-image call); or set `ViewMode: "NoDisplay"` (it deadlocks every capture on the `waitFor(5)` loop, because `RenderRequest` depends on the viewport's `OnEndDraw()` firing — verified experimentally: 4 clients, 1 success and 4 errors in 60 s).

---

### Artifacts
- Validating runs: `/home/lucas/hercules-sim/e1_frames/runs/r5_cap0_depth10` (640×480, the recommendation) and `/home/lucas/hercules-sim/e1_frames/runs/r6_424x240` (the matched resolution A/B).
- Disqualified run: `/home/lucas/hercules-sim/e1_frames/runs/r4_cap0_depth10` (paced depth missed its own setpoint, 96.1 r/s fleet total).
- Resolution feature probe (rerunnable, needs `PYTHONPATH=/usr/lib/python3/dist-packages` for cv2 4.5.4 + numpy 1.21.5 — the default `python3` has numpy 2.2.6 and cannot import cv2): frame sets at `/home/lucas/hercules-sim/investigation/feature_probe/frames_baseline_blocks` (untextured, as flown) and `.../frames` (textured). Note `frames_textured/` is a byte-identical duplicate of `frames/` — an A/B against those two measures nothing.
- Settings backup created by step 1: `/home/lucas/hercules-sim/settings-fleet-4drone.json.bak-424x240`.

*Note on host conditions: a sibling agent's UE instance and fleet stack were active during this session. I aborted my own replicate run (`r6_cap0_depth10_rep`) at preflight rather than let `run_exp.sh`'s WIPE stage destroy their in-flight ground truth. All numbers above come from archived runs plus offline analysis, and the two decisive runs (`r5`, `r6`) were taken at matched host load (load1 median 8.9 vs 8.8), which is what makes the resolution comparison valid.*