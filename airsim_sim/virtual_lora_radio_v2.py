#!/usr/bin/env python3
"""Virtual LoRa TDMA radio v2 for the HERCULES fleet sim (Run B).

Faithful emulation of N lora_mesh_v7_2_ros2 radios (Heltec Wireless Stick
Lite V3: ESP32-S3 + SX1262) plus the RF channel, per
investigation/RADIO-V2-DESIGN.md.  Owns one pty per drone; the real radiohive
lora_bridge_node opens the slave via symlink /tmp/hercules_lora/droneN, i.e.

    ros2 launch radiohive lora_bridge.launch.py \
        serial_port:=/tmp/hercules_lora/drone1

Layers (design doc section 1):
  L1 serial: 86.8 us/byte pacing, U(0.1, 1.0) ms USB jitter per transfer,
     CP2102 buffers 576 B device->host / 640 B host->device, silent overflow.
  L2 firmware serial: verbatim 5-state NO-RESCAN parser (fw:268-310), single
     -deep keep-OLDEST custom queue with the len>=2 rule (fw:323-331),
     last-write-wins pose register, real STATUS_RESP from the peer table.
  L3 TDMA/MAC: per-node millis() clocks with ppm skew and a 1 ms floor,
     verbatim v7.2 sync (shouldSyncTo / findNewSyncSource / epoch snap
     including the fixed-airtime quirk fw:573), TX window [5, 35-airtime]
     with whole-cycle miss and custom persistence, always-beacon with
     zero-init pose, 3 s peer timeout with PEER_LOST/PEER_JOINED, seq/missed
     accounting.
  L4 PHY: airtime-delayed delivery (lora_airtime_ms imported from their
     mdn_core.lora_packet -- never reimplemented), atomic pose+custom
     AirFrame with a SINGLE loss draw per receiver, half-duplex deafness +
     preamble arming + re-arm gap, collision/capture (6 dB / 3-symbol),
     geometry channel model
       RSSI(d) = 10 dBm - [31.7 + 10*n*log10(d)] - shadow - Rician fade
       SNR     = RSSI + 109.5
       PER     = 1 / (1 + exp(2.2 * (SNR + 7.5)))
     and a Gilbert-Elliott burst-loss overlay per directed link.

Wire ids are 0-BASED mesh ids (symlink droneN is mesh id N-1); custom
payloads pass through opaque (the claim src_id inside is 1-based).

Fault injection: UDP text datagrams on 127.0.0.1:$LORA_CTRL_PORT (default
47850) and/or a LORA_SCHEDULE file with lines "t=<seconds> <verb> ...".
Verbs (ids are 0-based mesh ids), design doc section 2:
  kill <id> | revive <id> | reset <id> | wedge <id> [rx|tx|both|off]
  unplug <id> [T] | eof <id> | stall <id> [T] | steal <id> <frac>
  roulette [a b] | corrupt <up|down> <rate> | garbage <id> [nbytes]
  mute_link <a> <b> [oneway] | partition <ids>|<ids> | freeze_sync <id> [0|1]
  jump <id> <ms> | drift <id> <ppm> | claim_every_k <id> <k>
  dup_claim <id> [0|1] | misconfig <id> node_id=<n> num_nodes=<m>
  interferer <duty%> <dBm> | drop <pct> [a b] | clear | stats

Boot env: LORA_N, LORA_SEED, LORA_DROP_PCT, LORA_MODE=faithful|ideal,
LORA_PPM_MAX, LORA_LOOP_JITTER_MS, LORA_GE='p_gb,p_bg,per_good,per_bad',
LORA_CHANNEL=geom|fixed, LORA_PATHLOSS_N, LORA_SHADOW_SIGMA, LORA_RICIAN_K,
LORA_NODE_CFG='mesh:NODE_ID:NUM_NODES,...', LORA_SERIAL_PACE, LORA_USB_JITTER,
LORA_CTRL_PORT, LORA_SCHEDULE.
Additions beyond the design list, all default-safe: LORA_LINK_DIR (test
isolation), LORA_RESET_ON_OPEN (gates the bridge-(re)open DTR-reset
emulation), LORA_TWORAY (design P3-17, off by default), LORA_STATS_S,
LORA_QUIET.

lora_only note (design P3-16): the legacy 32-byte full coordination packet
CANNOT ride the v7.2 piggyback at SF7/BW500 with 5 ms margins -- 38+32 = 70 B
= 32.064 ms airtime, above the 30 ms slot budget and above max_tx_size (64) --
so the firmware silently drops it whole (fw:509-521).  It does fit the raw
40 ms slot (LB:37-41) but fails the margin policy.  Claim latency is
invisible in-stack: the compact claim packet carries no timestamp
(pkt:248-293) and PeerEntry.latency_ms is never written (cl:62).

LORA_MODE=ideal reproduces v1 (virtual_lora_radio.py) semantics verbatim as a
regression escape: instant delivery, bridge-style rescanning parser, silent
until the first pose, len>=1 customs, BASE_PACKET=39 (its known off-by-one),
fake STATUS_RESP, no peer events, one shared clock.
"""
import errno
import hashlib
import heapq
import math
import os
import random
import socket
import struct
import sys
import time
import tty
from collections import deque

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'src', 'multi_drone_nvblox'))
sys.path.insert(0, '/home/lucas/hercules-sim/src/multi_drone_nvblox')
from mdn_core.lora_packet import lora_airtime_ms  # their exact airtime math

# ===================================================================
# Firmware constants (fw:42-79).  NUM_NODES / CYCLE_MS / BASE_PACKET_SIZE
# are per-node (they are build constants, and `misconfig` stages them).
# ===================================================================
SLOT_MS = 40
GUARD_MS = 30
TX_MARGIN_MS = 5
NODE_TIMEOUT_MS = 3000
STATUS_MS = 5000               # inert: printStatus body is commented out
SERIAL_START = 0xAA
SERIAL_BUF_SIZE = 220
MAX_CUSTOM_DATA = 160

# message types (fw:66-76)
T_POSE_UPDATE, T_CUSTOM_MSG, T_STATUS_REQ = 0x01, 0x02, 0x03
T_PEER_POSE, T_PEER_MSG, T_STATUS_RESP = 0x81, 0x82, 0x83
T_PEER_LOST, T_PEER_JOINED = 0x84, 0x85

# ---- serial link model (design L1) ----
BYTE_TIME_S = 86.8e-6          # 115200 8N1, one direction
DEV2HOST_CAP = 576             # CP2102 device->host buffer, silent overflow
HOST2DEV_CAP = 640             # CP2102 host->device buffer, silent overflow
USB_JITTER_MIN_S, USB_JITTER_MAX_S = 0.0001, 0.001

# ---- PHY model (design L4) ----
T_SYM_S = (2 ** 7) / 500e3     # SF7 / BW500 symbol time = 256 us
PREAMBLE_LOCK_S = 3.0 * T_SYM_S    # 3 preamble symbols = 0.768 ms
CAPTURE_DB = 6.0               # LoRaSim/Bor capture threshold
TX_POWER_DBM = 10.0            # fw:53 LORA_PWR
PL0_DB = 31.7                  # reference path loss at 1 m, 915 MHz
NOISE_OFFSET_DB = 109.5        # SNR = RSSI + 109.5 (BW500, NF 7.5)
SNR_LIMIT_DB = -7.5            # SF7 demod limit (SX1262 DS table 6-1)
PER_K = 2.2                    # PER waterfall slope, per dB
SHADOW_COHERENCE_S = 5.0       # slow shadowing redraw period (~seconds)
REARM_MIN_S, REARM_MAX_S = 0.0005, 0.002    # post-RX/TX re-arm gap
TWO_RAY_CROSS_M = 38.4         # 4*pi*h1*h2/lambda at 1 m AGL, 915 MHz
LAMBDA_M = 0.3276
ANTENNA_H_M = 1.0

# ---- ESP32 reset / boot behaviour (design L1 "open-reset") ----
BOOT_SPEW_DELAY_S = (0.100, 0.300)
BOOT_SPEW_BYTES = (200, 600)
BOOT_MUTE_S = (1.5, 3.0)

# ---- loop-latency jitter: U(0, LORA_LOOP_JITTER_MS) with rare spikes.
# A spike is what actually pushes a launch past the [5, 9] ms claim window
# (design P1-6); magnitude is an inference, so it is derived from the knob.
LOOP_SPIKE_P = 0.01
LOOP_SPIKE_MULT = 10.0

U32 = 0xFFFFFFFF
DEFAULT_LINKDIR = '/tmp/hercules_lora'
AIR_HISTORY_S = 1.0            # how long finished frames stay collidable


# ===================================================================
# Small helpers
# ===================================================================
def frame(mtype, payload=b''):
    """[0xAA][type][len][payload][xor(type,len,payload)] (fw:199-208)."""
    ln = len(payload) & 0xFF
    chk = mtype ^ ln
    for b in payload:
        chk ^= b
    return bytes([SERIAL_START, mtype & 0xFF, ln]) + bytes(payload) + \
        bytes([chk & 0xFF])


def airtime_us(size_bytes):
    """RadioLib getTimeOnAir() equivalent: integer microseconds."""
    return int(round(lora_airtime_ms(size_bytes) * 1000.0))


def airtime_int_ms(size_bytes):
    """Firmware airtime: integer-truncated us/1000 (fw:382-383, 446)."""
    return airtime_us(size_bytes) // 1000


def compute_max_tx_size(base_packet, sizeof_packet):
    """Largest on-air size whose getTimeOnAir <= 30,000 us (fw:388-393).
    At N=4 (base 38) this is 64 B, i.e. 26 custom bytes -- v1's 39-byte base
    made it 25, a live payload cliff."""
    budget_us = (SLOT_MS - 2 * TX_MARGIN_MS) * 1000
    m = base_packet
    for sz in range(base_packet, sizeof_packet + 1):
        if airtime_us(sz) <= budget_us:
            m = sz
        else:
            break
    return m


def u32(x):
    return x & U32


def to_i32(x):
    """C (int32_t) reinterpretation of a uint32."""
    return ((x & U32) ^ 0x80000000) - 0x80000000


def to_i16_trunc(f):
    """C (int16_t) cast of a float: truncate toward zero, then wrap."""
    return ((int(f) + 0x8000) & 0xFFFF) - 0x8000


def to_i8_constrain(f):
    """fw:607  rssi_of[id] = (int8_t)constrain((int)r, -128, 0)."""
    v = int(f)
    return max(-128, min(0, v))


def quant_rssi(rssi_db):
    """SX126x RSSI granularity: 0.5 dB steps."""
    return round(rssi_db * 2.0) / 2.0


def quant_snr(snr_db):
    """SX126x SnrPkt/4 quantization, capped at roughly +10 dB."""
    return min(10.0, round(snr_db * 4.0) / 4.0)


# ===================================================================
# Config (design section 2, "Boot env")
# ===================================================================
class Config(object):
    def __init__(self, env=None, **kw):
        env = dict(env if env is not None else os.environ)
        # keyword overrides are spelled like the env vars, minus LORA_
        env.update({('LORA_' + k.upper()): str(v) for k, v in kw.items()})

        def get(name, default):
            return env.get('LORA_' + name, default)

        self.n = int(get('N', '4'))
        self.seed = int(get('SEED', '7'))
        self.drop_pct = float(get('DROP_PCT', '0'))
        self.mode = get('MODE', 'faithful')
        self.ppm_max = float(get('PPM_MAX', '10'))
        self.loop_jitter_ms = float(get('LOOP_JITTER_MS', '2'))
        # Gilbert-Elliott: p_good->bad, p_bad->good, PER in good, PER in bad.
        # Default is inert (never enters bad, never loses) so the baseline sim
        # invents no failure mode; turn it on explicitly for burst studies.
        ge = [float(x) for x in get('GE', '0.0,1.0,0.0,1.0').split(',')]
        self.ge_p_gb, self.ge_p_bg, self.ge_per_good, self.ge_per_bad = ge
        self.channel = get('CHANNEL', 'geom')
        self.pathloss_n = float(get('PATHLOSS_N', '2.9'))
        self.shadow_sigma = float(get('SHADOW_SIGMA', '3'))
        self.rician_k_db = float(get('RICIAN_K', '8'))
        self.node_cfg = get('NODE_CFG', '')
        self.serial_pace = get('SERIAL_PACE', '1') not in ('0', '')
        self.usb_jitter = get('USB_JITTER', '1') not in ('0', '')
        self.ctrl_port = int(get('CTRL_PORT', '47850'))
        self.schedule = get('SCHEDULE', '')
        # additions (default-safe)
        self.linkdir = get('LINK_DIR', DEFAULT_LINKDIR)
        self.reset_on_open = get('RESET_ON_OPEN', '1') not in ('0', '')
        self.tworay = get('TWORAY', '0') not in ('0', '')
        self.stats_s = float(get('STATS_S', '10'))
        self.quiet = get('QUIET', '0') not in ('0', '')

    def node_constants(self, mesh):
        """(NODE_ID, NUM_NODES) build constants for a mesh index, honouring
        LORA_NODE_CFG='mesh:NODE_ID:NUM_NODES,...' misconfig staging."""
        for ent in self.node_cfg.split(','):
            p = ent.strip().split(':')
            if len(p) == 3 and p[0].strip().isdigit() and int(p[0]) == mesh:
                return int(p[1]), int(p[2])
        return mesh, self.n


# ===================================================================
# Deterministic keyed RNG: hash(seed, purpose, sender, rx, cycle) -> Random.
# Every draw is a pure function of its key, so results do not depend on the
# ORDER in which draws happen (design P0-1).  v1 seeded one global generator,
# which made runs irreproducible the moment scheduling wobbled.
# ===================================================================
class KeyedRng(object):
    def __init__(self, seed):
        self.seed = int(seed)

    def rng(self, purpose, *keys):
        h = hashlib.blake2b(digest_size=8)
        h.update(('%d|%s' % (self.seed, purpose)).encode())
        for k in keys:
            h.update(('|%r' % (k,)).encode())
        return random.Random(int.from_bytes(h.digest(), 'little'))

    def random(self, purpose, *keys):
        return self.rng(purpose, *keys).random()

    def uniform(self, a, b, purpose, *keys):
        return self.rng(purpose, *keys).uniform(a, b)

    def randint(self, a, b, purpose, *keys):
        return self.rng(purpose, *keys).randint(a, b)


# ===================================================================
# Clocks
# ===================================================================
class RealClock(object):
    """Wall clock, zeroed at construction."""

    def __init__(self):
        self.t0 = time.monotonic()

    def now(self):
        return time.monotonic() - self.t0


class SimClock(object):
    """Deterministic virtual clock: the driver advances it explicitly.
    Used by the validation suite so a run is bit-identical every time."""

    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class NodeClock(object):
    """One board's millis().  Fixed ppm skew from the 40 MHz ESP32 crystal
    plus the 1 ms integer floor that every firmware slot/sync/cycle read goes
    through -- all firmware timing is millis() (fw:178-186, 412-413)."""

    def __init__(self, wall0, ppm):
        self.wall0 = wall0
        self.ms0 = 0.0
        self.ppm = ppm

    def millis_f(self, wall):
        return self.ms0 + (wall - self.wall0) * 1000.0 * (1.0 + self.ppm * 1e-6)

    def millis(self, wall):
        return int(self.millis_f(wall)) & U32   # 1 ms floor + uint32 wrap

    def set_ppm(self, wall, ppm):
        self.ms0 = self.millis_f(wall)
        self.wall0 = wall
        self.ppm = ppm

    def reboot(self, wall):
        self.wall0 = wall
        self.ms0 = 0.0


# ===================================================================
# RF channel: geometry path loss + shadowing + Rician fade + PER curve,
# with a Gilbert-Elliott burst-loss overlay per directed link (design L4).
# ===================================================================
class Channel(object):
    def __init__(self, cfg, krng):
        self.cfg = cfg
        self.krng = krng
        self.ge_state = {}      # (s, r) -> 'g' | 'b'
        self.ge_bad_time = {}   # (s, r) -> seconds spent in the bad state
        self.ge_last = {}       # (s, r) -> wall of the previous GE step

    def _shadow_db(self, a, b, wall):
        """Slow log-normal shadowing, symmetric per link, redrawn every
        SHADOW_COHERENCE_S seconds (coherence ~ seconds)."""
        if self.cfg.shadow_sigma <= 0:
            return 0.0
        lo, hi = (a, b) if a < b else (b, a)
        epoch = int(wall // SHADOW_COHERENCE_S)
        return self.krng.rng('shadow', lo, hi, epoch).gauss(
            0.0, self.cfg.shadow_sigma)

    def _fade_db(self, key):
        """Per-packet Rician fade, K = LORA_RICIAN_K dB."""
        k_lin = 10.0 ** (self.cfg.rician_k_db / 10.0)
        g = self.krng.rng('fade', *key)
        s = math.sqrt(2.0 * (k_lin + 1.0))
        x = math.sqrt(k_lin / (k_lin + 1.0)) + g.gauss(0.0, 1.0) / s
        y = g.gauss(0.0, 1.0) / s
        return -10.0 * math.log10(max(x * x + y * y, 1e-12))

    def rssi_snr(self, tx_pose, rx_pose, key, wall, a, b):
        """(rssi_dbm, snr_db) for one packet on link a->b.  Poses are the raw
        16-byte POSE_UPDATE registers (team-frame x/y/z/yaw floats)."""
        if self.cfg.channel == 'fixed':
            return -40.0, 8.0
        tx = struct.unpack('<fff', bytes(tx_pose[:12]))
        rx = struct.unpack('<fff', bytes(rx_pose[:12]))
        d = math.sqrt(sum((p - q) ** 2 for p, q in zip(tx, rx)))
        d = max(d, 1.0)
        pl = PL0_DB + 10.0 * self.cfg.pathloss_n * math.log10(d)
        if self.cfg.tworay and d > TWO_RAY_CROSS_M:
            # optional two-ray null dips beyond the crossover (design P3-17)
            f = 2.0 * abs(math.sin(2.0 * math.pi * ANTENNA_H_M * ANTENNA_H_M
                                   / (LAMBDA_M * d)))
            pl += min(40.0, -20.0 * math.log10(max(f, 1e-4)))
        rssi = TX_POWER_DBM - pl - self._shadow_db(a, b, wall) - \
            self._fade_db(key)
        return rssi, rssi + NOISE_OFFSET_DB

    @staticmethod
    def per(snr_db):
        """PER(SNR) = 1 / (1 + exp(2.2 * (SNR + 7.5))); 0.5 at the -7.5 dB
        SF7 demod limit.  At 10-100 m the margin is 25-60 dB, so this is
        effectively inert at deployment ranges -- by design."""
        arg = PER_K * (snr_db - SNR_LIMIT_DB)
        if arg > 40.0:
            return 0.0
        if arg < -40.0:
            return 1.0
        return 1.0 / (1.0 + math.exp(arg))

    def ge_step(self, s, r, key, wall):
        """Advance the directed link's Gilbert-Elliott chain by one packet.
        Returns True when this packet is lost to the burst process.  Called
        once per packet per receiver regardless of other loss reasons, so the
        chain evolves deterministically."""
        pair = (s, r)
        st = self.ge_state.get(pair, 'g')
        last = self.ge_last.get(pair)
        if last is not None and st == 'b':
            self.ge_bad_time[pair] = \
                self.ge_bad_time.get(pair, 0.0) + (wall - last)
        self.ge_last[pair] = wall
        g = self.krng.rng('ge', *key)
        lost = g.random() < (self.cfg.ge_per_good if st == 'g'
                             else self.cfg.ge_per_bad)
        if st == 'g':
            if g.random() < self.cfg.ge_p_gb:
                st = 'b'
        else:
            if g.random() < self.cfg.ge_p_bg:
                st = 'g'
        self.ge_state[pair] = st
        return lost


# ===================================================================
# One on-air packet: pose + optional custom, ATOMIC.  A single loss draw per
# receiver decides both; 0x81 and 0x82 are delivered together or not at all
# (design P0-3).  v1 rolled twice and could deliver a claim with no pose.
# ===================================================================
class AirFrame(object):
    __slots__ = ('sender', 'node_id', 'seq', 'uptime', 'cycle_clock',
                 'sync_source', 'rssi_of', 'neighbor_mask', 'pose',
                 'has_custom', 'custom_dest', 'custom_len', 'custom_data',
                 'size', 'start', 'end', 'key', 'rssi_at', 'snr_at',
                 'armed_at', 'rf_at', 'dup')

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k))
        self.rssi_at = {}       # per receiver, sampled when the preamble lands
        self.snr_at = {}
        self.armed_at = {}      # receiver was listening in time
        self.rf_at = {}         # receiver's radio was powered/unmuted


# ===================================================================
# One direction of the CP2102 + UART byte path (design L1 / P2-11).
# ===================================================================
class SerialPipe(object):
    """Capped buffer with SILENT overflow (no flow control anywhere:
    CRTSCTS is cleared by the bridge, br:117-118), 86.8 us/byte wire pacing
    and one U(0.1, 1.0) ms USB latency quantum per transfer/burst."""

    def __init__(self, cap, cfg, krng, tag):
        self.cap = cap
        self.cfg = cfg
        self.krng = krng
        self.tag = tag
        self.q = bytearray()
        self.next_t = 0.0       # wall time the head byte finishes clocking out
        self.overflow = 0
        self.bursts = 0

    def push(self, data, wall):
        if not data:
            return
        free = self.cap - len(self.q)
        if free <= 0:
            self.overflow += len(data)
            return
        if len(data) > free:
            self.overflow += len(data) - free   # silent mid-frame truncation
            data = data[:free]
        if not self.q:
            jitter = 0.0
            if self.cfg.usb_jitter:
                jitter = self.krng.uniform(USB_JITTER_MIN_S, USB_JITTER_MAX_S,
                                           'usb', self.tag, self.bursts)
            self.next_t = max(self.next_t, wall) + jitter
            self.bursts += 1
        self.q.extend(data)

    def drain(self, wall):
        """Bytes whose wire time has elapsed."""
        if not self.q:
            return b''
        if not self.cfg.serial_pace:
            out = bytes(self.q)
            del self.q[:]
            return out
        if wall < self.next_t:
            return b''
        n = min(int((wall - self.next_t) / BYTE_TIME_S) + 1, len(self.q))
        out = bytes(self.q[:n])
        del self.q[:n]
        self.next_t += n * BYTE_TIME_S
        return out

    def clear(self):
        del self.q[:]


# ===================================================================
# Fault state (design section 2).  Vocabulary is a superset of
# mdn_core/fault_logic.py so behaviour-level and phy-level injection can
# be A/B'd with the same words.
# ===================================================================
class Faults(object):
    def __init__(self):
        self.clear()

    def clear(self):
        self.killed = set()
        self.wedged = {}          # id -> 'rx' | 'tx' | 'both'
        self.stalled = {}         # id -> wall time the stall ends (or inf)
        self.steal = {}           # id -> fraction of device->host bytes eaten
        self.corrupt_up = 0.0     # per-byte bit-flip rate, bridge -> radio
        self.corrupt_down = 0.0   # per-byte bit-flip rate, radio -> bridge
        self.muted = set()        # directed (a, b) pairs
        self.partitions = []      # list of id groups (fault_logic grammar)
        self.frozen_sync = set()
        self.claim_every_k = {}
        self.dup_claim = set()
        self.link_drop = {}       # (a, b) -> percent
        self.interferer = None    # (duty_fraction, dbm)

    # ---- link reachability -------------------------------------------------
    def partitioned(self, a, b):
        """Strict fault_logic semantics: with partitions declared, ids in
        different groups (or in no group) cannot hear each other."""
        if not self.partitions:
            return False
        ga = gb = None
        for i, grp in enumerate(self.partitions):
            if a in grp:
                ga = i
            if b in grp:
                gb = i
        if ga is None or gb is None:
            return True
        return ga != gb

    def link_blocked(self, a, b):
        """True when nothing sent by a can reach b."""
        if a in self.killed or b in self.killed:
            return True
        wa, wb = self.wedged.get(a), self.wedged.get(b)
        if wa in ('tx', 'both') or wb in ('rx', 'both'):
            return True
        if (a, b) in self.muted:
            return True
        return self.partitioned(a, b)


# ===================================================================
# One emulated radio: pty + CP2102 + UART + firmware + SX1262.
# ===================================================================
class VNode(object):
    def __init__(self, emu, mesh):
        self.emu = emu
        self.cfg = emu.cfg
        self.krng = emu.krng
        self.mesh = mesh                      # index used for links/geometry
        self.link = os.path.join(self.cfg.linkdir, 'drone%d' % (mesh + 1))
        self.mfd = None
        self.pts = None
        self.bridge_attached = False
        self.open_pty()

        ppm = self.krng.uniform(-self.cfg.ppm_max, self.cfg.ppm_max,
                                'ppm', mesh)
        self.clock = NodeClock(emu.clock.now(), ppm)
        self.to_bridge = SerialPipe(DEV2HOST_CAP, self.cfg, self.krng,
                                    ('d2h', mesh))
        self.from_bridge = SerialPipe(HOST2DEV_CAP, self.cfg, self.krng,
                                      ('h2d', mesh))
        self.tx_intervals = deque()           # (start, end) for deafness
        # (time_in_slot, this_airtime) of recent launches -- the trace the
        # window-miss / quirk-1 sawtooth analysis reads (design section 4.7)
        self.launch_offsets = deque(maxlen=512)
        self.loop_count = 0
        self.next_loop_wall = 0.0
        self.tx_busy_until = -1.0             # radio.transmit() blocks loop()
        self.mute_until = -1.0                # post-reset radio mute
        self.mcu_down_until = -1.0            # post-reset serial silence
        self.counters = {}
        self.setup(emu.clock.now())

    # ---- pty bootstrap (v1-compatible, emu:56-64) --------------------------
    def open_pty(self):
        mfd, sfd = os.openpty()
        tty.setraw(sfd)
        os.set_blocking(mfd, False)
        self.pts = os.ttyname(sfd)
        # Closing our slave fd makes reads on the master return EIO while no
        # bridge holds the slave open -- that EIO<->EAGAIN transition is how
        # we detect a bridge (re)open and fire the DTR reset (design P2-13).
        os.close(sfd)
        self.mfd = mfd
        if os.path.islink(self.link) or os.path.exists(self.link):
            os.unlink(self.link)
        os.symlink(self.pts, self.link)

    def close_pty(self, drop_link=True):
        if self.mfd is not None:
            try:
                os.close(self.mfd)
            except OSError:
                pass
            self.mfd = None
        if drop_link and (os.path.islink(self.link) or os.path.exists(self.link)):
            try:
                os.unlink(self.link)
            except OSError:
                pass

    # ---- firmware setup() (fw:365-407) ------------------------------------
    def setup(self, wall):
        self.node_id, self.num_nodes = self.cfg.node_constants(self.mesh)
        self.cycle_ms = self.num_nodes * SLOT_MS + GUARD_MS
        self.sizeof_packet = 34 + self.num_nodes + MAX_CUSTOM_DATA
        self.base_packet = self.sizeof_packet - MAX_CUSTOM_DATA   # 34 + N
        self.packet_airtime_ms = airtime_int_ms(self.base_packet) or 6
        self.max_tx_size = compute_max_tx_size(self.base_packet,
                                               self.sizeof_packet)

        # peer table (fw:111-121); index is the wire node id
        self.nodes = [dict(last_seq=0, last_seen=0, rssi=0, snr=0.0,
                           rx_count=0, missed=0, alive=False, sync_source=i,
                           pose=bytes(16)) for i in range(self.num_nodes)]
        self.rssi_of = [0] * self.num_nodes
        if self.node_id < self.num_nodes:
            self.nodes[self.node_id]['alive'] = True

        self.my_seq = 0
        self.my_pose = bytes(16)              # zero-init: always-beacon
        self.custom_pending = False
        self.custom_dest = 0xFF
        self.custom_len = 0
        self.custom_data = b''

        self.cycle_epoch = self.clock.millis(wall)
        self.synced = True
        self.my_sync_source = self.node_id
        self.tx_done_this_cycle = False
        self.last_cycle_num = 0xFFFFFFFF
        self.miss_cycle = None        # bookkeeping only, not firmware state
        # last loop() instant INSIDE this cycle's own slot (A7); None until the
        # slot is entered, cleared at every cycle boundary. Purely a
        # poll-granularity compensation -- never firmware state.
        self._last_tx_poll = None

        self.ser_state = 0
        self.ser_type = 0
        self.ser_len = 0
        self.ser_idx = 0
        self.ser_buf = bytearray(SERIAL_BUF_SIZE)
        self.ser_checksum = 0

        self.in_rx = True
        self.rx_armed_since = wall
        for k in ('tx', 'tx_custom', 'window_miss', 'queue_drop',
                  'air_drop_custom', 'rx_ok', 'peer_lost', 'peer_joined',
                  'ser_bad_chk', 'ser_overflow'):
            self.counters.setdefault(k, 0)

    def reboot(self, wall, spew=True):
        """MCU reset: firmware state, clock and pose registers all restart."""
        self.clock.reboot(wall)
        self.setup(wall)
        self.to_bridge.clear()
        self.from_bridge.clear()
        r = self.krng.rng('reset', self.mesh, int(wall * 1000))
        self.mcu_down_until = wall + r.uniform(*BOOT_SPEW_DELAY_S)
        self.mute_until = wall + r.uniform(*BOOT_MUTE_S)
        if spew:
            n = r.randint(*BOOT_SPEW_BYTES)
            blob = bytes(r.randrange(256) for _ in range(n))
            self.emu.at(self.mcu_down_until,
                        lambda w, b=blob: self.to_bridge.push(b, w))

    # ---- radio availability ------------------------------------------------
    def radio_off(self, wall):
        """`kill` and the post-reset mute stop the radio.  Losing the serial
        link does NOT: a board whose bridge died keeps beaconing its last pose
        register (the zombie beacon that prevents peers' 3 s expiry)."""
        return (self.mesh in self.emu.faults.killed
                or wall < self.mute_until)

    # ---- serial: firmware -> ROS2 -----------------------------------------
    def emit(self, mtype, payload, wall):
        before = self.to_bridge.overflow
        self.to_bridge.push(frame(mtype, payload), wall)
        self.counters['ser_overflow'] += self.to_bridge.overflow - before

    def send_peer_pose(self, i, wall):
        n = self.nodes[i]
        self.emit(T_PEER_POSE,
                  bytes([i]) + n['pose'] +
                  struct.pack('<hf', n['rssi'], n['snr']), wall)

    def send_peer_msg(self, src, data, wall):
        self.emit(T_PEER_MSG, bytes([src]) + bytes(data), wall)

    def send_peer_joined(self, i, rssi, snr, wall):
        self.emit(T_PEER_JOINED, struct.pack('<Bhf', i, to_i16_trunc(rssi),
                                             snr), wall)
        self.counters['peer_joined'] += 1

    def send_peer_lost(self, i, wall):
        self.emit(T_PEER_LOST, bytes([i]), wall)
        self.counters['peer_lost'] += 1

    def send_status_resp(self, wall):
        """Real table-driven STATUS_RESP (fw:245-263); v1 faked it."""
        buf = bytearray([self.node_id & 0xFF, 0])
        count = 0
        for i in range(self.num_nodes):
            if i == self.node_id:
                continue
            if self.nodes[i]['alive']:
                buf += struct.pack('<Bhf', i, self.nodes[i]['rssi'],
                                   self.nodes[i]['snr'])
                count += 1
        buf[1] = count
        self.emit(T_STATUS_RESP, bytes(buf), wall)

    # ---- serial: ROS2 -> firmware (fw:268-337) -----------------------------
    def process_serial_byte(self, b, wall):
        """Verbatim 5-state machine.  NOTE the absence of any rescan: after
        0xAA the firmware blindly consumes type+len+payload+checksum, so a
        single corrupted len byte swallows up to 255 following bytes -- good
        frames included.  v1 wrongly used the bridge's advance-1-byte resync
        here and therefore under-dropped once corruption existed."""
        st = self.ser_state
        if st == 0:
            if b == SERIAL_START:
                self.ser_state = 1
        elif st == 1:
            self.ser_type = b
            self.ser_checksum = b
            self.ser_state = 2
        elif st == 2:
            self.ser_len = b
            self.ser_checksum ^= b
            self.ser_idx = 0
            self.ser_state = 4 if b == 0 else 3
        elif st == 3:
            # fw writes past ser_buf[220] for len > 220 (memory corruption we
            # deliberately do not fabricate, design section 5); clamp instead.
            if self.ser_idx < SERIAL_BUF_SIZE:
                self.ser_buf[self.ser_idx] = b
            self.ser_idx += 1
            self.ser_checksum ^= b
            if self.ser_idx >= self.ser_len:
                self.ser_state = 4
        elif st == 4:
            if b == self.ser_checksum:
                n = min(self.ser_len, SERIAL_BUF_SIZE)
                self.handle_serial_message(self.ser_type,
                                           bytes(self.ser_buf[:n]), wall)
            else:
                self.counters['ser_bad_chk'] += 1
            self.ser_state = 0
        else:
            self.ser_state = 0

    def handle_serial_message(self, mtype, payload, wall):
        if mtype == T_POSE_UPDATE:
            if len(payload) >= 16:
                self.my_pose = payload[:16]          # last-write-wins register
        elif mtype == T_CUSTOM_MSG:
            # fw:324  len >= 2 AND !custom_pending.  A dest-only frame (len 1)
            # is IGNORED -- v1 queued it.
            if len(payload) >= 2:
                if not self.custom_pending:
                    self.custom_dest = payload[0]
                    data = payload[1:]
                    if len(data) > MAX_CUSTOM_DATA:
                        data = data[:MAX_CUSTOM_DATA]
                    self.custom_len = len(data)
                    self.custom_data = data
                    self.custom_pending = True
                else:
                    # single-deep, keep-OLDEST: silently dropped, no NACK
                    self.counters['queue_drop'] += 1
        elif mtype == T_STATUS_REQ:
            self.send_status_resp(wall)

    # ---- TDMA helpers (fw:178-186) ----------------------------------------
    def cycle_clock(self, now):
        return u32(now - self.cycle_epoch)

    def cycle_elapsed(self, now):
        return self.cycle_clock(now) % self.cycle_ms

    def cycle_num(self, now):
        return self.cycle_clock(now) // self.cycle_ms

    def current_slot(self, now):
        e = self.cycle_elapsed(now)
        if e >= self.num_nodes * SLOT_MS:
            return -1
        return e // SLOT_MS

    def neighbor_mask(self):
        m = 0
        for i in range(min(self.num_nodes, 8)):
            if self.nodes[i]['alive']:
                m |= (1 << i)
        return m

    # ---- sync (fw:342-360) -------------------------------------------------
    def find_new_sync_source(self):
        best = self.node_id
        for i in range(self.num_nodes):
            if i == self.node_id or not self.nodes[i]['alive']:
                continue
            s = self.nodes[i]['sync_source']
            if s < best:
                best = s
            if i < best:
                best = i
        self.my_sync_source = best

    def should_sync_to(self, sender_id, sender_sync):
        auth = sender_id if sender_id < sender_sync else sender_sync
        if auth < self.my_sync_source:
            return True
        if auth == self.my_sync_source and sender_id <= self.my_sync_source:
            return True
        return False

    # ---- main loop (fw:412-481) -------------------------------------------
    def loop(self, wall):
        if wall < self.mcu_down_until:
            return                       # MCU in reset: no serial, no radio
        if wall < self.tx_busy_until:
            return                       # radio.transmit() is blocking
        now = self.clock.millis(wall)

        # ---- SERIAL RX FROM ROS2 ----
        chunk = self.from_bridge.drain(wall)
        for b in chunk:
            self.process_serial_byte(b, wall)

        # ---- TDMA TX ----  (RX is event-driven; see Emulator._air_end)
        if self.synced and not self.radio_off(wall):
            cn = self.cycle_num(now)
            if cn != self.last_cycle_num:
                self.last_cycle_num = cn
                self.tx_done_this_cycle = False
                # A7: the poll-granularity compensation below is SLOT-LOCAL.
                # Clearing it at the cycle boundary is what makes the launch
                # window observable at all -- see the long comment there.
                self._last_tx_poll = None
            slot = self.current_slot(now)
            if slot == self.node_id and not self.tx_done_this_cycle:
                slot_start = u32(self.cycle_epoch
                                 + (self.cycle_clock(now) // self.cycle_ms)
                                 * self.cycle_ms
                                 + self.node_id * SLOT_MS)
                time_in_slot = u32(now - slot_start)
                # airtime of the packet we are ACTUALLY about to send: a
                # pending custom shrinks the launch window (fw:441-448)
                this_airtime = self.packet_airtime_ms
                attach = (self.custom_pending and self.custom_len > 0 and
                          self.base_packet + self.custom_len <= self.max_tx_size)
                if attach:
                    this_airtime = airtime_int_ms(
                        self.base_packet + self.custom_len) \
                        or self.packet_airtime_ms
                # The real ESP32 loop() runs at ~kHz, so it samples the
                # launch window many times; only a genuine loop stall makes it
                # miss (that is what loop-jitter models).  Our Python poll is
                # far coarser, so treat the window as HIT if it fell anywhere
                # inside the interval this poll covered, and launch at the
                # earliest legal instant (fw:451-453 semantics, not our poll
                # granularity).  Without this the emulator misses a 4 ms
                # claim window ~100% of the time -- a sim artifact.
                #
                # A7 (COORDINATION-REDESIGN.md): that compensation must be
                # SLOT-LOCAL.  _last_tx_poll is only ever written here, inside
                # the node's own-slot branch, so if it is allowed to carry
                # across cycles the first poll of each slot back-dates itself
                # by a whole cycle (prev_in_slot ~ time_in_slot - 190) and
                # window_covered is unconditionally true for every airtime the
                # 26 B max_tx_size gate admits -- window_miss then becomes
                # structurally unreachable (measured: 0 misses / 3790
                # transmits) and payload-size risk is not modelled at all.
                # With the cycle-boundary reset above, the first in-slot poll
                # covers only itself, so a node whose loop stalls past win_hi
                # genuinely misses -- and win_hi shrinks with airtime, which is
                # exactly the size risk the emulator has to reproduce
                # (26 B: win_hi = 6 ms; 12 B: win_hi = 11 ms).
                win_lo = TX_MARGIN_MS
                win_hi = SLOT_MS - TX_MARGIN_MS - this_airtime
                if self._last_tx_poll is None:
                    prev_in_slot = time_in_slot   # first loop() in this slot
                else:
                    prev_in_slot = time_in_slot - max(
                        int(now - self._last_tx_poll), 0)
                self._last_tx_poll = now
                window_covered = (win_hi >= win_lo and
                                  prev_in_slot <= win_hi and
                                  time_in_slot >= win_lo)
                if window_covered:
                    time_in_slot = max(win_lo, min(time_in_slot, win_hi))
                if (time_in_slot >= TX_MARGIN_MS and
                        time_in_slot + this_airtime <= SLOT_MS - TX_MARGIN_MS):
                    self.in_rx = False
                    # trace for the launch-offset / quirk-1 sawtooth analysis
                    self.launch_offsets.append((time_in_slot, this_airtime))
                    self.do_tx(wall, now)
                    self.tx_done_this_cycle = True
                elif (time_in_slot + this_airtime > SLOT_MS - TX_MARGIN_MS
                        and self.miss_cycle != cn):
                    # Too late to fit: there is NO pose-only fallback, so the
                    # node stays silent for the WHOLE cycle (pose AND claim)
                    # and the pending custom persists.  The firmware simply
                    # keeps failing this test for the rest of the slot -- we
                    # do not touch tx_done_this_cycle, only count it once.
                    self.miss_cycle = cn
                    self.counters['window_miss'] += 1
                    self.emu.log('node%d window miss (slot+%d ms, '
                                 'airtime %d ms)%s' %
                                 (self.mesh, time_in_slot, this_airtime,
                                  ' [claim pending]' if attach else ''))

        # ---- TIMEOUTS (fw:469-480) ----
        for i in range(self.num_nodes):
            if i == self.node_id:
                continue
            n = self.nodes[i]
            if n['alive'] and n['last_seen'] > 0:
                age = u32(now - n['last_seen'])
                if age > NODE_TIMEOUT_MS and age < 0x80000000:
                    n['alive'] = False
                    self.send_peer_lost(i, wall)
                    self.emu.log('node%d PEER_LOST %d (age %d ms)'
                                 % (self.mesh, i, age))
                    if self.my_sync_source == i:
                        self.find_new_sync_source()

    # ---- transmit (fw:486-539) --------------------------------------------
    def do_tx(self, wall, now):
        tx_custom_len = 0
        has_custom = 0
        dest = 0
        data = b''
        k = self.emu.faults.claim_every_k.get(self.mesh)
        hold_for_k = bool(k) and (self.cycle_num(now) % k) != 0
        if self.custom_pending and not hold_for_k:
            if (self.custom_len > 0 and
                    self.base_packet + self.custom_len <= self.max_tx_size):
                tx_custom_len = self.custom_len
                has_custom = 1
                dest = self.custom_dest
                data = self.custom_data
            else:
                # too big for a slot: dropped WHOLE, never truncated
                self.counters['air_drop_custom'] += 1
            self.custom_pending = False       # cleared either way (fw:522)

        txlen = self.base_packet + tx_custom_len
        fr = AirFrame(sender=self.mesh, node_id=self.node_id, seq=self.my_seq,
                      uptime=now, cycle_clock=self.cycle_clock(now),
                      sync_source=self.my_sync_source,
                      rssi_of=tuple(self.rssi_of),
                      neighbor_mask=self.neighbor_mask(), pose=self.my_pose,
                      has_custom=has_custom, custom_dest=dest,
                      custom_len=tx_custom_len, custom_data=data, size=txlen,
                      start=wall, end=wall + lora_airtime_ms(txlen) / 1000.0,
                      key=(self.mesh, self.my_seq),
                      dup=self.mesh in self.emu.faults.dup_claim)
        self.my_seq = u32(self.my_seq + 1)    # increments even on lost TXs
        self.counters['tx'] += 1
        if has_custom:
            self.counters['tx_custom'] += 1

        # half-duplex: blocking transmit, deaf for the whole airtime
        self.tx_busy_until = fr.end
        self.tx_intervals.append((fr.start, fr.end))
        while self.tx_intervals and self.tx_intervals[0][1] < wall - AIR_HISTORY_S:
            self.tx_intervals.popleft()
        self.go_rx(fr.end)                    # goRx() after transmit
        self.emu.launch(fr)

    def go_rx(self, wall):
        """startReceive() plus the 0.5-2 ms re-arm gap."""
        self.in_rx = True
        self.rx_armed_since = wall + self.krng.uniform(
            REARM_MIN_S, REARM_MAX_S, 'rearm', self.mesh, self.loop_count)

    def deaf_during(self, a, b):
        for s, e in self.tx_intervals:
            if s < b and e > a:
                return True
        return False

    # ---- receive (fw:544-626) ---------------------------------------------
    def handle_rx(self, fr, wall, rssi, snr):
        # PHY length gate then the exact-length gate: a mixed NUM_NODES build
        # rejects 100% of the other build's pose-only packets (37 != 38).
        if fr.size < self.base_packet or fr.size > self.sizeof_packet:
            return
        if fr.node_id >= self.num_nodes or fr.node_id == self.node_id:
            return
        if fr.has_custom:
            if fr.custom_len > MAX_CUSTOM_DATA:
                return
            if fr.size != self.base_packet + fr.custom_len:
                return
        else:
            if fr.size != self.base_packet:
                return

        now = self.clock.millis(wall)
        i = fr.node_id
        n = self.nodes[i]
        self.counters['rx_ok'] += 1

        # ---- CLOCK SYNC ----
        if (self.should_sync_to(i, fr.sync_source)
                and self.mesh not in self.emu.faults.frozen_sync):
            sender_pos = i * SLOT_MS + TX_MARGIN_MS
            # QUIRK 1 (fw:573): resync always subtracts the BASE airtime, even
            # for a claim-laden 57 B packet (26.9 ms) -- a fixed ~+6.9 ms epoch
            # error injected into every follower on every claim cycle.
            tx_moment = u32(now - self.packet_airtime_ms)
            # QUIRK 2 (fw:572): the receiver assumes the sender launched at
            # exactly +5 ms into its slot; the real launch is anywhere in the
            # window, so the residual error rides on the sender's loop jitter.
            sender_cycle_num = fr.cycle_clock // self.cycle_ms
            new_epoch = u32(tx_moment - sender_pos
                            - sender_cycle_num * self.cycle_ms)
            correction = to_i32(u32(new_epoch - self.cycle_epoch))
            self.cycle_epoch = new_epoch
            if correction > SLOT_MS // 2 or correction < -(SLOT_MS // 2):
                self.tx_done_this_cycle = False       # legal double-TX
            self.my_sync_source = min(fr.sync_source, i)
            if self.cycle_elapsed(now) > (self.node_id + 1) * SLOT_MS:
                self.tx_done_this_cycle = True        # own slot skipped

        # ---- UPDATE STATE ----
        if n['rx_count'] > 0 and fr.seq > n['last_seq'] + 1:
            n['missed'] += fr.seq - n['last_seq'] - 1
        n['last_seq'] = fr.seq
        n['last_seen'] = now
        n['rssi'] = to_i16_trunc(rssi)
        n['snr'] = snr
        n['rx_count'] += 1
        n['sync_source'] = fr.sync_source
        n['pose'] = fr.pose
        self.rssi_of[i] = to_i8_constrain(rssi)

        was_alive = n['alive']
        n['alive'] = True

        # emission order: JOINED -> POSE -> MSG (fw:609-625)
        if not was_alive:
            self.send_peer_joined(i, rssi, snr, wall)
            self.emu.log('node%d PEER_JOINED %d' % (self.mesh, i))
            self.find_new_sync_source()
        self.send_peer_pose(i, wall)
        if fr.has_custom and fr.custom_len > 0:
            if fr.custom_dest == self.node_id or fr.custom_dest == 0xFF:
                self.send_peer_msg(i, fr.custom_data, wall)
                if fr.dup:
                    self.send_peer_msg(i, fr.custom_data, wall)

        self.in_rx = False
        self.go_rx(wall)

    # ---- host I/O ----------------------------------------------------------
    def pump_pty(self, wall):
        if self.mfd is None:
            return
        # bridge -> radio.  `stall` = the emulator stops reading its pty, so
        # the bridge's unchecked nonblocking write eventually short-writes /
        # raises: a silent uplink blackhole while the downlink keeps running.
        stall_until = self.emu.faults.stalled.get(self.mesh)
        stalled = stall_until is not None and wall < stall_until
        if not stalled:
            try:
                data = os.read(self.mfd, 4096)
                if not self.bridge_attached:
                    self.on_bridge_attach(wall)
                if data:
                    data = self.emu.corrupt(data, self.emu.faults.corrupt_up,
                                            ('cup', self.mesh))
                    before = self.from_bridge.overflow
                    self.from_bridge.push(data, wall)
                    self.counters['ser_overflow'] += \
                        self.from_bridge.overflow - before
            except BlockingIOError:
                if not self.bridge_attached:
                    self.on_bridge_attach(wall)
            except OSError as e:
                if e.errno == errno.EIO:
                    if self.bridge_attached:
                        self.bridge_attached = False
                        self.emu.log('node%d bridge detached' % self.mesh)

        # radio -> bridge
        out = self.to_bridge.drain(wall)
        if out:
            frac = self.emu.faults.steal.get(self.mesh, 0.0)
            if frac > 0:
                keep = bytearray()
                for k, b in enumerate(out):
                    if self.krng.random('steal', self.mesh,
                                        self.to_bridge.bursts, k) >= frac:
                        keep.append(b)
                out = bytes(keep)
            out = self.emu.corrupt(out, self.emu.faults.corrupt_down,
                                   ('cdn', self.mesh))
            try:
                os.write(self.mfd, out)
            except OSError:
                pass                      # no reader / full: bytes are lost

    def on_bridge_attach(self, wall):
        self.bridge_attached = True
        self.emu.log('node%d bridge attached (%s)' % (self.mesh, self.pts))
        if self.cfg.reset_on_open:
            # DTR/RTS auto-reset of the ESP32 on (re)open (design L1)
            self.reboot(wall)


# ===================================================================
# The emulator: N nodes + RF channel + fault API + event scheduler.
# ===================================================================
class Emulator(object):
    def __init__(self, cfg=None, clock=None):
        self.cfg = cfg or Config()
        self.clock = clock or RealClock()
        self.krng = KeyedRng(self.cfg.seed)
        self.channel = Channel(self.cfg, self.krng)
        self.faults = Faults()
        self.events = []                   # (time, seq, fn) kept sorted
        self.event_seq = 0
        self.air = []                      # recent AirFrames (collision set)
        self.stats = dict(delivered=0, lost_unarmed=0, lost_deaf=0,
                          lost_blocked=0, lost_collision=0, lost_ge=0,
                          lost_per=0, lost_drop=0, lost_interferer=0)
        self.link_stats = {}
        self.log_lines = []
        os.makedirs(self.cfg.linkdir, exist_ok=True)
        self.nodes = [VNode(self, i) for i in range(self.cfg.n)]
        self.ctrl = None
        if self.cfg.ctrl_port > 0:
            try:
                self.ctrl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.ctrl.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.ctrl.bind(('127.0.0.1', self.cfg.ctrl_port))
                self.ctrl.setblocking(False)
            except OSError as e:
                self.log('ctrl socket unavailable: %s' % e)
                self.ctrl = None
        self.sched = self.load_schedule(self.cfg.schedule)
        self.last_stats = 0.0

    # ---- infrastructure ----------------------------------------------------
    def log(self, msg):
        line = '[%9.3f] %s' % (self.clock.now(), msg)
        self.log_lines.append(line)
        if len(self.log_lines) > 4000:
            del self.log_lines[:2000]
        if not self.cfg.quiet:
            print('radio: ' + line, flush=True)

    def at(self, when, fn):
        """Schedule fn(wall) at `when`; ties break on insertion order."""
        self.event_seq += 1
        heapq.heappush(self.events, (when, self.event_seq, fn))

    def load_schedule(self, path):
        out = []
        if not path or not os.path.exists(path):
            return out
        with open(path) as f:
            for raw in f:
                line = raw.split('#', 1)[0].strip()
                if not line:
                    continue
                parts = line.split(None, 1)
                if not parts[0].startswith('t='):
                    continue
                try:
                    t = float(parts[0][2:])
                except ValueError:
                    continue
                if len(parts) > 1:
                    out.append((t, parts[1].strip()))
        out.sort(key=lambda e: e[0])
        return out

    def corrupt(self, data, rate, tag):
        """Per-byte single-bit-flip injector."""
        if rate <= 0 or not data:
            return data
        out = bytearray(data)
        for i in range(len(out)):
            r = self.krng.rng('corrupt', tag, self.event_seq, i)
            if r.random() < rate:
                out[i] ^= 1 << r.randrange(8)
        return bytes(out)

    # ---- air interface -----------------------------------------------------
    def launch(self, fr):
        self.air.append(fr)
        self.at(fr.start, lambda w, f=fr: self._air_start(f))
        self.at(fr.end, lambda w, f=fr: self._air_end(f))

    def _air_start(self, fr):
        """Snapshot per-receiver arming and signal strength at the instant the
        preamble hits: 'armed within the first 3 preamble symbols'."""
        sender = self.nodes[fr.sender]
        for r in self.nodes:
            if r is sender:
                continue
            rssi, snr = self.channel.rssi_snr(
                sender.my_pose, r.my_pose, ('sig', fr.sender, r.mesh, fr.seq),
                fr.start, fr.sender, r.mesh)
            fr.rssi_at[r.mesh] = rssi
            fr.snr_at[r.mesh] = snr
            fr.rf_at[r.mesh] = not r.radio_off(fr.start)
            fr.armed_at[r.mesh] = r.rx_armed_since <= fr.start + PREAMBLE_LOCK_S

    def _air_end(self, fr):
        sender = self.nodes[fr.sender]
        for r in self.nodes:
            if r is sender:
                continue
            key = (fr.sender, r.mesh, fr.seq)
            # GE advances once per packet per link no matter what else happens
            ge_lost = self.channel.ge_step(fr.sender, r.mesh, key, fr.end)
            reason = None
            if (self.faults.link_blocked(fr.sender, r.mesh)
                    or not fr.rf_at.get(r.mesh, False)):
                reason = 'lost_blocked'
            elif not fr.armed_at.get(r.mesh, False):
                reason = 'lost_unarmed'
            elif r.deaf_during(fr.start, fr.end):
                reason = 'lost_deaf'
            elif not self._captures(fr, r):
                reason = 'lost_collision'
            elif self._interferer_hit(fr, r):
                reason = 'lost_interferer'
            elif self._uniform_drop(fr, r):
                reason = 'lost_drop'
            elif ge_lost:
                reason = 'lost_ge'
            elif self.krng.random('per', *key) < Channel.per(
                    fr.snr_at.get(r.mesh, 100.0)):
                reason = 'lost_per'
            ls = self.link_stats.setdefault((fr.sender, r.mesh),
                                            dict(ok=0, lost=0))
            if reason:
                self.stats[reason] += 1
                ls['lost'] += 1
            else:
                self.stats['delivered'] += 1
                ls['ok'] += 1
                r.handle_rx(fr, fr.end, quant_rssi(fr.rssi_at[r.mesh]),
                            quant_snr(fr.snr_at[r.mesh]))
        # prune the collision window
        cut = fr.end - AIR_HISTORY_S
        self.air = [f for f in self.air if f.end >= cut]

    def _captures(self, fr, r):
        """Capture rules (design L4): fr survives every overlapping frame it
        must, i.e. it is >= 6 dB stronger AND started no later than 3 symbols
        after the other frame (otherwise the receiver is already locked)."""
        for b in self.air:
            if b is fr or b.sender == fr.sender:
                continue
            if b.start >= fr.end or b.end <= fr.start:
                continue
            if self.faults.link_blocked(b.sender, r.mesh):
                continue                     # that signal never reaches r
            other = b.rssi_at.get(r.mesh)
            if other is None:
                continue
            if fr.rssi_at[r.mesh] - other < CAPTURE_DB:
                return False
            if fr.start > b.start + PREAMBLE_LOCK_S:
                return False
        return True

    def _interferer_hit(self, fr, r):
        if not self.faults.interferer:
            return False
        duty, dbm = self.faults.interferer
        if self.krng.random('intf', fr.sender, r.mesh, fr.seq) >= duty:
            return False
        return fr.rssi_at[r.mesh] - dbm < CAPTURE_DB

    def _uniform_drop(self, fr, r):
        pct = self.faults.link_drop.get((fr.sender, r.mesh),
                                        self.cfg.drop_pct)
        if pct <= 0:
            return False
        return self.krng.uniform(0, 100, 'drop', fr.sender, r.mesh,
                                 fr.seq) < pct

    # ---- fault verbs (design section 2) -----------------------------------
    def command(self, line):
        parts = line.strip().split()
        if not parts:
            return 'empty'
        op = parts[0].lower()
        a = parts[1:]

        def nid(x):
            return int(x)

        try:
            if op == 'kill':
                self.faults.killed.add(nid(a[0]))
            elif op == 'revive':
                i = nid(a[0])
                self.faults.killed.discard(i)
                # fresh arbitrary epoch, seq 0, sync_source = self: the node
                # collides with the mesh until it hears the authority again
                nd = self.nodes[i]
                w = self.clock.now()
                pose = nd.my_pose
                nd.setup(w)
                nd.my_pose = pose            # the ROS node kept sending poses
            elif op == 'reset':
                self.nodes[nid(a[0])].reboot(self.clock.now())
            elif op == 'wedge':
                i = nid(a[0])
                how = a[1] if len(a) > 1 else 'both'
                if how == 'off':
                    self.faults.wedged.pop(i, None)
                else:
                    self.faults.wedged[i] = how
            elif op == 'unplug':
                i = nid(a[0])
                t = float(a[1]) if len(a) > 1 else 5.0
                self.nodes[i].close_pty(drop_link=True)
                self.at(self.clock.now() + t,
                        lambda w, n=self.nodes[i]: (n.open_pty(),
                                                    n.reboot(w)))
            elif op == 'eof':
                # close the master only: on Linux the bridge's read() returns
                # b'' forever -> br:215-222 is dead code -> 100% CPU busy-spin
                self.nodes[nid(a[0])].close_pty(drop_link=False)
            elif op == 'stall':
                i = nid(a[0])
                t = float(a[1]) if len(a) > 1 else float('inf')
                self.faults.stalled[i] = self.clock.now() + t
            elif op == 'steal':
                self.faults.steal[nid(a[0])] = float(a[1])
            elif op == 'roulette':
                i, j = (nid(a[0]), nid(a[1])) if len(a) >= 2 else (0, 1)
                x, y = self.nodes[i], self.nodes[j]
                for n, tgt in ((x, y.pts), (y, x.pts)):
                    if os.path.islink(n.link):
                        os.unlink(n.link)
                    os.symlink(tgt, n.link)
            elif op == 'corrupt':
                if a[0] == 'up':
                    self.faults.corrupt_up = float(a[1])
                else:
                    self.faults.corrupt_down = float(a[1])
            elif op == 'garbage':
                i = nid(a[0])
                n = int(a[1]) if len(a) > 1 else 128
                r = self.krng.rng('garbage', i, self.event_seq)
                self.nodes[i].to_bridge.push(
                    bytes(r.randrange(256) for _ in range(n)),
                    self.clock.now())
            elif op == 'mute_link':
                x, y = nid(a[0]), nid(a[1])
                self.faults.muted.add((x, y))
                if len(a) < 3 or a[2] != 'oneway':
                    self.faults.muted.add((y, x))
            elif op == 'partition':
                rest = line.strip()[len('partition'):]
                groups = []
                for grp in rest.split('|'):
                    ids = [int(t) for t in grp.split() if t.isdigit()]
                    if ids:
                        groups.append(sorted(ids))
                self.faults.partitions = groups
            elif op == 'freeze_sync':
                i = nid(a[0])
                if len(a) > 1 and a[1] == '0':
                    self.faults.frozen_sync.discard(i)
                else:
                    self.faults.frozen_sync.add(i)
            elif op == 'jump':
                n = self.nodes[nid(a[0])]
                n.cycle_epoch = u32(n.cycle_epoch + int(a[1]))
            elif op == 'drift':
                n = self.nodes[nid(a[0])]
                n.clock.set_ppm(self.clock.now(), float(a[1]))
            elif op == 'claim_every_k':
                self.faults.claim_every_k[nid(a[0])] = int(a[1])
            elif op == 'dup_claim':
                i = nid(a[0])
                if len(a) > 1 and a[1] == '0':
                    self.faults.dup_claim.discard(i)
                else:
                    self.faults.dup_claim.add(i)
            elif op == 'misconfig':
                i = nid(a[0])
                kv = dict(t.split('=', 1) for t in a[1:] if '=' in t)
                node_id, num_nodes = self.cfg.node_constants(i)
                node_id = int(kv.get('node_id', node_id))
                num_nodes = int(kv.get('num_nodes', num_nodes))
                ent = '%d:%d:%d' % (i, node_id, num_nodes)
                keep = [e for e in self.cfg.node_cfg.split(',')
                        if e.strip() and not e.strip().startswith('%d:' % i)]
                self.cfg.node_cfg = ','.join(keep + [ent])
                self.nodes[i].reboot(self.clock.now(), spew=False)
            elif op == 'interferer':
                self.faults.interferer = (float(a[0]) / 100.0, float(a[1]))
            elif op == 'drop':
                pct = float(a[0])
                if len(a) >= 3:
                    self.faults.link_drop[(nid(a[1]), nid(a[2]))] = pct
                else:
                    self.cfg.drop_pct = pct
            elif op == 'clear':
                self.faults.clear()
                self.cfg.drop_pct = 0.0
            elif op == 'stats':
                self.print_stats()
            else:
                return 'unknown verb %r' % op
        except (IndexError, ValueError) as e:
            return 'bad args for %s: %s' % (op, e)
        self.log('FAULT %s | %s' % (line.strip(), self.counter_line()))
        return 'ok'

    # ---- counters ----------------------------------------------------------
    def counter_line(self):
        tx = sum(n.counters['tx'] for n in self.nodes)
        cus = sum(n.counters['tx_custom'] for n in self.nodes)
        miss = sum(n.counters['window_miss'] for n in self.nodes)
        qd = sum(n.counters['queue_drop'] for n in self.nodes)
        ad = sum(n.counters['air_drop_custom'] for n in self.nodes)
        lost_ev = sum(n.counters['peer_lost'] for n in self.nodes)
        join_ev = sum(n.counters['peer_joined'] for n in self.nodes)
        ovf = sum(n.counters['ser_overflow'] for n in self.nodes)
        bad = sum(n.counters['ser_bad_chk'] for n in self.nodes)
        s = self.stats
        return ('tx=%d custom=%d ok=%d lost(blocked=%d unarmed=%d deaf=%d '
                'coll=%d ge=%d per=%d drop=%d intf=%d) miss=%d qdrop=%d '
                'airdrop=%d LOST/JOIN=%d/%d ser(ovf=%d badchk=%d)'
                % (tx, cus, s['delivered'], s['lost_blocked'],
                   s['lost_unarmed'], s['lost_deaf'], s['lost_collision'],
                   s['lost_ge'], s['lost_per'], s['lost_drop'],
                   s['lost_interferer'], miss, qd, ad, lost_ev, join_ev,
                   ovf, bad))

    def snapshot(self):
        """Full counter snapshot, for the determinism check."""
        snap = {'global': dict(self.stats)}
        snap['links'] = {('%d-%d' % k): dict(v)
                         for k, v in sorted(self.link_stats.items())}
        snap['nodes'] = []
        for n in self.nodes:
            snap['nodes'].append(dict(
                counters=dict(n.counters), seq=n.my_seq,
                sync=n.my_sync_source, epoch=n.cycle_epoch,
                alive=[bool(p['alive']) for p in n.nodes],
                rx=[p['rx_count'] for p in n.nodes],
                missed=[p['missed'] for p in n.nodes]))
        snap['ge_bad_s'] = {('%d-%d' % k): round(v, 6)
                            for k, v in sorted(self.channel.ge_bad_time.items())}
        return snap

    def print_stats(self):
        self.log('STATS ' + self.counter_line())

    # ---- driving -----------------------------------------------------------
    def poll(self):
        wall = self.clock.now()

        # runtime control datagrams
        if self.ctrl is not None:
            while True:
                try:
                    data, addr = self.ctrl.recvfrom(4096)
                except (BlockingIOError, OSError):
                    break
                for line in data.decode('utf-8', 'replace').splitlines():
                    if line.strip():
                        res = self.command(line)
                        try:
                            self.ctrl.sendto((res + '\n').encode(), addr)
                        except OSError:
                            pass

        # timed schedule
        while self.sched and self.sched[0][0] <= wall:
            _, cmd = self.sched.pop(0)
            self.command(cmd)

        # due events (air start/end, deferred reopen, boot spew)
        while self.events and self.events[0][0] <= wall:
            _, _, fn = heapq.heappop(self.events)
            fn(wall)

        # per-node loop() with modelled loop-latency jitter
        for n in self.nodes:
            n.pump_pty(wall)
            if wall >= n.next_loop_wall:
                n.loop(wall)
                n.loop_count += 1
                j = self.cfg.loop_jitter_ms
                if j > 0:
                    r = self.krng.rng('loop', n.mesh, n.loop_count)
                    d = (r.uniform(0, j * LOOP_SPIKE_MULT)
                         if r.random() < LOOP_SPIKE_P else r.uniform(0, j))
                    n.next_loop_wall = wall + d / 1000.0
                else:
                    n.next_loop_wall = wall

        if self.cfg.stats_s > 0 and wall - self.last_stats >= self.cfg.stats_s:
            self.last_stats = wall
            self.print_stats()

    def run(self, poll_s=0.0005):
        self.log('virtual radio v2: %d nodes, cycle %d ms, links in %s/, '
                 'ctrl udp 127.0.0.1:%d, seed %d'
                 % (self.cfg.n, self.nodes[0].cycle_ms, self.cfg.linkdir,
                    self.cfg.ctrl_port, self.cfg.seed))
        while True:
            self.poll()
            time.sleep(poll_s)

    def close(self):
        for n in self.nodes:
            n.close_pty()
        if self.ctrl is not None:
            self.ctrl.close()
            self.ctrl = None


# ===================================================================
# LORA_MODE=ideal -- v1 semantics verbatim, as a regression escape.
# Instant delivery, bridge-style rescanning parser, silent until the first
# pose, len>=1 customs, BASE_PACKET=39 (v1's off-by-one), fake STATUS_RESP,
# no peer events, one shared clock.  Bugs preserved on purpose.
# ===================================================================
class IdealEmulator(object):
    BASE_PACKET = 39
    AIRTIME_BUDGET_MS = 30.0

    class Port(object):
        def __init__(self, emu, mesh):
            self.emu = emu
            self.mesh = mesh
            self.mfd, sfd = os.openpty()
            tty.setraw(sfd)
            os.set_blocking(self.mfd, False)
            link = os.path.join(emu.cfg.linkdir, 'drone%d' % (mesh + 1))
            if os.path.islink(link) or os.path.exists(link):
                os.unlink(link)
            os.symlink(os.ttyname(sfd), link)
            self.link = link
            self.buf = bytearray()
            self.pose = None
            self.custom = None
            self.dropped_customs = 0
            self._last_tx_cycle = -1

        def read(self):
            try:
                data = os.read(self.mfd, 4096)
                if data:
                    self.buf.extend(data)
            except (BlockingIOError, OSError):
                pass
            i = 0
            while i < len(self.buf):
                if self.buf[i] != SERIAL_START:
                    i += 1
                    continue
                if len(self.buf) - i < 4:
                    break
                mtype, ln = self.buf[i + 1], self.buf[i + 2]
                if len(self.buf) - i < 4 + ln:
                    break
                payload = bytes(self.buf[i + 3:i + 3 + ln])
                chk = self.buf[i + 3 + ln]
                x = mtype ^ ln
                for b in payload:
                    x ^= b
                if (x & 0xFF) != chk:
                    i += 1
                    continue
                self._handle(mtype, payload)
                i += 4 + ln
            del self.buf[:i]

        def _handle(self, mtype, payload):
            if mtype == T_POSE_UPDATE and len(payload) == 16:
                self.pose = payload
            elif mtype == T_CUSTOM_MSG and len(payload) >= 1:
                if self.custom is None:
                    self.custom = (payload[0], payload[1:])
                else:
                    self.dropped_customs += 1
            elif mtype == T_STATUS_REQ:
                alive = [p for p in self.emu.ports
                         if p is not self and p.pose is not None]
                resp = bytes([self.mesh, len(alive)])
                for p in alive:
                    resp += struct.pack('<Bhf', p.mesh, -40, 8.0)
                self.write(frame(T_STATUS_RESP, resp))

        def write(self, data):
            try:
                os.write(self.mfd, data)
            except OSError:
                pass

    def __init__(self, cfg=None, clock=None):
        self.cfg = cfg or Config()
        self.clock = clock or RealClock()
        self.n = self.cfg.n
        self.cycle_ms = self.n * SLOT_MS + GUARD_MS
        os.makedirs(self.cfg.linkdir, exist_ok=True)
        self.ports = [IdealEmulator.Port(self, i) for i in range(self.n)]
        self.rng = random.Random(self.cfg.seed)
        self.stats = dict(pose_tx=0, custom_tx=0, custom_drop_air=0)
        self.start = self.clock.now()
        self.last_report = self.start

    def poll(self):
        now = self.clock.now()
        for p in self.ports:
            p.read()
        t_in_cycle = ((now - self.start) * 1000.0) % self.cycle_ms
        slot = int(t_in_cycle // SLOT_MS)
        for p in self.ports:
            if p.mesh != slot or slot >= self.n:
                continue
            cycle_no = int((now - self.start) * 1000.0 // self.cycle_ms)
            if p._last_tx_cycle == cycle_no:
                continue
            if t_in_cycle - slot * SLOT_MS < TX_MARGIN_MS:
                continue
            p._last_tx_cycle = cycle_no
            if p.pose is not None:
                out = frame(T_PEER_POSE, struct.pack('<B', p.mesh) + p.pose +
                            struct.pack('<hf', -40, 8.0))
                for q in self.ports:
                    if q is not p and \
                            self.rng.uniform(0, 100) >= self.cfg.drop_pct:
                        q.write(out)
                self.stats['pose_tx'] += 1
            if p.custom is not None:
                dest, data = p.custom
                if lora_airtime_ms(self.BASE_PACKET + len(data)) > \
                        self.AIRTIME_BUDGET_MS:
                    self.stats['custom_drop_air'] += 1
                else:
                    out = frame(T_PEER_MSG,
                                struct.pack('<B', p.mesh) + data)
                    for q in self.ports:
                        if q is p:
                            continue
                        if dest != 0xFF and dest != q.mesh:
                            continue
                        if self.rng.uniform(0, 100) >= self.cfg.drop_pct:
                            q.write(out)
                    self.stats['custom_tx'] += 1
                p.custom = None
        if now - self.last_report > 10:
            drops = sum(p.dropped_customs for p in self.ports)
            print('radio(ideal): pose_tx=%d custom_tx=%d air_drops=%d '
                  'queue_drops=%d' % (self.stats['pose_tx'],
                                      self.stats['custom_tx'],
                                      self.stats['custom_drop_air'], drops),
                  flush=True)
            self.last_report = now

    def run(self, poll_s=0.002):
        print('virtual radio (ideal/v1 mode): %d nodes, cycle %d ms, '
              'links in %s/' % (self.n, self.cycle_ms, self.cfg.linkdir),
              flush=True)
        while True:
            self.poll()
            time.sleep(poll_s)

    def close(self):
        for p in self.ports:
            try:
                os.close(p.mfd)
            except OSError:
                pass


def build(cfg=None, clock=None):
    cfg = cfg or Config()
    if cfg.mode == 'ideal':
        return IdealEmulator(cfg, clock)
    return Emulator(cfg, clock)


def main():
    emu = build()
    try:
        emu.run()
    except KeyboardInterrupt:
        pass
    finally:
        emu.close()


if __name__ == '__main__':
    main()
