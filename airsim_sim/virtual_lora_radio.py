#!/usr/bin/env python3
"""Virtual LoRa TDMA radio for the HERCULES fleet sim (Run B).

Plays the role of N lora_mesh v7.2 radios + the RF channel, per the wiring
runbook: owns one pty per drone (the real radiohive lora_bridge_node opens
the slave side), runs the firmware's TDMA cycle, and rebroadcasts each
drone's pose/claim to every other drone with real slot timing.

Protocol (mirrors radiohive/scripts/lora_bridge_node.py + lora_mesh_v7_2):
  frame  = [0xAA][type][len][payload...][xor(type,len,payload)]
  bridge->radio: 0x01 POSE_UPDATE <ffff x,y,z,yaw   0x02 CUSTOM_MSG dest+data
  radio->bridge: 0x81 PEER_POSE <Bffffhf node0,x,y,z,yaw,rssi,snr
                 0x82 PEER_MSG src+data
Wire ids: 0x81 node byte is 0-BASED mesh id (vehicle_id-1, FLAG-6);
custom payloads pass through opaque (claim src_id inside is 1-based).
TDMA (N=4 firmware constants): SLOT_MS=40, GUARD_MS=30 -> CYCLE_MS=190.
One pending custom per node, single-deep, dropped iff airtime(39+len)>30ms
(lora_airtime_ms imported from their mdn_core.lora_packet).
Realism knobs via env: LORA_DROP_PCT (per-receiver drop %), LORA_SEED.
"""
import os
import random
import struct
import sys
import time
import tty

sys.path.insert(0, '/home/lucas/hercules-sim/src/multi_drone_nvblox')
from mdn_core.lora_packet import lora_airtime_ms  # their exact SX127x math

N = int(os.environ.get('LORA_N', '4'))
SLOT_MS, GUARD_MS, TX_MARGIN_MS = 40, 30, 5
CYCLE_MS = N * SLOT_MS + GUARD_MS          # 190 ms at N=4
BASE_PACKET = 39                            # firmware base packet size at N=4
AIRTIME_BUDGET_MS = 30.0                    # SLOT_MS - 2*TX_MARGIN
DROP_PCT = float(os.environ.get('LORA_DROP_PCT', '0'))
random.seed(int(os.environ.get('LORA_SEED', '7')))

SYNC = 0xAA
T_POSE_UPDATE, T_CUSTOM_MSG, T_STATUS_REQ = 0x01, 0x02, 0x03
T_PEER_POSE, T_PEER_MSG, T_STATUS_RESP = 0x81, 0x82, 0x83
LINKDIR = '/tmp/hercules_lora'


def frame(mtype, payload=b''):
    chk = mtype ^ (len(payload) & 0xFF)
    for b in payload:
        chk ^= b
    return bytes([SYNC, mtype & 0xFF, len(payload) & 0xFF]) + payload + \
        bytes([chk & 0xFF])


class Port:
    """One drone's radio-side pty + parser state + per-node radio state."""

    def __init__(self, mesh_id):
        self.mesh_id = mesh_id                       # 0-based
        self.mfd, sfd = os.openpty()
        tty.setraw(sfd)
        os.set_blocking(self.mfd, False)
        link = f'{LINKDIR}/drone{mesh_id + 1}'
        if os.path.islink(link) or os.path.exists(link):
            os.unlink(link)
        os.symlink(os.ttyname(sfd), link)
        self.buf = bytearray()
        self.pose = None                             # latest <ffff payload
        self.custom = None                           # pending (dest, data)
        self.dropped_customs = 0

    def read(self):
        try:
            data = os.read(self.mfd, 4096)
            if data:
                self.buf.extend(data)
        except (BlockingIOError, OSError):
            pass
        # parse frames (resync semantics of lora_bridge_node)
        i = 0
        while i < len(self.buf):
            if self.buf[i] != SYNC:
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
            self.pose = payload                      # last-write-wins
        elif mtype == T_CUSTOM_MSG and len(payload) >= 1:
            if self.custom is None:                  # single-deep queue
                self.custom = (payload[0], payload[1:])
            else:
                self.dropped_customs += 1            # firmware drops silently
        elif mtype == T_STATUS_REQ:
            alive = [p for p in PORTS if p is not self and p.pose is not None]
            resp = bytes([self.mesh_id, len(alive)])
            for p in alive:
                resp += struct.pack('<Bhf', p.mesh_id, -40, 8.0)
            self.write(frame(T_STATUS_RESP, resp))

    def write(self, data):
        try:
            os.write(self.mfd, data)
        except OSError:
            pass


os.makedirs(LINKDIR, exist_ok=True)
PORTS = [Port(i) for i in range(N)]
print(f'virtual radio: {N} nodes, cycle {CYCLE_MS}ms, links in {LINKDIR}/',
      flush=True)

cycle_start = time.monotonic()
stats = {'pose_tx': 0, 'custom_tx': 0, 'custom_drop_air': 0}
last_report = time.monotonic()

while True:
    now = time.monotonic()
    for p in PORTS:
        p.read()

    t_in_cycle = ((now - cycle_start) * 1000.0) % CYCLE_MS
    slot = int(t_in_cycle // SLOT_MS)
    # transmit for the node whose slot just hit its TX point
    for p in PORTS:
        if p.mesh_id != slot or slot >= N:
            continue
        if not hasattr(p, '_last_tx_cycle'):
            p._last_tx_cycle = -1
        cycle_no = int((now - cycle_start) * 1000.0 // CYCLE_MS)
        if p._last_tx_cycle == cycle_no:
            continue
        if t_in_cycle - slot * SLOT_MS < TX_MARGIN_MS:
            continue
        p._last_tx_cycle = cycle_no
        # pose broadcast (base packet)
        if p.pose is not None:
            out = frame(T_PEER_POSE,
                        struct.pack('<B', p.mesh_id) + p.pose +
                        struct.pack('<hf', -40, 8.0))
            for q in PORTS:
                if q is not p and random.uniform(0, 100) >= DROP_PCT:
                    q.write(out)
            stats['pose_tx'] += 1
        # piggybacked custom (claim), airtime-gated like the firmware
        if p.custom is not None:
            dest, data = p.custom
            if lora_airtime_ms(BASE_PACKET + len(data)) > AIRTIME_BUDGET_MS:
                stats['custom_drop_air'] += 1
            else:
                out = frame(T_PEER_MSG, struct.pack('<B', p.mesh_id) + data)
                for q in PORTS:
                    if q is p:
                        continue
                    if dest != 0xFF and dest != q.mesh_id:
                        continue
                    if random.uniform(0, 100) >= DROP_PCT:
                        q.write(out)
                stats['custom_tx'] += 1
            p.custom = None                          # cleared even if dropped

    if now - last_report > 10:
        drops = sum(p.dropped_customs for p in PORTS)
        print(f'radio: pose_tx={stats["pose_tx"]} custom_tx={stats["custom_tx"]} '
              f'air_drops={stats["custom_drop_air"]} queue_drops={drops}',
              flush=True)
        last_report = now
    time.sleep(0.002)
