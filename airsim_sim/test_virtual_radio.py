#!/usr/bin/env python3
"""Validation suite for virtual_lora_radio_v2.py (RADIO-V2-DESIGN.md section 4).

Runnable as a plain script -- no pytest, no ROS:

    python3 test_virtual_radio.py            # all tests
    python3 test_virtual_radio.py cadence    # substring filter

Covers design section 4 items 1, 2, 3 and 6, plus the determinism check from
item 5:

  1. airtime goldens {38: 20.544, 57: 26.944, 64: 29.504, 198: 79.424} ms,
     max_tx_size == 64, and the 26-byte-accept / 27-byte-drop custom gate
     (the regression test for v1's BASE_PACKET = 39 off-by-one).
  2. frame-level goldens: REAL bytes through the pty, parsed by a verbatim
     copy of lora_bridge_node._parse (br:240-268) -- byte-exact 0x81/0x82/
     0x83 layouts, PEER_JOINED before the first PEER_POSE, the CUSTOM_MSG
     len >= 2 rule, dest filtering and no self-delivery.
  3. cadence: ~10 s of wall time with four attached bridges -- per-peer
     PEER_POSE inter-arrival 190 ms +/- jitter, ~15.8 Hz aggregate per bridge.
  6. peer-expiry reachability: uniform 20 % loss yields ZERO 3 s expiries,
     while a Gilbert-Elliott burst configuration produces them.
  5. determinism: same seed + same schedule file, two runs, identical
     counters / peer tables / event logs.

Plus a smoke test of the runtime fault API (UDP datagram round trip).
"""

import os
import shutil
import struct
import sys
import tempfile
import termios
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import virtual_lora_radio_v2 as V   # noqa: E402

SYNC = 0xAA
T_POSE_UPDATE, T_CUSTOM_MSG, T_STATUS_REQ = 0x01, 0x02, 0x03
T_PEER_POSE, T_PEER_MSG, T_STATUS_RESP = 0x81, 0x82, 0x83
T_PEER_LOST, T_PEER_JOINED = 0x84, 0x85

FMT_PEER_POSE = '<Bffffhf'      # br:63
FMT_JOINED = '<Bhf'             # br:64
SZ_STATUS_ENTRY = 7             # br:65


# ===================================================================
# A stand-in for lora_bridge_node: opens the symlink exactly the way the
# real node does (raw termios 8N1, CRTSCTS cleared, TCIOFLUSH on open,
# br:109-125) and parses with a verbatim copy of its frame parser.
# ===================================================================
class FakeBridge(object):
    def __init__(self, path, stamp):
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = a
        iflag = oflag = lflag = 0
        cflag = (cflag & ~termios.CSIZE & ~termios.PARENB & ~termios.CSTOPB
                 & ~termios.CRTSCTS) | termios.CS8 | termios.CLOCAL \
            | termios.CREAD
        ispeed = ospeed = termios.B115200
        cc = list(cc)
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW,
                          [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self._rx = bytearray()
        self.raw = bytearray()
        self.frames = []            # (t, type, payload)
        self.stamp = stamp          # callable returning the current time

    # verbatim port of LoraBridge._parse (br:240-268)
    def _parse(self, t):
        buf = self._rx
        i = 0
        n = len(buf)
        while i < n:
            if buf[i] != SYNC:
                i += 1
                continue
            if n - i < 3:
                break
            mtype = buf[i + 1]
            ln = buf[i + 2]
            frame_len = 3 + ln + 1
            if n - i < frame_len:
                break
            payload = bytes(buf[i + 3:i + 3 + ln])
            chk = buf[i + 3 + ln]
            calc = mtype ^ ln
            for b in payload:
                calc ^= b
            if (calc & 0xFF) != chk:
                i += 1
                continue
            self.frames.append((t, mtype, payload))
            i += frame_len
        del self._rx[:i]

    def pump(self):
        try:
            chunk = os.read(self.fd, 4096)
        except (BlockingIOError, OSError):
            return
        if chunk:
            self.raw.extend(chunk)
            self._rx.extend(chunk)
            self._parse(self.stamp())

    # verbatim port of LoraBridge._frame / _send (br:330-347)
    @staticmethod
    def frame(mtype, payload=b''):
        ln = len(payload) & 0xFF
        chk = mtype ^ ln
        for b in payload:
            chk ^= b
        return bytes([SYNC, mtype & 0xFF, ln]) + payload + bytes([chk & 0xFF])

    def send(self, mtype, payload=b''):
        os.write(self.fd, self.frame(mtype, payload))

    def of_type(self, mtype):
        return [f for f in self.frames if f[1] == mtype]

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


# ===================================================================
# Harness
# ===================================================================
class Harness(object):
    """An emulator on a virtual clock, with a bridge on every pty."""

    def __init__(self, tmp, attach=True, **cfg):
        cfg.setdefault('link_dir', os.path.join(tmp, 'links'))
        cfg.setdefault('ctrl_port', 0)
        cfg.setdefault('quiet', 1)
        cfg.setdefault('stats_s', 0)
        cfg.setdefault('reset_on_open', 0)   # no DTR reset mid-test
        cfg.setdefault('n', 4)
        cfg.setdefault('seed', 7)
        self.cfg = V.Config(env={}, **cfg)
        self.clock = V.SimClock()
        self.emu = V.Emulator(self.cfg, self.clock)
        self.bridges = []
        if attach:
            for n in self.emu.nodes:
                self.bridges.append(FakeBridge(n.link, self.clock.now))

    def run(self, seconds, step=0.0005):
        end = self.clock.now() + seconds
        while self.clock.now() < end:
            self.clock.advance(step)
            self.emu.poll()
            for b in self.bridges:
                b.pump()

    def close(self):
        for b in self.bridges:
            b.close()
        self.emu.close()


def approx(a, b, tol):
    return abs(a - b) <= tol


# ===================================================================
# Design section 4, item 1 -- airtime inheritance + the custom-size gate
# ===================================================================
def test_airtime_goldens(tmp):
    golden = {38: 20.544, 57: 26.944, 64: 29.504, 198: 79.424}
    for size, ms in golden.items():
        got = V.lora_airtime_ms(size)
        assert approx(got, ms, 1e-6), \
            'airtime(%d) = %.6f ms, expected %.3f' % (size, got, ms)

    # firmware slot gate: largest size with getTimeOnAir <= 30,000 us
    assert V.compute_max_tx_size(38, 198) == 64, V.compute_max_tx_size(38, 198)
    assert V.airtime_us(64) <= 30000 < V.airtime_us(65)

    # the integer-truncated airtimes the TX window arithmetic uses (fw:382)
    assert V.airtime_int_ms(38) == 20
    assert V.airtime_int_ms(57) == 26

    # v1's BASE_PACKET = 39 would have capped the custom at 25 bytes
    assert V.compute_max_tx_size(38, 198) - 38 == 26
    assert V.compute_max_tx_size(39, 199) - 39 == 25

    # design P3-16: the legacy 32-byte full packet cannot ride the piggyback
    assert approx(V.lora_airtime_ms(38 + 32), 32.064, 1e-6)
    assert 38 + 32 > V.compute_max_tx_size(38, 198)

    h = Harness(tmp)
    try:
        n = h.emu.nodes[0]
        assert n.base_packet == 38 and n.sizeof_packet == 198
        assert n.max_tx_size == 64 and n.packet_airtime_ms == 20
        assert n.cycle_ms == 190
    finally:
        h.close()
    return 'airtimes %s; max_tx_size=64 (26 B custom)' % \
        ','.join('%d:%.3f' % (k, v) for k, v in sorted(golden.items()))


def test_custom_size_gate(tmp):
    """26 B custom rides; 27 B is dropped WHOLE (never truncated), and the
    pose still goes out in the same cycle."""
    out = []
    for size, expect in ((26, True), (27, False)):
        h = Harness(tmp)
        try:
            h.run(0.5)                      # let the mesh form
            data = bytes(range(size))
            h.bridges[0].send(T_CUSTOM_MSG, bytes([0xFF]) + data)
            base_pose = len(h.bridges[1].of_type(T_PEER_POSE))
            h.run(1.0)
            msgs = [p for _, _, p in h.bridges[1].of_type(T_PEER_MSG)]
            got = any(p == bytes([0]) + data for p in msgs)
            assert got == expect, \
                '%d-byte custom: delivered=%s expected=%s' % (size, got, expect)
            assert len(h.bridges[1].of_type(T_PEER_POSE)) > base_pose, \
                'pose flow must continue regardless of the custom'
            if not expect:
                assert h.emu.nodes[0].counters['air_drop_custom'] == 1
            out.append('%dB:%s' % (size, 'accepted' if got else 'dropped'))
        finally:
            h.close()
    return ' '.join(out)


# ===================================================================
# Design section 4, item 2 -- frame-level goldens through the real pty
# ===================================================================
def test_frames_golden(tmp):
    h = Harness(tmp)
    try:
        pose = struct.pack('<ffff', 12.5, -3.25, 1.75, 0.5)

        # (a) always-beacon: peers see a ZERO pose before any POSE_UPDATE,
        #     and PEER_JOINED is emitted before that first PEER_POSE.
        h.run(0.5)
        f1 = h.bridges[1].frames
        idx_join = next(i for i, f in enumerate(f1)
                        if f[1] == T_PEER_JOINED and f[2][0] == 0)
        idx_pose = next(i for i, f in enumerate(f1)
                        if f[1] == T_PEER_POSE and f[2][0] == 0)
        assert idx_join < idx_pose, 'PEER_JOINED must precede PEER_POSE'
        j = f1[idx_join][2]
        assert len(j) == struct.calcsize(FMT_JOINED) == 7, len(j)
        jn, jrssi, jsnr = struct.unpack(FMT_JOINED, j)
        assert jn == 0 and -200 < jrssi < 30 and jsnr <= 10.0
        first_pose = f1[idx_pose][2]
        assert len(first_pose) == struct.calcsize(FMT_PEER_POSE) == 23
        assert first_pose[1:17] == bytes(16), \
            'always-beacon must carry the zero-init pose register'

        # (b) POSE_UPDATE -> byte-exact PEER_POSE at every peer, none at self
        h.bridges[0].send(T_POSE_UPDATE, pose)
        h.run(1.0)
        for b in (h.bridges[1], h.bridges[2], h.bridges[3]):
            hit = [p for _, _, p in b.of_type(T_PEER_POSE)
                   if p[0] == 0 and p[1:17] == pose]
            assert hit, 'no PEER_POSE carrying the exact pose at %s' % b.path
            p = hit[-1]
            node, lat, lon, alt, hdg, rssi, snr = struct.unpack(FMT_PEER_POSE, p)
            assert (node, lat, lon, alt, hdg) == (0, 12.5, -3.25, 1.75, 0.5)
            assert -32768 <= rssi <= 32767 and snr <= 10.0
            # and the raw wire framing is exactly [AA][81][17][payload][xor]
            assert FakeBridge.frame(T_PEER_POSE, p) in bytes(b.raw)
        assert not [f for f in h.bridges[0].of_type(T_PEER_POSE)
                    if f[2][0] == 0], 'no self-delivery'

        # (c) dest filtering: dest = 2 reaches only node 2
        data = b'\x01claim-to-2'
        h.bridges[0].send(T_CUSTOM_MSG, bytes([2]) + data)
        h.run(1.0)
        assert [p for _, _, p in h.bridges[2].of_type(T_PEER_MSG)
                if p == bytes([0]) + data], 'dest=2 must receive PEER_MSG'
        for b in (h.bridges[1], h.bridges[3], h.bridges[0]):
            assert not [p for _, _, p in b.of_type(T_PEER_MSG)
                        if p == bytes([0]) + data], \
                'dest filtering leaked to %s' % b.path

        # (d) broadcast 0xFF reaches all peers, never the sender
        bdata = b'\x02broadcast'
        h.bridges[0].send(T_CUSTOM_MSG, bytes([0xFF]) + bdata)
        h.run(1.0)
        for b in (h.bridges[1], h.bridges[2], h.bridges[3]):
            assert [p for _, _, p in b.of_type(T_PEER_MSG)
                    if p == bytes([0]) + bdata], 'broadcast missed %s' % b.path
        assert not h.bridges[0].of_type(T_PEER_MSG), 'sender got its own claim'

        # (e) CUSTOM_MSG with len < 2 (dest only) is IGNORED (fw:324).
        #     v1 queued it and shipped an empty PEER_MSG.
        before = [len(b.of_type(T_PEER_MSG)) for b in h.bridges]
        h.bridges[1].send(T_CUSTOM_MSG, bytes([0xFF]))
        h.run(1.0)
        after = [len(b.of_type(T_PEER_MSG)) for b in h.bridges]
        assert before == after, 'len<2 CUSTOM_MSG must be ignored'
        assert not h.emu.nodes[1].custom_pending

        # (f) STATUS_RESP is built from the real peer table (fw:245-263)
        h.bridges[0].send(T_STATUS_REQ, b'')
        h.run(0.5)
        st = h.bridges[0].of_type(T_STATUS_RESP)
        assert st, 'no STATUS_RESP'
        p = st[-1][2]
        assert p[0] == 0, p[0]
        num = p[1]
        assert num == 3, 'expected 3 alive peers, got %d' % num
        assert len(p) == 2 + num * SZ_STATUS_ENTRY == 23, len(p)
        ids = []
        for k in range(num):
            off = 2 + k * SZ_STATUS_ENTRY
            pid, rssi, snr = struct.unpack('<Bhf', p[off:off + SZ_STATUS_ENTRY])
            ids.append(pid)
            assert -200 < rssi < 30 and snr <= 10.0
        assert ids == [1, 2, 3], ids
    finally:
        h.close()
    return ('0x81/0x82/0x83 byte-exact; JOINED<POSE; zero-init beacon; '
            'dest filter + no self-delivery; len<2 ignored')


# ===================================================================
# Design section 4, item 3 -- cadence (real wall clock, ~10 s)
# ===================================================================
def test_cadence(tmp):
    cfg = dict(link_dir=os.path.join(tmp, 'cadence'), ctrl_port=0, quiet=1,
               stats_s=0, reset_on_open=0, n=4, seed=11)
    emu = V.Emulator(V.Config(env={}, **cfg), V.RealClock())
    bridges = [FakeBridge(n.link, emu.clock.now) for n in emu.nodes]
    try:
        t_end = emu.clock.now() + 10.0
        while emu.clock.now() < t_end:
            emu.poll()
            for b in bridges:
                b.pump()
            time.sleep(0.0002)
        span = emu.clock.now()

        gaps = []
        rates = []
        for bi, b in enumerate(bridges):
            poses = b.of_type(T_PEER_POSE)
            rates.append(len(poses) / span)
            per_peer = {}
            for t, _, p in poses:
                per_peer.setdefault(p[0], []).append(t)
            assert len(per_peer) == 3, \
                'bridge %d saw %d peers' % (bi, len(per_peer))
            for pid, ts in per_peer.items():
                assert len(ts) > 40, \
                    'bridge %d peer %d: only %d poses' % (bi, pid, len(ts))
                gaps += [(b - a) * 1000.0 for a, b in zip(ts, ts[1:])]

        gaps.sort()
        median = gaps[len(gaps) // 2]
        within = sum(1 for g in gaps if abs(g - 190.0) <= 30.0) / len(gaps)
        agg = sum(rates) / len(rates)
        assert approx(median, 190.0, 5.0), \
            'median PEER_POSE gap %.1f ms, expected 190' % median
        assert within >= 0.90, \
            'only %.1f%% of gaps within 190+/-30 ms' % (100.0 * within)
        assert 14.0 <= agg <= 16.5, \
            'aggregate %.2f Hz per bridge, expected ~15.8' % agg
        # every launch sat inside [5, 35 - airtime]; nothing reached the guard
        offs = [o for n in emu.nodes for o in n.launch_offsets]
        assert offs, 'no launches recorded'
        for tis, air in offs:
            assert 5 <= tis <= 35 - air, \
                'launch at +%d ms with %d ms airtime is outside the window' \
                % (tis, air)
        assert max(t for t, _ in offs) <= 15, 'pose launch past +15 ms'
        assert emu.stats['lost_collision'] == 0, 'unexpected collisions'
        return ('%.1f s wall, median gap %.1f ms, %.1f%% within +/-30 ms, '
                '%.2f Hz/bridge, launches +%d..+%d ms'
                % (span, median, 100.0 * within, agg,
                   min(t for t, _ in offs), max(t for t, _ in offs)))
    finally:
        for b in bridges:
            b.close()
        emu.close()


# ===================================================================
# Design section 4, item 6 -- peer-expiry reachability
# ===================================================================
def _expiry_run(tmp, tag, seconds, **cfg):
    h = Harness(tmp, link_dir=os.path.join(tmp, tag), **cfg)
    try:
        h.run(seconds, step=0.001)
        return sum(n.counters['peer_lost'] for n in h.emu.nodes), \
            h.emu.stats['delivered'], \
            (h.emu.stats['lost_drop'] + h.emu.stats['lost_ge'])
    finally:
        h.close()


def test_uniform_vs_burst_expiry(tmp):
    """A peer expires only after 3.0 s of silence = 15.8 consecutive lost
    cycles.  Uniform 20 % loss makes that ~1e-11 per opportunity, so it can
    never explain the field's PEER_LOST events; a Gilbert-Elliott burst can.
    (Design section 4 item 6, shortened analog of the 30 min run.)"""
    lost_u, ok_u, drop_u = _expiry_run(tmp, 'uniform', 120.0, drop_pct=20.0)
    assert drop_u > 0, 'uniform drop did not fire at all'
    assert 0.15 < drop_u / float(ok_u + drop_u) < 0.25, \
        'uniform loss rate %.3f, expected ~0.20' % (drop_u / float(ok_u + drop_u))
    assert lost_u == 0, \
        'uniform 20%% produced %d PEER_LOST events (must be zero)' % lost_u

    # mean bad-state run = 1/p_bg = 20 packets = 3.8 s > the 3 s timeout
    lost_g, ok_g, drop_g = _expiry_run(tmp, 'burst', 120.0,
                                       ge='0.02,0.05,0.0,1.0')
    assert lost_g >= 1, 'GE bursts produced no PEER_LOST events'
    return ('uniform 20%%: %d expiries (%d lost / %d ok) | GE burst: '
            '%d expiries (%d lost / %d ok)'
            % (lost_u, drop_u, ok_u, lost_g, drop_g, ok_g))


# ===================================================================
# Design section 4, item 5 -- determinism
# ===================================================================
SCHEDULE = """# scripted faults for the determinism check
t=2.0   drop 5
t=4.0   kill 0
t=8.0   revive 0
t=10.0  mute_link 1 2 oneway
t=12.0  claim_every_k 3 3
t=14.0  freeze_sync 2
t=16.0  clear
"""


def test_determinism(tmp):
    sched = os.path.join(tmp, 'schedule.txt')
    with open(sched, 'w') as f:
        f.write(SCHEDULE)

    def once(tag):
        h = Harness(tmp, link_dir=os.path.join(tmp, tag), schedule=sched,
                    seed=1234, ge='0.01,0.05,0.0,1.0')
        try:
            pose = struct.pack('<ffff', 1.0, 2.0, 3.0, 4.0)
            for i, b in enumerate(h.bridges):
                b.send(T_POSE_UPDATE, pose)
            h.run(6.0, step=0.001)
            for i, b in enumerate(h.bridges):
                b.send(T_CUSTOM_MSG, bytes([0xFF]) + b'\x01claim%d' % i)
            h.run(12.0, step=0.001)
            return h.emu.snapshot(), list(h.emu.log_lines), \
                [len(b.frames) for b in h.bridges]
        finally:
            h.close()

    a_snap, a_log, a_frames = once('det_a')
    b_snap, b_log, b_frames = once('det_b')
    assert a_snap == b_snap, _first_diff(a_snap, b_snap)
    assert a_log == b_log, 'event logs differ'
    assert a_frames == b_frames, 'bridge frame counts differ: %s vs %s' % (
        a_frames, b_frames)
    assert a_snap['global']['delivered'] > 100, 'run produced no traffic'
    return ('two runs identical: %d counters, %d log lines, frames %s'
            % (len(a_snap['nodes']), len(a_log), a_frames))


def _first_diff(a, b, path='snapshot'):
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            if a.get(k) != b.get(k):
                return _first_diff(a.get(k), b.get(k), '%s.%s' % (path, k))
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        for i, (x, y) in enumerate(zip(a, b)):
            if x != y:
                return _first_diff(x, y, '%s[%d]' % (path, i))
    return 'differs at %s: %r != %r' % (path, a, b)


# ===================================================================
# Runtime fault API smoke test (design section 2)
# ===================================================================
def test_fault_api(tmp):
    import socket
    h = Harness(tmp, link_dir=os.path.join(tmp, 'ctrl'), ctrl_port=47859)
    try:
        h.run(0.5)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(1.0)
        replies = []
        for cmd in ('kill 0', 'wedge 1 tx', 'drift 2 40', 'jump 3 25',
                    'partition 0 1|2 3', 'bogus_verb'):
            s.sendto(cmd.encode(), ('127.0.0.1', 47859))
            for _ in range(200):
                h.clock.advance(0.0005)
                h.emu.poll()
            try:
                replies.append(s.recv(256).decode().strip())
            except socket.timeout:
                replies.append('<timeout>')
        s.close()
        assert replies[:5] == ['ok'] * 5, replies
        assert replies[5].startswith('unknown verb'), replies[5]
        assert 0 in h.emu.faults.killed
        assert h.emu.faults.wedged.get(1) == 'tx'
        assert approx(h.emu.nodes[2].clock.ppm, 40.0, 1e-9)
        assert h.emu.faults.partitions == [[0, 1], [2, 3]]
        # a killed radio must go silent; peers then expire it after 3 s
        h.run(4.0, step=0.001)
        assert h.emu.nodes[1].counters['peer_lost'] >= 1, \
            'kill 0 did not produce a PEER_LOST at its peers'
        h.emu.command('clear')
        assert not h.emu.faults.killed and not h.emu.faults.partitions
    finally:
        h.close()
    return 'udp verbs ok; kill -> PEER_LOST after 3 s; clear resets'


# ===================================================================
TESTS = [
    ('airtime_goldens', test_airtime_goldens),
    ('custom_size_gate', test_custom_size_gate),
    ('frames_golden', test_frames_golden),
    ('cadence', test_cadence),
    ('uniform_vs_burst_expiry', test_uniform_vs_burst_expiry),
    ('determinism', test_determinism),
    ('fault_api', test_fault_api),
]


def main(argv):
    want = argv[1:] if len(argv) > 1 else None
    tmp = tempfile.mkdtemp(prefix='vlora2_test_')
    failed = 0
    try:
        for name, fn in TESTS:
            if want and not any(w in name for w in want):
                continue
            t0 = time.monotonic()
            try:
                detail = fn(tmp)
                print('PASS  %-24s %5.1fs  %s'
                      % (name, time.monotonic() - t0, detail or ''))
            except Exception as e:                     # noqa: BLE001
                failed += 1
                print('FAIL  %-24s %5.1fs  %s: %s'
                      % (name, time.monotonic() - t0, type(e).__name__, e))
                import traceback
                traceback.print_exc()
            sys.stdout.flush()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('\n%d test(s) failed' % failed if failed else '\nall tests passed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
