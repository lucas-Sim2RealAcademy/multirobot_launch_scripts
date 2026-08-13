#!/usr/bin/env python3
"""Virtual RC pilot: reproduces the field flight-entry ritual against PX4 SITL.

Field ritual (AGENT_HANDOFF / flight_autonomous.sh): pilot arms in Position
mode on the RC, takes off to a hover on sticks, then flips the mode switch to
Offboard — and keeps holding centered sticks all flight, which is what makes
the Offboard->POSCTL demotion and RC-loss failsafes REACHABLE.

MANUAL_CONTROL scaling (mavlink_receiver.cpp:2083-2087): x/y/r in -1000..1000,
z throttle 0..1000 (500 = center). Arm gesture: z=0, r=+1000 held ~2.5s
(MAN_ARM_GESTURE default on). SITL GCS link: udpin 14550.

Flags: --alt (hover altitude, m), --drop-rc-at SEC (stop MANUAL_CONTROL
mid-flight: RC-loss failsafe repro), --no-offboard (stay in stick-held
Position hover: EV/EKF2 debugging regime).
"""
import argparse
import threading
import time

from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument('--alt', type=float, default=1.0)
ap.add_argument('--drop-rc-at', type=float, default=0.0)
ap.add_argument('--no-offboard', action='store_true')
ap.add_argument('--port', type=int, default=14550)
args = ap.parse_args()

m = mavutil.mavlink_connection(f'udpin:0.0.0.0:{args.port}')
print('pilot: waiting for heartbeat...', flush=True)
m.wait_heartbeat()
print(f'pilot: heartbeat from sys {m.target_system}', flush=True)

stick = {'x': 0, 'y': 0, 'z': 500, 'r': 0, 'on': True}
t_start = time.monotonic()


def rc_loop():
    while True:
        if args.drop_rc_at > 0 and time.monotonic() - t_start > args.drop_rc_at:
            if stick['on']:
                print('pilot: DROPPING RC (simulated link loss)', flush=True)
                stick['on'] = False
        if stick['on']:
            m.mav.manual_control_send(
                m.target_system, stick['x'], stick['y'], stick['z'],
                stick['r'], 0)
        time.sleep(0.02)


threading.Thread(target=rc_loop, daemon=True).start()


def lpos():
    msg = m.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2)
    return msg


def nav_state():
    # Only the autopilot's heartbeat counts: AirSim's MAVLink node also
    # heartbeats on this link and reports main_mode=0/disarmed.
    msg = None
    for _ in range(8):
        cand = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if cand is None:
            break
        if cand.get_srcSystem() == m.target_system and \
                cand.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            msg = cand
            break
    if msg is None:
        return None, False
    armed = bool(msg.base_mode
                 & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return msg.custom_mode, armed


def set_mode(main_mode, sub_mode=0):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        main_mode, sub_mode, 0, 0, 0, 0)


# (a) wait for a finite, quiet local position (EKF2 origin ready)
print('pilot: waiting for stable LOCAL_POSITION_NED...', flush=True)
stable = 0
while stable < 25:
    p = lpos()
    if p is not None and abs(p.x) < 50 and abs(p.y) < 50 and abs(p.z) < 50 \
            and abs(p.vx) < 1 and abs(p.vy) < 1 and abs(p.vz) < 1:
        stable += 1
    else:
        stable = 0
    time.sleep(0.05)
print('pilot: EKF2 position stable', flush=True)

# (b) Position mode (POSCTL = main_mode 3)
set_mode(3)
time.sleep(1.0)

# (c) arm via stick gesture: throttle low, yaw full right, 2.5 s
print('pilot: arm gesture (z=0, r=+1000)...', flush=True)
stick.update(z=0, r=1000)
t0 = time.monotonic()
armed = False
while time.monotonic() - t0 < 6.0:
    _, armed = nav_state()
    if armed:
        break
    time.sleep(0.2)
stick.update(z=500, r=0)
if not armed:
    print('pilot: gesture arm FAILED — falling back to COMPONENT_ARM_DISARM',
          flush=True)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(1.5)
    _, armed = nav_state()
print(f'pilot: armed={armed} (gesture={not armed or "yes"})', flush=True)

# (d) stick takeoff: throttle up until at altitude, then hover
print(f'pilot: stick takeoff to {args.alt} m...', flush=True)
stick.update(z=800)
t0 = time.monotonic()
while time.monotonic() - t0 < 30.0:
    p = lpos()
    if p is not None and p.z <= -args.alt:
        break
    time.sleep(0.1)
stick.update(z=500)
print('pilot: hovering (stick-held Position mode)', flush=True)
time.sleep(5.0)

if args.no_offboard:
    print('pilot: --no-offboard: holding Position hover indefinitely', flush=True)
    while True:
        time.sleep(5)
        p = lpos()
        if p:
            print(f'pilot: hover z={p.z:.2f}', flush=True)

# (e) Offboard only after the guard's setpoint stream is live: the guard
# heartbeats offboard_control_mode unconditionally, so a short settle wait
# mirrors the pilot checking the tablet before flipping the switch.
time.sleep(3.0)
print('pilot: switching to OFFBOARD', flush=True)
set_mode(6)

# (f) keep streaming centered sticks; report nav_state transitions
last = None
while True:
    cm, armed = nav_state()
    if cm is not None:
        main = (cm >> 16) & 0xFF
        if main != last:
            print(f'pilot: main_mode={main} armed={armed} '
                  f't={time.monotonic() - t_start:.0f}s', flush=True)
            last = main
    time.sleep(0.5)
