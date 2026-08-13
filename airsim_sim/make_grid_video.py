#!/usr/bin/env python3
"""Compose a 2x2 fleet grid video: one pane per drone (chase cam + live
planner state + last planner log), from a run's per-drone capture dirs."""
import glob
import json
import os
import sys

from PIL import Image, ImageDraw, ImageFont

LABEL = sys.argv[1] if len(sys.argv) > 1 else 'fast4'
BASE = '/home/lucas/hercules-sim/e1_frames'
DRONES = ['ghost', 'delta', 'buckshee', 'thunderstrike']
OUT = f'{BASE}/{LABEL}_grid'
os.makedirs(OUT, exist_ok=True)

PW, PH = 640, 400          # pane size
W, H = PW * 2, PH * 2 + 40  # + banner
STATE_COLORS = {'INIT': (150, 150, 150), 'PLAN': (255, 180, 40),
                'EXECUTE': (60, 200, 90), 'DONE': (80, 160, 255)}
F = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 14)
FB = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 20)
FN = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 16)

series = {}
for d in DRONES:
    frames = sorted(glob.glob(f'{BASE}/{LABEL}_{d}/chase_*.png'))
    series[d] = frames
    print(f'{d}: {len(frames)} frames')
n = min(len(v) for v in series.values())
if n == 0:
    sys.exit('missing frames for at least one drone')

for i in range(n):
    canvas = Image.new('RGB', (W, H), (10, 10, 14))
    d0 = ImageDraw.Draw(canvas)
    d0.text((12, 10), f'HERCULES 4-DRONE FLEET — field sensor rates '
                      f'(IMU 200 Hz/drone) — real cuVSLAM + nvblox + FIS + '
                      f'planner + coordination per drone',
            font=FN, fill=(255, 255, 255))
    for k, drone in enumerate(DRONES):
        fp = series[drone][i]
        idx = os.path.basename(fp)[6:11]
        mp = f'{BASE}/{LABEL}_{drone}/meta_{idx}.json'
        img = Image.open(fp).convert('RGB').resize((PW, PH - 46))
        ox, oy = (k % 2) * PW, 40 + (k // 2) * PH
        canvas.paste(img, (ox, oy))
        dr = ImageDraw.Draw(canvas)
        meta = json.load(open(mp)) if os.path.exists(mp) else {}
        state = meta.get('state', '?')
        ned = meta.get('ned', [0, 0, 0, 0])
        col = STATE_COLORS.get(state, (200, 200, 200))
        by = oy + PH - 46
        dr.rectangle([ox, by, ox + PW, by + 46], fill=(18, 18, 24))
        dr.text((ox + 8, by + 5), drone.upper(), font=FB, fill=(230, 230, 240))
        dr.rectangle([ox + 150, by + 6, ox + 150 + 88, by + 28], fill=col)
        dr.text((ox + 156, by + 8), state, font=FN, fill=(0, 0, 0))
        dr.text((ox + 250, by + 9),
                f'NED {ned[0]:+.1f} {ned[1]:+.1f} {ned[2]:+.1f}',
                font=F, fill=(200, 200, 210))
        log = (meta.get('log') or '')[:78]
        warn = 'NO PATH' in log
        dr.text((ox + 8, by + 30), log, font=F,
                fill=(255, 90, 80) if warn else (170, 170, 180))
        dr.rectangle([ox, oy, ox + PW - 1, oy + PH - 1], outline=(60, 60, 70))
    canvas.save(f'{OUT}/g_{i:05d}.png')

print(f'wrote {n} grid frames -> {OUT}')
