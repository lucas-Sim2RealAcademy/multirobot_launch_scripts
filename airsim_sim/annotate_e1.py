#!/usr/bin/env python3
"""Annotate E1 frames (chase + front + meta sidecar) into composite video
frames: chase on top, front camera bottom-left, live status panel
bottom-right (planner state + last planner log line)."""
import glob
import json
import os
import sys

from PIL import Image, ImageDraw, ImageFont

RUN = sys.argv[1]                      # e.g. prefix | tip
LABEL = sys.argv[2]                    # banner text
SRC = f'/home/lucas/hercules-sim/e1_frames/{RUN}'
DST = f'/home/lucas/hercules-sim/e1_frames/{RUN}_annot'
os.makedirs(DST, exist_ok=True)

STATE_COLORS = {'INIT': (150, 150, 150), 'PLAN': (255, 180, 40),
                'EXECUTE': (60, 200, 90), 'DONE': (80, 160, 255)}

try:
    FONT = ImageFont.truetype(
        '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 17)
    FONT_BIG = ImageFont.truetype(
        '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 22)
except Exception:
    FONT = FONT_BIG = ImageFont.load_default()


def wrap(text, width=44):
    out, line = [], ''
    for w in text.split():
        if len(line) + len(w) + 1 > width:
            out.append(line)
            line = w
        else:
            line = (line + ' ' + w).strip()
    if line:
        out.append(line)
    return out[:6]


frames = sorted(glob.glob(f'{SRC}/chase_*.png'))
for i, cp in enumerate(frames):
    idx = os.path.basename(cp)[6:11]
    fp = f'{SRC}/front_{idx}.png'
    mp = f'{SRC}/meta_{idx}.json'
    if not (os.path.exists(fp) and os.path.exists(mp)):
        continue
    meta = json.load(open(mp))
    chase = Image.open(cp).convert('RGB').resize((960, 540))
    front = Image.open(fp).convert('RGB').resize((480, 360))

    canvas = Image.new('RGB', (960, 900), (12, 12, 16))
    canvas.paste(chase, (0, 0))
    canvas.paste(front, (0, 540))
    d = ImageDraw.Draw(canvas)

    # banner
    d.rectangle([0, 0, 960, 34], fill=(0, 0, 0))
    d.text((10, 6), LABEL, font=FONT_BIG, fill=(255, 255, 255))

    # status panel
    px, py = 490, 548
    state = meta.get('state', '?')
    col = STATE_COLORS.get(state, (200, 200, 200))
    d.rectangle([px - 6, py - 4, 954, 894], outline=(70, 70, 80), width=1)
    d.text((px, py), 'PLANNER STATE', font=FONT, fill=(160, 160, 170))
    d.rectangle([px, py + 26, px + 150, py + 52], fill=col)
    d.text((px + 8, py + 29), state, font=FONT_BIG, fill=(0, 0, 0))
    ned = meta.get('ned', [0, 0, 0, 0])
    d.text((px, py + 64),
           f'NED  x={ned[0]:+.1f}  y={ned[1]:+.1f}  z={ned[2]:+.1f}',
           font=FONT, fill=(200, 200, 210))
    d.text((px, py + 96), 'LAST PLANNER LOG', font=FONT, fill=(160, 160, 170))
    log = meta.get('log', '') or '(none)'
    warn = 'NO PATH' in log
    for k, line in enumerate(wrap(log)):
        d.text((px, py + 122 + 22 * k), line, font=FONT,
               fill=(255, 90, 80) if warn else (220, 220, 225))
    d.text((0, 878), ' front camera (RGB, 20deg down)', font=FONT,
           fill=(160, 160, 170))

    canvas.save(f'{DST}/f_{i:05d}.png')

print(f'{RUN}: annotated {len(frames)} frames -> {DST}')
