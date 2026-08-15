#!/usr/bin/env python3
"""Re-tint the bagged /dN/mesh vertex colors so the merged map reads as a 3D
reconstruction (DEFECT 1 fix).

The recorded colors are the old FLAT per-drone tint (identity fill), which
destroyed the 3D read. This tool rewrites ONLY the ColorRGBA arrays inside the
CDR blobs, directly at the sqlite level -- vertex/normal/triangle data,
message sizes and timestamps are untouched, so replay timing (and the compose
scripts' offset math) is byte-identical.

New color = TURBO height colormap (z over [Z0,Z1]) * strong directional
Lambert shade with a subtle per-drone identity tint (TINT blend, default
0.18) applied to the base color BEFORE shading so shading contrast is never
diluted.

The bag has NO per-vertex normals (nvblox streamed empty normal arrays; the
old relay's nz fallback therefore shaded EVERY vertex with the same constant
-- the root cause of the flat look). Normals are recomputed here from the
bagged triangles: area-weighted face normals accumulated per vertex.

usage: retint_bag.py <src_bag_dir> <dst_bag_dir>
env: TINT (0..1, default 0.18), Z0 (-0.40), Z1 (3.60), AMBIENT (0.30),
     SHADE_GAIN (1-AMBIENT), TLO/THI turbo range compress (0.04/0.95)
"""
import os
import shutil
import sqlite3
import sys
import time

import numpy as np

TINT = float(os.environ.get("TINT", "0.18"))
Z0 = float(os.environ.get("Z0", "-0.40"))
Z1 = float(os.environ.get("Z1", "3.60"))
AMBIENT = float(os.environ.get("AMBIENT", "0.30"))
TLO = float(os.environ.get("TLO", "0.04"))
THI = float(os.environ.get("THI", "0.95"))

COLORS = {  # same identity hues as paths/markers
    "/d1/mesh": (0.25, 0.55, 1.00),   # ghost blue
    "/d2/mesh": (0.25, 1.00, 0.45),   # delta green
    "/d3/mesh": (1.00, 0.30, 0.95),   # buckshee magenta
    "/d4/mesh": (1.00, 0.58, 0.12),   # thunderstrike orange
}

# fairly grazing key light: the world is ~98% gently-undulating ground (bag
# audit), so a low light elevation is what makes the relief visible; walls of
# the boxes swing 0.30..0.79 by azimuth.
_LDIR = [float(v) for v in os.environ.get("LIGHT", "0.60,0.35,0.60").split(",")]
L = np.array(_LDIR, dtype=np.float32)
L /= np.linalg.norm(L)


def turbo(x):
    """Google turbo colormap polynomial fit, x in [0,1] -> (n,3) float32."""
    x = np.clip(x, 0.0, 1.0).astype(np.float32)
    r = 0.13572138 + x * (4.61539260 + x * (-42.66032258 + x * (
        132.13108234 + x * (-152.94239396 + x * 59.28637943))))
    g = 0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (
        -14.18503333 + x * (4.27729857 + x * 2.82956604))))
    b = 0.10667330 + x * (12.64194608 + x * (-60.58204836 + x * (
        110.36276771 + x * (-89.90310912 + x * 27.34824973))))
    return np.clip(np.stack([r, g, b], axis=1), 0.0, 1.0)


def vertex_normals(verts, tris):
    """Area-weighted per-vertex normals from (n,3) verts + (m,3) tri indices."""
    vn = np.zeros_like(verts)
    if len(tris):
        e1 = verts[tris[:, 1]] - verts[tris[:, 0]]
        e2 = verts[tris[:, 2]] - verts[tris[:, 0]]
        fn = np.cross(e1, e2)
        for c in range(3):
            np.add.at(vn, tris[:, c], fn)
    nl = np.linalg.norm(vn, axis=1)
    bad = nl < 1e-9
    vn[bad] = (0.0, 0.0, 1.0)
    nl[bad] = 1.0
    return vn / nl[:, None]


def compute_colors(verts, norms, rgb_tint):
    """verts,(n,3) unit norms,(n,3) -> (n,4) float32 RGBA."""
    t = (verts[:, 2] - Z0) / (Z1 - Z0)
    base = turbo(TLO + (THI - TLO) * np.clip(t, 0.0, 1.0))
    tintv = np.array(rgb_tint, dtype=np.float32)
    base = base * (1.0 - TINT) + tintv * TINT
    lam = np.clip(norms @ L, 0.0, 1.0)
    shade = (AMBIENT + (1.0 - AMBIENT) * lam).astype(np.float32)
    out = np.empty((len(verts), 4), dtype=np.float32)
    out[:, :3] = base * shade[:, None]
    out[:, 3] = 1.0
    return np.clip(out, 0.0, 1.0)


def u32(mv, off):
    return int.from_bytes(mv[off:off + 4], "little")


def patch_msg(data, rgb_tint):
    """Parse nvblox_msgs/Mesh CDR blob, rewrite each block's colors array."""
    b = bytearray(data)
    mv = memoryview(b)
    assert b[0] == 0x00 and b[1] == 0x01, "unexpected CDR encapsulation"
    off = 4
    off += 8                                   # stamp sec + nanosec
    slen = u32(mv, off)
    off += 4 + slen                            # frame_id (len incl NUL)
    off = 4 + (((off - 4) + 3) & ~3)           # align 4 (rel. to payload)
    off += 4                                   # block_size_m
    nbi = u32(mv, off)
    off += 4 + nbi * 12                        # Index3D int32 x3
    nblk = u32(mv, off)
    off += 4
    spans = []
    for _ in range(nblk):
        nv = u32(mv, off); voff = off + 4; off = voff + nv * 12
        nn = u32(mv, off); off = off + 4 + nn * 12
        nc = u32(mv, off); coff = off + 4; off = coff + nc * 16
        nt = u32(mv, off); toff = off + 4; off = toff + nt * 4
        assert nn in (0, nv) and nc in (0, nv), f"block arity {nv}/{nn}/{nc}"
        if nv:
            spans.append((voff, coff, toff, nv, nc, nt))
    assert off < len(b) <= off + 8, f"tail mismatch {off} vs {len(b)}"

    if not spans:
        return data
    # concatenate blocks; offset triangle indices into the big vertex array
    vlist, tlist = [], []
    base = 0
    for voff, coff, toff, nv, nc, nt in spans:
        vlist.append(np.frombuffer(b, "<f4", nv * 3, voff).reshape(-1, 3))
        if nt:
            tlist.append(np.frombuffer(b, "<i4", nt, toff).reshape(-1, 3) + base)
        base += nv
    verts = np.concatenate(vlist)
    tris = np.concatenate(tlist) if tlist else np.zeros((0, 3), np.int64)
    norms = vertex_normals(verts, tris)
    cols = compute_colors(verts, norms, rgb_tint)
    cur = 0
    for voff, coff, toff, nv, nc, nt in spans:
        if nc:
            mv[coff:coff + nv * 16] = cols[cur:cur + nv].tobytes()
        cur += nv
    return bytes(b)


def main():
    src, dst = sys.argv[1], sys.argv[2]
    if os.path.isdir(dst):
        shutil.rmtree(dst)
    os.makedirs(dst)
    for f in os.listdir(src):
        shutil.copy2(os.path.join(src, f), os.path.join(dst, f))
    db = [f for f in os.listdir(dst) if f.endswith(".db3")][0]
    con = sqlite3.connect(os.path.join(dst, db))
    cur = con.cursor()
    tids = {}
    for tid, name in cur.execute("SELECT id,name FROM topics"):
        if name in COLORS:
            tids[tid] = COLORS[name]
    print("mesh topic ids:", tids)
    t0 = time.time()
    ntot = 0
    for tid, rgb in tids.items():
        rows = cur.execute(
            "SELECT id,data FROM messages WHERE topic_id=?", (tid,)).fetchall()
        wcur = con.cursor()
        for i, (rid, data) in enumerate(rows):
            wcur.execute("UPDATE messages SET data=? WHERE id=?",
                         (patch_msg(data, rgb), rid))
            if i % 200 == 0:
                con.commit()
                print(f"  topic {tid}: {i}/{len(rows)} ({time.time()-t0:.0f}s)",
                      flush=True)
        con.commit()
        ntot += len(rows)
    con.commit()
    con.close()
    print(f"RETINT_OK {ntot} mesh msgs in {time.time()-t0:.0f}s -> {dst}")


if __name__ == "__main__":
    main()
