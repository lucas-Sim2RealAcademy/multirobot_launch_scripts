#!/usr/bin/env python3
"""PHASE-2 (c) TEXTURE.  Same probe as investigation/feature_probe/measure_features.py,
pointed at any probe directory.

Reference points from that experiment, so these numbers are comparable:
  Blocks (untextured)  ST  23-215   ground features    0-39   (median 11)
  Blocks (texturized)  ST 126-1793
  JapanFest_Street     ST 923-1369  ground features  476-747  (median 552)

ST        Shi-Tomasi corners in the left image (cv2.goodFeaturesToTrack, 2000/0.01/8)
grndFeat  ST corners below the horizon row (y>240) -- the ones stereo VIO needs and the
          ones Blocks starves on
usable    fraction of LK-matched stereo correspondences with disparity > 2 px
"""
import glob
import json
import os
import sys

import cv2
import numpy as np

D = sys.argv[1]
LABEL = sys.argv[2] if len(sys.argv) > 2 else os.path.basename(D)
poses = sorted({os.path.basename(p)[:-6] for p in glob.glob(D + '/*_L.npy')})
fast = cv2.FastFeatureDetector_create(threshold=20, nonmaxSuppression=True)


def cells(p, gx=8, gy=6):
    if len(p) == 0:
        return 0
    return len(set(zip(np.clip((p[:, 0] / 640 * gx).astype(int), 0, gx - 1).tolist(),
                       np.clip((p[:, 1] / 480 * gy).astype(int), 0, gy - 1).tolist())))


print("%-12s|%5s %6s %9s|%8s %9s %13s|%9s %10s" %
      ('pose', 'ST', 'FAST', 'cells/48', 'LKmatch', 'medDisp', 'usable(>2px)',
       'grndFeat', 'grndDepth'))
res = []
for l in poses:
    L = np.load('%s/%s_L.npy' % (D, l))
    R = np.load('%s/%s_R.npy' % (D, l))
    dp = np.load('%s/%s_depth.npy' % (D, l))
    st = cv2.goodFeaturesToTrack(L, 2000, 0.01, 8)
    p = st.reshape(-1, 2) if st is not None else np.zeros((0, 2), np.float32)
    n = len(p)
    ntrk, med, f2 = 0, 0.0, 0.0
    if n:
        q, s, _ = cv2.calcOpticalFlowPyrLK(L, R, p.astype(np.float32), None,
                                           winSize=(21, 21), maxLevel=3)
        m = s.reshape(-1) == 1
        disp = p[m, 0] - q.reshape(-1, 2)[m, 0]
        disp = disp[(disp > -2) & (disp < 300)]
        ntrk = len(disp)
        if ntrk:
            med = float(np.median(disp))
            f2 = float((disp > 2).mean() * 100)
    grnd = int((p[:, 1] > 240).sum()) if n else 0
    gd = float(np.median(dp[300:470, 200:440]))
    print("%-12s|%5d %6d %9d|%8d %9.2f %12.0f%%|%9d %9.2fm" %
          (l, n, len(fast.detect(L, None)), cells(p), ntrk, med, f2, grnd, gd))
    res.append(dict(pose=l, st=n, fast=len(fast.detect(L, None)), cells=cells(p),
                    lk=ntrk, meddisp=med, usable_gt2px=f2, grnd=grnd, grndZ=gd))

if res:
    stv = sorted(r['st'] for r in res)
    gv = sorted(r['grnd'] for r in res)
    summ = dict(label=LABEL, n_poses=len(res),
                st_min=stv[0], st_med=stv[len(stv) // 2], st_max=stv[-1],
                grnd_min=gv[0], grnd_med=gv[len(gv) // 2], grnd_max=gv[-1])
    print("\nSUMMARY %s  ST %d-%d (med %d)   groundFeat %d-%d (med %d)" %
          (LABEL, summ['st_min'], summ['st_max'], summ['st_med'],
           summ['grnd_min'], summ['grnd_max'], summ['grnd_med']))
    json.dump(dict(summary=summ, per_pose=res),
              open('%s/features_%s.json' % (D, LABEL), 'w'), indent=1)
