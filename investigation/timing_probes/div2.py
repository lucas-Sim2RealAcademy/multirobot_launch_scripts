import re,glob,json
import numpy as np
base="/home/lucas/hercules-sim/e1_frames"
pat=re.compile(r'\[INFO\] \[(\d+\.\d+)\].*pos=\(([-\d.]+),([-\d.]+)\)')
for v in ["ghost","delta","buckshee","thunderstrike"]:
    P=[]
    for L in open(f"{base}/planner_radio4_{v}.log",errors="ignore"):
        m=pat.search(L)
        if m: P.append((float(m.group(1)),float(m.group(2)),float(m.group(3))))
    G=[]
    for f in sorted(glob.glob(f"{base}/radio4_{v}/meta_*.json")):
        try:
            d=json.load(open(f)); n=d['ned']; G.append((d['t'],n[0],n[1]))
        except Exception: pass
    if not P or not G: continue
    G=np.array(G,float); P=np.array(P,float)
    # meta 't' is elapsed seconds since bridge start; align to planner wall clock by
    # matching the first planner entry to the earliest gt after takeoff
    off=P[0,0]-G[0,0]
    gt_t=G[:,0]+off
    print(f"== {v}")
    print(f"   {'t_rel':>6} {'|belief|':>9} {'|gt|':>7} {'gap':>8}")
    for frac in [0.1,0.25,0.4,0.55,0.7,0.85,1.0]:
        i=min(int(frac*(len(P)-1)),len(P)-1)
        t=P[i,0]; b=np.hypot(P[i,1],P[i,2])
        j=int(np.argmin(np.abs(gt_t-t))); g=np.hypot(G[j,1],G[j,2])
        print(f"   {t-P[0,0]:6.0f} {b:9.1f} {g:7.1f} {b-g:8.1f}")
