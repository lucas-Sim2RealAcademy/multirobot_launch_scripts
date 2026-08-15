import re,sys,statistics as st
pat=re.compile(r'\[WARN\] \[(\d+\.\d+)\] \[visual_slam\]: Delta between current and previous frame \[([\d.]+) ms\]')
for v in ["ghost","delta","buckshee","thunderstrike"]:
    rows=[]
    for line in open(f"/home/lucas/hercules-sim/e1_frames/cuvslam_radio4_{v}.log",errors="ignore"):
        m=pat.search(line)
        if m: rows.append((float(m.group(1)),float(m.group(2))))
    if not rows: continue
    d=[r[1] for r in rows]
    t0,t1=rows[0][0],rows[-1][0]
    dur=t1-t0
    print(f"=== {v}: n={len(rows)} span={dur:.1f}s  rate={len(rows)/dur*60:.0f}/min")
    ds=sorted(d)
    def q(p): return ds[int(p*(len(ds)-1))]
    print(f"   delta ms: min={min(d):.1f} p50={q(.5):.1f} p90={q(.9):.1f} p99={q(.99):.1f} max={max(d):.1f} mean={st.mean(d):.1f}")
    # buckets
    b={ '34-50':0,'50-70':0,'70-100':0,'100-200':0,'200-500':0,'>500':0}
    for x in d:
        if x<50:b['34-50']+=1
        elif x<70:b['50-70']+=1
        elif x<100:b['70-100']+=1
        elif x<200:b['100-200']+=1
        elif x<500:b['200-500']+=1
        else:b['>500']+=1
    print("   ",b)
    # total time accounted for by >34ms gaps
    print(f"   time in >34ms gaps: {sum(d)/1000:.1f}s of {dur:.1f}s = {sum(d)/1000/dur*100:.0f}%")
    # wall-arrival gap vs reported delta mismatch
    mism=[]
    for i in range(1,len(rows)):
        wall=(rows[i][0]-rows[i-1][0])*1000
        if wall < rows[i][1]-1.0:   # stamp delta exceeds wall arrival gap
            mism.append(rows[i][1]-wall)
    print(f"   frames where stamp-delta > wall-arrival-gap: {len(mism)} ({len(mism)/len(rows)*100:.0f}%), median excess {st.median(mism) if mism else 0:.1f} ms, max {max(mism) if mism else 0:.1f} ms")
