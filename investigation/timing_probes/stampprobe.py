import sys, time, threading, statistics as st
sys.path.insert(0,'/home/lucas/hercules-sim/HERCULES/PythonClient')
import hercules_cosysairsim as airsim
NAMES=["ghost","delta","buckshee","thunderstrike"]

def now_ns(): return time.time_ns()

res={}
def stereo_worker(veh, n, key):
    c=airsim.MultirotorClient(); c.confirmConnection()
    reqs=[airsim.ImageRequest("front_left",airsim.ImageType.Scene,False,False),
          airsim.ImageRequest("front_right",airsim.ImageType.Scene,False,False)]
    rows=[]
    for i in range(n):
        t0=now_ns()
        r=c.simGetImages(reqs, vehicle_name=veh)
        t1=now_ns()
        if len(r)<2: continue
        rows.append((t0,t1,r[0].time_stamp,r[1].time_stamp,
                     r[0].camera_position.x_val,r[0].camera_position.y_val,r[0].camera_position.z_val))
    res[key]=rows

def imu_worker(veh,n,key):
    c=airsim.MultirotorClient(); c.confirmConnection()
    rows=[]
    for i in range(n):
        t0=now_ns(); d=c.getImuData("imu",veh); t1=now_ns()
        rows.append((t0,t1,d.time_stamp))
        time.sleep(0.005)
    res[key]=rows

mode=sys.argv[1] if len(sys.argv)>1 else "single"
ths=[]
if mode=="single":
    ths=[threading.Thread(target=stereo_worker,args=("ghost",120,"s_ghost")),
         threading.Thread(target=imu_worker,args=("ghost",600,"i_ghost"))]
else:
    for v in NAMES:
        ths.append(threading.Thread(target=stereo_worker,args=(v,120,"s_"+v)))
        ths.append(threading.Thread(target=imu_worker,args=(v,600,"i_"+v)))
for t in ths: t.start()
for t in ths: t.join()

def summ(name,rows,kind):
    if not rows: return
    if kind=="s":
        lag_ret=[(t1-ts0)/1e6 for t0,t1,ts0,ts1,*_ in rows]      # stamp -> RPC return
        pre   =[(ts0-t0)/1e6 for t0,t1,ts0,ts1,*_ in rows]        # RPC call -> stamp
        skew  =[(ts1-ts0)/1e6 for t0,t1,ts0,ts1,*_ in rows]       # intra-batch left->right
        rtt   =[(t1-t0)/1e6 for t0,t1,*_ in rows]
        gaps  =[(rows[i][2]-rows[i-1][2])/1e6 for i in range(1,len(rows))]
        f=lambda a:f"med={st.median(a):7.2f} p90={sorted(a)[int(.9*(len(a)-1))]:7.2f} max={max(a):7.2f} min={min(a):7.2f}"
        print(f"[{name}] n={len(rows)}")
        print(f"   RPC rtt ms            {f(rtt)}")
        print(f"   call->stamp ms        {f(pre)}")
        print(f"   stamp->return ms      {f(lag_ret)}   <-- stamp age at delivery")
        print(f"   intra-batch skew L->R {f(skew)}")
        print(f"   stamp gap ms          {f(gaps)}  jitter sd={st.pstdev(gaps):.2f}")
    else:
        age=[(t1-ts)/1e6 for t0,t1,ts in rows]
        rtt=[(t1-t0)/1e6 for t0,t1,ts in rows]
        gaps=[(rows[i][2]-rows[i-1][2])/1e6 for i in range(1,len(rows))]
        gaps=[g for g in gaps if g>0]
        f=lambda a:f"med={st.median(a):7.2f} p90={sorted(a)[int(.9*(len(a)-1))]:7.2f} max={max(a):7.2f} min={min(a):7.2f}"
        print(f"[{name}] n={len(rows)}")
        print(f"   RPC rtt ms            {f(rtt)}")
        print(f"   IMU sample age ms     {f(age)}   <-- return_time - imu.time_stamp")
        print(f"   IMU stamp gap ms      {f(gaps)}")

for k,v in sorted(res.items()):
    summ(k,v,k[0])
