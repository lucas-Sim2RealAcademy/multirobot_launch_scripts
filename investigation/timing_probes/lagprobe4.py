# Offset-invariant estimate: cross-correlate the camera_position(y) series carried in the
# ImageResponse (sampled on the GAME thread at true capture) against the ground-truth y
# series, both indexed by the RENDER-thread time_stamp / physics time_stamp.
import sys, time, threading
sys.path.insert(0,'/home/lucas/hercules-sim/HERCULES/PythonClient')
import hercules_cosysairsim as airsim
import numpy as np
NAMES=["ghost","delta","buckshee","thunderstrike"]; stop=threading.Event(); VEH="ghost"
def load_worker(veh):
    c=airsim.MultirotorClient(); c.confirmConnection()
    reqs=[airsim.ImageRequest("front_left",airsim.ImageType.Scene,False,False),
          airsim.ImageRequest("front_right",airsim.ImageType.Scene,False,False)]
    d=[airsim.ImageRequest("front_center",airsim.ImageType.DepthPlanar,True,False)]
    while not stop.is_set():
        try: c.simGetImages(reqs,vehicle_name=veh); time.sleep(0.03); c.simGetImages(d,vehicle_name=veh); time.sleep(0.03)
        except Exception: pass
gt=[]
def gt_worker():
    c=airsim.MultirotorClient(); c.confirmConnection()
    while not stop.is_set():
        try:
            k=c.simGetGroundTruthKinematics(vehicle_name=VEH); d=c.getImuData("imu",VEH)
            gt.append((d.time_stamp,k.position.y_val))
        except Exception: pass
        pass
samples=[]
def cam_worker():
    cc=airsim.MultirotorClient(); cc.confirmConnection()
    reqs=[airsim.ImageRequest("front_left",airsim.ImageType.Scene,False,False),
          airsim.ImageRequest("front_right",airsim.ImageType.Scene,False,False)]
    while not stop.is_set():
        try:
            r=cc.simGetImages(reqs,vehicle_name=VEH)
            if len(r)>=2: samples.append((r[0].time_stamp,r[0].camera_position.y_val))
        except Exception: pass
        time.sleep(0.02)
c=airsim.MultirotorClient(); c.confirmConnection()
c.enableApiControl(True,VEH); c.armDisarm(True,VEH)
c.takeoffAsync(vehicle_name=VEH).join(); c.moveToZAsync(-6,2,vehicle_name=VEH).join()
mode=sys.argv[1]
if mode=="quad":
    for v in NAMES[1:]: threading.Thread(target=load_worker,args=(v,),daemon=True).start()
threading.Thread(target=gt_worker,daemon=True).start(); threading.Thread(target=cam_worker,daemon=True).start()
time.sleep(1.0)
for k in range(20):
    c.moveByVelocityZAsync(0, 3.0 if k%2==0 else -3.0, -6, 1.5, vehicle_name=VEH).join()
stop.set(); time.sleep(0.4)
try:
    c.hoverAsync(vehicle_name=VEH).join(); c.landAsync(vehicle_name=VEH)
    c.armDisarm(False,VEH); c.enableApiControl(False,VEH)
except Exception: pass
g=np.array(gt,float); s=np.array(samples,float)
g=g[np.argsort(g[:,0])]; _,ix=np.unique(g[:,0],return_index=True); g=g[ix]
s=s[np.argsort(s[:,0])]
print(f"mode={mode} gt n={len(g)} cam n={len(s)}")
if len(g)<200 or len(s)<40: sys.exit()
t0=max(g[0,0],s[0,0])+0.5e9; t1=min(g[-1,0],s[-1,0])-0.5e9
grid=np.arange(t0,t1,2e6)          # 2 ms grid
gy=np.interp(grid,g[:,0],g[:,1]); gy-=gy.mean()
best=None
for lag_ms in np.arange(-40,300,1.0):
    cy=np.interp(grid, s[:,0]-lag_ms*1e6, s[:,1]); cy=cy-cy.mean()
    r=float(np.corrcoef(gy,cy)[0,1])
    if best is None or r>best[1]: best=(lag_ms,r)
print(f"  best lag = {best[0]:.0f} ms  (corr {best[1]:.4f})   <- stamp is LATE by this vs image content")
