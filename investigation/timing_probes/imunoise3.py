import sys,time,numpy as np,threading
sys.path.insert(0,'/home/lucas/hercules-sim/HERCULES/PythonClient')
import hercules_cosysairsim as airsim
NAMES=["ghost","delta","buckshee","thunderstrike"]; V="ghost"
c=airsim.MultirotorClient(); c.confirmConnection()
c.enableApiControl(True,V); c.armDisarm(True,V)
c.takeoffAsync(vehicle_name=V).join(); c.moveToZAsync(-6,2,vehicle_name=V).join()
c.hoverAsync(vehicle_name=V).join(); time.sleep(3)
S=[]; t_end=time.time()+12
while time.time()<t_end:
    d=c.getImuData("imu",V)
    S.append((d.time_stamp,d.angular_velocity.x_val,d.angular_velocity.y_val,d.angular_velocity.z_val,
              d.linear_acceleration.x_val,d.linear_acceleration.y_val,d.linear_acceleration.z_val))
try:
    c.landAsync(vehicle_name=V).join(); c.armDisarm(False,V); c.enableApiControl(False,V)
except Exception: pass
A=np.array(S,float); _,ix=np.unique(A[:,0],return_index=True); A=A[np.sort(ix)]
dt=np.median(np.diff(A[:,0]))/1e9
print(f"HOVER: n={len(A)} rate={1/dt:.0f} Hz")
g=A[:,1:4]; a=A[:,4:7]
# high-frequency (white) component = 1/sqrt(2) * std of first difference
gw=np.diff(g,axis=0).std(0)/np.sqrt(2); aw=np.diff(a,axis=0).std(0)/np.sqrt(2)
print("gyro  white sigma (rad/s) :", np.array2string(gw,precision=6))
print("accel white sigma (m/s^2) :", np.array2string(aw,precision=6))
print(f"=> gyro_noise_density  (rad/s/sqrt(Hz)) = {np.array2string(gw*np.sqrt(dt),precision=8)}")
print(f"=> accel_noise_density (m/s^2/sqrt(Hz)) = {np.array2string(aw*np.sqrt(dt),precision=8)}")
print("gyro  total std:", np.array2string(g.std(0),precision=6), " accel total std:", np.array2string(a.std(0),precision=6))
