#!/usr/bin/env python3
"""Fly the ghost pawn through Blocks on a frontier-exploration-like pattern
while capturing chase cam + front RGB (20 deg down) + depth vis frames.

Frames -> /home/lucas/hercules-sim/video_frames/{chase,front,depth}_NNNNN.png
Timing -> capture_meta.txt (measured fps for ffmpeg assembly)
"""
import sys, time, os

sys.path.insert(0, '/home/lucas/hercules-sim/HERCULES/PythonClient')
import hercules_cosysairsim as airsim

OUT = '/home/lucas/hercules-sim/video_frames'
os.makedirs(OUT, exist_ok=True)

VEH = 'ghost'
H = -2.5          # cruise altitude (m, NED)
V = 3.0           # m/s

# perimeter sweep around the Blocks obstacle field, then a high overflight
# across the middle: visually interesting and collision-free
WAYPOINTS = [
    (16, 0, H), (16, 14, H), (-16, 14, H), (-16, -14, H),
    (16, -14, H), (16, 0, H), (16, 0, -9.0), (-16, 0, -9.0),
    (0, 0, -4.0), (0, 0, H),
]

REQS = [
    airsim.ImageRequest('chase', airsim.ImageType.Scene, False, True),
    airsim.ImageRequest('front_center', airsim.ImageType.Scene, False, True),
    airsim.ImageRequest('front_center', airsim.ImageType.DepthVis, False, True),
]

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, VEH)
client.armDisarm(True, VEH)

print('takeoff...', flush=True)
client.takeoffAsync(timeout_sec=20, vehicle_name=VEH).join()
client.moveToZAsync(H, 1.0, vehicle_name=VEH).join()

n = 0
t0 = time.time()
for (x, y, z) in WAYPOINTS:
    print(f'-> wp ({x},{y},{z})', flush=True)
    client.moveToPositionAsync(x, y, z, V, drivetrain=airsim.DrivetrainType.ForwardOnly,
                               yaw_mode=airsim.YawMode(False, 0), vehicle_name=VEH)
    wp_t0 = time.time()
    last_pos, last_move_t = None, time.time()
    while True:
        resp = client.simGetImages(REQS, vehicle_name=VEH)
        for tag, r in zip(('chase', 'front', 'depth'), resp):
            with open(f'{OUT}/{tag}_{n:05d}.png', 'wb') as f:
                f.write(r.image_data_uint8)
        n += 1
        pos = client.getMultirotorState(vehicle_name=VEH).kinematics_estimated.position
        if (pos.x_val - x) ** 2 + (pos.y_val - y) ** 2 + (pos.z_val - z) ** 2 < 1.5 ** 2:
            break
        if last_pos is not None:
            moved = ((pos.x_val - last_pos[0]) ** 2 + (pos.y_val - last_pos[1]) ** 2
                     + (pos.z_val - last_pos[2]) ** 2) ** 0.5
            if moved > 0.3:
                last_move_t = time.time()
        last_pos = (pos.x_val, pos.y_val, pos.z_val)
        if time.time() - last_move_t > 6:
            print('   stuck (no motion 6 s) -> next waypoint', flush=True)
            break
        if time.time() - wp_t0 > 40:
            print('   waypoint timeout -> next', flush=True)
            break

dt = time.time() - t0
fps = n / dt if dt > 0 else 1
print(f'captured {n} frames in {dt:.1f}s -> {fps:.2f} fps', flush=True)
with open(f'{OUT}/capture_meta.txt', 'w') as f:
    f.write(f'{n} {dt:.3f} {fps:.3f}\n')

# prove the rest of the sensor suite responds
dist = client.getDistanceSensorData('lidar_down', VEH)
imu = client.getImuData('imu', VEH)
print(f'lidar_down: {dist.distance:.2f} m  (commanded AGL {-H:.1f} m)', flush=True)
print(f'imu accel z: {imu.linear_acceleration.z_val:.2f} m/s^2', flush=True)

client.landAsync(vehicle_name=VEH).join()
client.armDisarm(False, VEH)
client.enableApiControl(False, VEH)
print('done', flush=True)
