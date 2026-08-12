#!/usr/bin/env python3
"""E1 harness bridge: AirSim <-> real simple_exploration_planner.

Feeds the planner exactly what it consumes on the drone:
  - visual_slam/tracking/odometry   (FLU odom, from AirSim ground truth)
  - /fmu/out/vehicle_local_position (NED, PX4 QoS)
  - /fmu/out/vehicle_status         (armed + offboard, PX4 QoS)
  - nvblox_node/static_esdf_pointcloud
        flat-open-field ESDF: every known cell's intensity = distance to
        the GROUND (1.2 m) -- the phantom-obstacle condition from the
        2026-08-12 field test (walls=0)
  - fis/frontier_{centroids,viewpoints,gains}

Flies the planner's output: planning/trajectory_setpoint -> AirSim.
Captures chase+front frames with a per-frame sidecar (state + last
planner log line) for the annotated A/B video.
"""
import json
import math
import os
import struct
import sys
import threading
import time

sys.path.insert(0, '/home/lucas/hercules-sim/HERCULES/PythonClient')
import hercules_cosysairsim as airsim

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64MultiArray, String
from std_msgs.msg import Header
from rcl_interfaces.msg import Log
from px4_msgs.msg import (TrajectorySetpoint, VehicleLocalPosition,
                          VehicleStatus)

OUT = os.environ.get('E1_OUT', '/home/lucas/hercules-sim/e1_frames/run')
RUN_SECONDS = float(os.environ.get('E1_SECONDS', '110'))
VEH = 'ghost'
FLIGHT_Z = -1.0
GROUND_DIST = float(os.environ.get('E1_GROUND', '1.2'))  # phantom floor distance
KNOWN_RADIUS = 9.5         # known-cell disc radius (m)
VOXEL = 0.2
FRONTIERS = [(6.0, 0.0), (0.0, 6.0), (-6.0, 0.0), (0.0, -6.0),
             (5.0, -5.0), (-5.0, 5.0)]


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class E1Bridge(Node):
    def __init__(self, client):
        super().__init__('e1_sim_bridge')
        self.client = client
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.odom_pub = self.create_publisher(
            Odometry, 'visual_slam/tracking/odometry', 10)
        self.vlp_pub = self.create_publisher(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', px4_qos)
        self.vstat_pub = self.create_publisher(
            VehicleStatus, '/fmu/out/vehicle_status', px4_qos)
        self.esdf_pub = self.create_publisher(
            PointCloud2, 'nvblox_node/static_esdf_pointcloud', 5)
        self.cent_pub = self.create_publisher(
            PoseArray, 'fis/frontier_centroids', 10)
        self.vp_pub = self.create_publisher(
            PoseArray, 'fis/frontier_viewpoints', 10)
        self.gain_pub = self.create_publisher(
            Float64MultiArray, 'fis/frontier_gains', 10)

        self.create_subscription(
            TrajectorySetpoint, 'planning/trajectory_setpoint',
            self._sp_cb, 10)
        self.create_subscription(String, 'exploration/state',
                                 self._state_cb, 10)
        self.create_subscription(Log, '/rosout', self._rosout_cb, 50)

        self.sp = None
        self.sp_time = 0.0
        self.state = 'INIT'
        self.last_log = ''
        self.ned = (0.0, 0.0, 0.0, 0.0)  # x y z yaw

        self.create_timer(0.05, self._poll_airsim)   # 20 Hz state out
        self.create_timer(0.5, self._pub_esdf_fis)   # 2 Hz map/frontiers
        self.create_timer(0.5, self._apply_sp)       # 2 Hz flight command

        self._esdf_msg = self._build_esdf()

    # ---- AirSim state -> planner inputs ----
    def _poll_airsim(self):
        try:
            st = self.client.getMultirotorState(vehicle_name=VEH)
        except Exception:
            return
        p = st.kinematics_estimated.position
        q = st.kinematics_estimated.orientation
        yaw_ned = math.atan2(
            2.0 * (q.w_val * q.z_val + q.x_val * q.y_val),
            1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val))
        self.ned = (p.x_val, p.y_val, p.z_val, yaw_ned)
        now = self.get_clock().now().to_msg()

        od = Odometry()
        od.header = Header(stamp=now, frame_id='odom')
        od.pose.pose.position.x = p.x_val         # FLU x = NED x
        od.pose.pose.position.y = -p.y_val        # FLU y = -NED y
        od.pose.pose.position.z = -p.z_val
        qx, qy, qz, qw = yaw_to_quat(-yaw_ned)    # FLU yaw = -NED yaw
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        self.odom_pub.publish(od)

        vlp = VehicleLocalPosition()
        vlp.x, vlp.y, vlp.z = p.x_val, p.y_val, p.z_val
        vlp.heading = yaw_ned
        vlp.xy_valid = vlp.z_valid = vlp.v_xy_valid = vlp.v_z_valid = True
        self.vlp_pub.publish(vlp)

        vs = VehicleStatus()
        vs.arming_state = 2    # ARMED
        vs.nav_state = 14      # OFFBOARD
        self.vstat_pub.publish(vs)

    # ---- synthetic flat-field ESDF + static frontier set ----
    def _build_esdf(self):
        # Emit points at the planner's exact grid-cell centers
        # (bbox_min + (i+0.5)*voxel) so every cell in the disc rasterizes
        # KNOWN — matching the field condition (whole area mapped, raw ESDF
        # = distance-to-ground everywhere, zero walls).
        pts = []
        bbox_min = -10.0
        ncells = 101
        for i in range(ncells):
            for j in range(ncells):
                x = bbox_min + (i + 0.5) * VOXEL
                y = bbox_min + (j + 0.5) * VOXEL
                if x * x + y * y <= KNOWN_RADIUS * KNOWN_RADIUS:
                    pts.append(struct.pack('<ffff', x, y, 0.0, GROUND_DIST))
        msg = PointCloud2()
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * len(pts)
        msg.data = b''.join(pts)
        msg.is_dense = True
        return msg

    def _pub_esdf_fis(self):
        now = self.get_clock().now().to_msg()
        self._esdf_msg.header = Header(stamp=now, frame_id='odom')
        self.esdf_pub.publish(self._esdf_msg)

        cents, vps = PoseArray(), PoseArray()
        cents.header = vps.header = Header(stamp=now, frame_id='odom')
        gains = Float64MultiArray()
        for (fx, fy) in FRONTIERS:
            c = Pose()
            c.position.x, c.position.y = fx, fy
            cents.poses.append(c)
            v = Pose()
            v.position.x, v.position.y = fx * 0.9, fy * 0.9
            yaw = math.atan2(fy, fx)
            _, _, v.orientation.z, v.orientation.w = (0, 0) + yaw_to_quat(yaw)[2:]
            vps.poses.append(v)
            gains.data.append(10.0)
        self.cent_pub.publish(cents)
        self.vp_pub.publish(vps)
        self.gain_pub.publish(gains)

    # ---- planner output -> AirSim flight ----
    def _sp_cb(self, msg):
        self.sp = msg
        self.sp_time = time.time()

    def _apply_sp(self):
        if self.sp is None or (time.time() - self.sp_time) > 2.0:
            return
        px, py, pz = float(self.sp.position[0]), float(self.sp.position[1]), \
            float(self.sp.position[2])
        if not (math.isfinite(px) and math.isfinite(py)):
            return
        if not math.isfinite(pz):
            pz = FLIGHT_Z
        yaw_deg = math.degrees(self.sp.yaw) if math.isfinite(self.sp.yaw) else 0.0
        try:
            self.client.moveToPositionAsync(
                px, py, pz, 1.5,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(False, yaw_deg), vehicle_name=VEH)
        except Exception:
            pass

    # ---- annotation feeds ----
    def _state_cb(self, msg):
        self.state = msg.data

    def _rosout_cb(self, msg):
        if msg.name == 'simple_exploration_planner':
            self.last_log = msg.msg[:150]


def capture_loop(bridge, client, stop_evt):
    os.makedirs(OUT, exist_ok=True)
    reqs = [
        airsim.ImageRequest('chase', airsim.ImageType.Scene, False, True),
        airsim.ImageRequest('front_center', airsim.ImageType.Scene, False, True),
    ]
    n = 0
    while not stop_evt.is_set():
        try:
            resp = client.simGetImages(reqs, vehicle_name=VEH)
            for tag, r in zip(('chase', 'front'), resp):
                with open(f'{OUT}/{tag}_{n:05d}.png', 'wb') as f:
                    f.write(r.image_data_uint8)
            meta = {'t': time.time(), 'state': bridge.state,
                    'log': bridge.last_log,
                    'ned': [round(v, 2) for v in bridge.ned]}
            with open(f'{OUT}/meta_{n:05d}.json', 'w') as f:
                json.dump(meta, f)
            n += 1
        except Exception:
            pass
        time.sleep(0.12)
    print(f'captured {n} frames', flush=True)


def main():
    # separate RPC clients: control vs capture
    ctl = airsim.MultirotorClient()
    ctl.confirmConnection()
    for attempt in range(4):
        ctl.enableApiControl(True, VEH)
        ctl.armDisarm(True, VEH)
        time.sleep(1.0)
        ctl.takeoffAsync(timeout_sec=12, vehicle_name=VEH).join()
        time.sleep(0.5)
        z = ctl.getMultirotorState(VEH).kinematics_estimated.position.z_val
        if z < -0.4:
            break
        print(f'takeoff attempt {attempt + 1} failed (z={z:.2f}), retrying',
              flush=True)
    ctl.moveToZAsync(FLIGHT_Z, 1.0, vehicle_name=VEH)
    t0 = time.time()
    while time.time() - t0 < 15:
        z = ctl.getMultirotorState(VEH).kinematics_estimated.position.z_val
        if abs(z - FLIGHT_Z) < 0.3:
            break
        time.sleep(0.3)
    print('airborne', flush=True)

    cap = airsim.MultirotorClient()
    cap.confirmConnection()

    rclpy.init()
    bridge = E1Bridge(ctl)
    stop_evt = threading.Event()
    cap_thread = threading.Thread(target=capture_loop,
                                  args=(bridge, cap, stop_evt), daemon=True)
    cap_thread.start()

    t_end = time.time() + RUN_SECONDS
    try:
        while time.time() < t_end:
            rclpy.spin_once(bridge, timeout_sec=0.05)
    finally:
        stop_evt.set()
        cap_thread.join(timeout=5)
        try:
            ctl.landAsync(vehicle_name=VEH).join()
            ctl.armDisarm(False, VEH)
            ctl.enableApiControl(False, VEH)
        except Exception:
            pass
        bridge.destroy_node()
        rclpy.shutdown()
    print('bridge done', flush=True)


if __name__ == '__main__':
    main()
