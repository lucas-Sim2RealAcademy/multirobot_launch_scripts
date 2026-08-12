#!/usr/bin/env python3
"""Real-perception sim bridge: AirSim -> depth + camera_info + TF for the
fork's real nvblox_ros; odom/PX4-status for the real FIS + planner.

Chain under test:  AirSim depth camera (87deg FOV, 20deg down)
  -> nvblox_node (fork, ekf2-nvblox-pose branch, CUDA)  [real]
  -> FIS frontier detection via get_esdf_and_gradient    [real]
  -> simple_exploration_planner (8/12 tip)               [real]
  -> planning/trajectory_setpoint -> AirSim flight
Only odometry is ground truth (cuVSLAM stand-in).
"""
import json
import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, '/home/lucas/hercules-sim/HERCULES/PythonClient')
import hercules_cosysairsim as airsim

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header, String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rcl_interfaces.msg import Log
from px4_msgs.msg import (TrajectorySetpoint, VehicleLocalPosition,
                          VehicleStatus)

OUT = os.environ.get('NB_OUT', '/home/lucas/hercules-sim/e1_frames/nvblox_run')
RUN_SECONDS = float(os.environ.get('NB_SECONDS', '150'))
VEH = 'ghost'
FLIGHT_Z = -1.0
W, H_PX, FOV = 640, 480, 87.0
FX = (W / 2.0) / math.tan(math.radians(FOV / 2.0))
CAM_PITCH_DOWN = math.radians(20.0)


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_axis(axis, ang):
    s = math.sin(ang / 2.0)
    return (math.cos(ang / 2.0), axis[0] * s, axis[1] * s, axis[2] * s)


# camera_link (x fwd, z up) -> optical (z fwd, x right, y down): rpy(-90,0,-90)
Q_OPTICAL = qmul(q_axis((0, 0, 1), -math.pi / 2), q_axis((1, 0, 0), -math.pi / 2))
Q_CAM_PITCH = q_axis((0, 1, 0), CAM_PITCH_DOWN)  # pitch down 20 deg in FLU


def tfmsg(stamp, parent, child, xyz, q):
    t = TransformStamped()
    t.header = Header(stamp=stamp, frame_id=parent)
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, \
        t.transform.translation.z = xyz
    t.transform.rotation.w, t.transform.rotation.x, \
        t.transform.rotation.y, t.transform.rotation.z = q
    return t


class NvbloxBridge(Node):
    def __init__(self, ctl):
        super().__init__('nvblox_sim_bridge')
        self.ctl = ctl
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
        self.depth_pub = self.create_publisher(Image, '/sim/depth/image', 10)
        self.cinfo_pub = self.create_publisher(
            CameraInfo, '/sim/depth/camera_info', 10)

        self.tf_bc = TransformBroadcaster(self)
        self.static_bc = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        self.static_bc.sendTransform([
            tfmsg(now, 'base_link', 'camera0_link', (0.10, 0.0, 0.0),
                  Q_CAM_PITCH),
            tfmsg(now, 'camera0_link', 'camera0_depth_optical_frame',
                  (0.0, 0.0, 0.0), Q_OPTICAL),
        ])

        self.create_subscription(TrajectorySetpoint,
                                 'planning/trajectory_setpoint',
                                 self._sp_cb, 10)
        self.create_subscription(String, 'exploration/state',
                                 self._state_cb, 10)
        self.create_subscription(Log, '/rosout', self._rosout_cb, 50)

        self.sp, self.sp_time = None, 0.0
        self.state, self.last_log = 'INIT', ''
        self.ned = (0.0, 0.0, 0.0, 0.0)

        self.cinfo = CameraInfo()
        self.cinfo.width, self.cinfo.height = W, H_PX
        self.cinfo.distortion_model = 'plumb_bob'
        self.cinfo.d = [0.0] * 5
        self.cinfo.k = [FX, 0.0, W / 2.0, 0.0, FX, H_PX / 2.0, 0.0, 0.0, 1.0]
        self.cinfo.p = [FX, 0.0, W / 2.0, 0.0, 0.0, FX, H_PX / 2.0, 0.0,
                        0.0, 0.0, 1.0, 0.0]

        self.create_timer(0.05, self._poll_state)   # 20 Hz odom/TF
        self.create_timer(0.5, self._apply_sp)      # 2 Hz flight command

    def _poll_state(self):
        try:
            st = self.ctl.getMultirotorState(vehicle_name=VEH)
        except Exception:
            return
        p = st.kinematics_estimated.position
        q = st.kinematics_estimated.orientation  # NED (w,x,y,z)
        yaw_ned = math.atan2(
            2.0 * (q.w_val * q.z_val + q.x_val * q.y_val),
            1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val))
        self.ned = (p.x_val, p.y_val, p.z_val, yaw_ned)
        # NED -> FLU: pos (x,-y,-z); quat (w,x,-y,-z)
        fx_, fy_, fz_ = p.x_val, -p.y_val, -p.z_val
        qf = (q.w_val, q.x_val, -q.y_val, -q.z_val)
        now = self.get_clock().now().to_msg()

        self.tf_bc.sendTransform(
            tfmsg(now, 'odom', 'base_link', (fx_, fy_, fz_), qf))

        od = Odometry()
        od.header = Header(stamp=now, frame_id='odom')
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = fx_
        od.pose.pose.position.y = fy_
        od.pose.pose.position.z = fz_
        od.pose.pose.orientation.w = qf[0]
        od.pose.pose.orientation.x = qf[1]
        od.pose.pose.orientation.y = qf[2]
        od.pose.pose.orientation.z = qf[3]
        self.odom_pub.publish(od)

        vlp = VehicleLocalPosition()
        vlp.x, vlp.y, vlp.z = p.x_val, p.y_val, p.z_val
        vlp.heading = yaw_ned
        vlp.xy_valid = vlp.z_valid = vlp.v_xy_valid = vlp.v_z_valid = True
        self.vlp_pub.publish(vlp)

        vs = VehicleStatus()
        vs.arming_state = 2
        vs.nav_state = 14
        self.vstat_pub.publish(vs)

    def publish_depth(self, depth_np):
        h, w = depth_np.shape
        # D435i-like minimum range: drop returns closer than 0.35 m so the
        # pawn's own props/body never integrate as an obstacle (0 = invalid)
        depth_np = depth_np.copy()
        depth_np[depth_np < 0.35] = 0.0
        now = self.get_clock().now().to_msg()
        img = Image()
        img.header = Header(stamp=now, frame_id='camera0_depth_optical_frame')
        img.height, img.width = h, w
        img.encoding = '32FC1'
        img.is_bigendian = 0
        img.step = w * 4
        img.data = depth_np.astype(np.float32).tobytes()
        self.depth_pub.publish(img)
        # keep intrinsics consistent with the actual returned image size
        fx = (w / 2.0) / math.tan(math.radians(FOV / 2.0))
        self.cinfo.width, self.cinfo.height = w, h
        self.cinfo.k = [fx, 0.0, w / 2.0, 0.0, fx, h / 2.0, 0.0, 0.0, 1.0]
        self.cinfo.p = [fx, 0.0, w / 2.0, 0.0, 0.0, fx, h / 2.0, 0.0,
                        0.0, 0.0, 1.0, 0.0]
        self.cinfo.header = img.header
        self.cinfo_pub.publish(self.cinfo)

    def _sp_cb(self, msg):
        self.sp, self.sp_time = msg, time.time()

    def _apply_sp(self):
        if self.sp is None or (time.time() - self.sp_time) > 2.0:
            return
        px, py, pz = (float(self.sp.position[0]), float(self.sp.position[1]),
                      float(self.sp.position[2]))
        if not (math.isfinite(px) and math.isfinite(py)):
            return
        if not math.isfinite(pz):
            pz = FLIGHT_Z
        yaw_deg = math.degrees(self.sp.yaw) if math.isfinite(self.sp.yaw) else 0.0
        try:
            self.ctl.moveToPositionAsync(
                px, py, pz, 1.5,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(False, yaw_deg), vehicle_name=VEH)
        except Exception:
            pass

    def _state_cb(self, msg):
        self.state = msg.data

    def _rosout_cb(self, msg):
        if msg.name in ('simple_exploration_planner',
                        'frontier_info_structure', 'nvblox_node'):
            self.last_log = f'[{msg.name.split("_")[0]}] {msg.msg[:140]}'


def depth_loop(bridge, cli, stop_evt):
    """~10 Hz float depth -> nvblox."""
    req = [airsim.ImageRequest('front_center', airsim.ImageType.DepthPlanar,
                               True, False)]
    while not stop_evt.is_set():
        try:
            r = cli.simGetImages(req, vehicle_name=VEH)[0]
            d = airsim.get_pfm_array(r).reshape(r.height, r.width)
            bridge.publish_depth(d)
        except Exception:
            pass
        time.sleep(0.08)


def capture_loop(bridge, cli, stop_evt):
    os.makedirs(OUT, exist_ok=True)
    reqs = [
        airsim.ImageRequest('chase', airsim.ImageType.Scene, False, True),
        airsim.ImageRequest('front_center', airsim.ImageType.Scene, False, True),
    ]
    n = 0
    while not stop_evt.is_set():
        try:
            resp = cli.simGetImages(reqs, vehicle_name=VEH)
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
        print(f'takeoff retry {attempt + 1} (z={z:.2f})', flush=True)
    ctl.moveToZAsync(FLIGHT_Z, 1.0, vehicle_name=VEH)
    t0 = time.time()
    while time.time() - t0 < 15:
        if abs(ctl.getMultirotorState(VEH).kinematics_estimated.position.z_val
               - FLIGHT_Z) < 0.3:
            break
        time.sleep(0.3)
    print('airborne', flush=True)

    depth_cli = airsim.MultirotorClient()
    depth_cli.confirmConnection()
    cap_cli = airsim.MultirotorClient()
    cap_cli.confirmConnection()

    rclpy.init()
    bridge = NvbloxBridge(ctl)
    stop_evt = threading.Event()
    threads = [
        threading.Thread(target=depth_loop, args=(bridge, depth_cli, stop_evt),
                         daemon=True),
        threading.Thread(target=capture_loop, args=(bridge, cap_cli, stop_evt),
                         daemon=True),
    ]
    for t in threads:
        t.start()

    t_end = time.time() + RUN_SECONDS
    try:
        while time.time() < t_end:
            rclpy.spin_once(bridge, timeout_sec=0.05)
    finally:
        stop_evt.set()
        for t in threads:
            t.join(timeout=5)
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
