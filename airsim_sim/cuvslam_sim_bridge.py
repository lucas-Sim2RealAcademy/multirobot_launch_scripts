#!/usr/bin/env python3
"""Full-real perception bridge: AirSim -> stereo IR pair + IMU + depth.

REAL nodes downstream (nothing synthetic in the perception chain):
  cuVSLAM (isaac_ros_visual_slam, stereo+IMU, their flight params)
    -> /visual_slam/tracking/odometry            [REAL VIO]
    -> odom_correction (+20 deg)  -> odom->camera0_link TF   [REAL]
    -> nvblox (fork, CUDA)        -> ESDF                    [REAL]
    -> FIS -> simple_exploration_planner                     [REAL]
The bridge only simulates the SENSORS (stereo/IMU/depth from AirSim),
the PX4 status stub, and executes trajectory setpoints.
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
from sensor_msgs.msg import Image, CameraInfo, Imu
from std_msgs.msg import Header, String
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import Log
from px4_msgs.msg import (TrajectorySetpoint, VehicleLocalPosition,
                          VehicleStatus)

OUT = os.environ.get('NB_OUT', '/home/lucas/hercules-sim/e1_frames/cuvslam_run')
RUN_SECONDS = float(os.environ.get('NB_SECONDS', '150'))
VEH = os.environ.get('NB_VEH', 'ghost')
FLIGHT_Z = -1.0
FOV = 87.0
BASELINE = 0.05
CAM_PITCH = math.radians(20.0)


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def qconj(q):
    return (q[0], -q[1], -q[2], -q[3])


def qrot(q, v):
    """Rotate vector v by quaternion q."""
    p = (0.0, v[0], v[1], v[2])
    w, x, y, z = qmul(qmul(q, p), qconj(q))
    return (x, y, z)


def q_axis(axis, ang):
    s = math.sin(ang / 2.0)
    return (math.cos(ang / 2.0), axis[0] * s, axis[1] * s, axis[2] * s)


Q_OPTICAL = qmul(q_axis((0, 0, 1), -math.pi / 2), q_axis((1, 0, 0), -math.pi / 2))
Q_CAM_PITCH = q_axis((0, 1, 0), CAM_PITCH)
# body-FLU vector -> optical-frame components: apply inverse of (pitch*optical)
Q_FLU_TO_OPT = qconj(qmul(Q_CAM_PITCH, Q_OPTICAL))


def tfmsg(stamp, parent, child, xyz, q):
    t = TransformStamped()
    t.header = Header(stamp=stamp, frame_id=parent)
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, \
        t.transform.translation.z = xyz
    t.transform.rotation.w, t.transform.rotation.x, \
        t.transform.rotation.y, t.transform.rotation.z = q
    return t


def make_cinfo(w, h, frame, tx=0.0):
    fx = (w / 2.0) / math.tan(math.radians(FOV / 2.0))
    ci = CameraInfo()
    ci.width, ci.height = w, h
    ci.distortion_model = 'plumb_bob'
    ci.d = [0.0] * 5
    ci.k = [fx, 0.0, w / 2.0, 0.0, fx, h / 2.0, 0.0, 0.0, 1.0]
    ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    ci.p = [fx, 0.0, w / 2.0, tx * fx * -1.0, 0.0, fx, h / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0]
    ci.header.frame_id = frame
    return ci


class CuvslamBridge(Node):
    def __init__(self, ctl):
        super().__init__('cuvslam_sim_bridge')
        self.ctl = ctl
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.vlp_pub = self.create_publisher(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', px4_qos)
        self.vstat_pub = self.create_publisher(
            VehicleStatus, '/fmu/out/vehicle_status', px4_qos)
        self.left_pub = self.create_publisher(Image, '/sim/ir_left/image', 10)
        self.right_pub = self.create_publisher(Image, '/sim/ir_right/image', 10)
        self.li_pub = self.create_publisher(
            CameraInfo, '/sim/ir_left/camera_info', 10)
        self.ri_pub = self.create_publisher(
            CameraInfo, '/sim/ir_right/camera_info', 10)
        self.imu_pub = self.create_publisher(Imu, '/sim/imu', 50)
        self.depth_pub = self.create_publisher(Image, '/sim/depth/image', 10)
        self.dinfo_pub = self.create_publisher(
            CameraInfo, '/sim/depth/camera_info', 10)

        # Static TF: optical frames hang off camera0_link, whose pose in odom
        # is owned by the REAL odom_correction node (from REAL cuVSLAM).
        self.static_bc = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        self.static_bc.sendTransform([
            tfmsg(now, 'camera0_link', 'camera0_infra1_optical_frame',
                  (0.0, 0.025, 0.0), Q_OPTICAL),
            tfmsg(now, 'camera0_link', 'camera0_infra2_optical_frame',
                  (0.0, -0.025, 0.0), Q_OPTICAL),
            tfmsg(now, 'camera0_link', 'camera0_depth_optical_frame',
                  (0.0, 0.0, 0.0), Q_OPTICAL),
            tfmsg(now, 'camera0_link', 'camera0_gyro_optical_frame',
                  (0.0, 0.0, 0.0), Q_OPTICAL),
        ])

        self.li = make_cinfo(640, 480, 'camera0_infra1_optical_frame')
        self.ri = make_cinfo(640, 480, 'camera0_infra2_optical_frame',
                             tx=BASELINE)
        self.di = make_cinfo(640, 480, 'camera0_depth_optical_frame')

        self.create_subscription(TrajectorySetpoint,
                                 'planning/trajectory_setpoint',
                                 self._sp_cb, 10)
        self.create_subscription(String, 'exploration/state',
                                 self._state_cb, 10)
        self.create_subscription(Log, '/rosout', self._rosout_cb, 50)

        self.sp, self.sp_time = None, 0.0
        self.state, self.last_log = 'INIT', ''
        self.ned = (0.0, 0.0, 0.0, 0.0)
        self.imu_count = 0
        self.stereo_count = 0

        self.create_timer(0.05, self._poll_state)
        self.create_timer(0.5, self._apply_sp)

    def _poll_state(self):
        try:
            st = self.ctl.getMultirotorState(vehicle_name=VEH)
        except Exception:
            return
        p = st.kinematics_estimated.position
        q = st.kinematics_estimated.orientation
        yaw_ned = math.atan2(
            2.0 * (q.w_val * q.z_val + q.x_val * q.y_val),
            1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val))
        self.ned = (p.x_val, p.y_val, p.z_val, yaw_ned)
        vlp = VehicleLocalPosition()
        vlp.x, vlp.y, vlp.z = p.x_val, p.y_val, p.z_val
        vlp.heading = yaw_ned
        vlp.xy_valid = vlp.z_valid = vlp.v_xy_valid = vlp.v_z_valid = True
        self.vlp_pub.publish(vlp)
        vs = VehicleStatus()
        vs.arming_state = 2
        vs.nav_state = 14
        self.vstat_pub.publish(vs)

    def pub_stereo(self, left8, right8):
        now = self.get_clock().now().to_msg()
        for arr, pub, ipub, ci in ((left8, self.left_pub, self.li_pub, self.li),
                                   (right8, self.right_pub, self.ri_pub, self.ri)):
            h, w = arr.shape
            img = Image()
            img.header = Header(stamp=now, frame_id=ci.header.frame_id)
            img.height, img.width = h, w
            img.encoding = 'mono8'
            img.step = w
            img.data = arr.tobytes()
            pub.publish(img)
            ci.header.stamp = now
            ipub.publish(ci)
        self.stereo_count += 1

    def pub_imu(self, imu_data):
        now = self.get_clock().now().to_msg()
        av, la = imu_data.angular_velocity, imu_data.linear_acceleration
        # AirSim body NED -> body FLU -> optical gyro frame
        g_flu = (av.x_val, -av.y_val, -av.z_val)
        a_flu = (la.x_val, -la.y_val, -la.z_val)
        g_opt = qrot(Q_FLU_TO_OPT, g_flu)
        a_opt = qrot(Q_FLU_TO_OPT, a_flu)
        m = Imu()
        m.header = Header(stamp=now, frame_id='camera0_gyro_optical_frame')
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = g_opt
        m.linear_acceleration.x, m.linear_acceleration.y, \
            m.linear_acceleration.z = a_opt
        m.orientation_covariance[0] = -1.0
        self.imu_pub.publish(m)
        self.imu_count += 1

    def pub_depth(self, depth_np):
        h, w = depth_np.shape
        depth_np = depth_np.copy()
        depth_np[depth_np < 0.35] = 0.0
        now = self.get_clock().now().to_msg()
        img = Image()
        img.header = Header(stamp=now, frame_id='camera0_depth_optical_frame')
        img.height, img.width = h, w
        img.encoding = '32FC1'
        img.step = w * 4
        img.data = depth_np.astype(np.float32).tobytes()
        self.depth_pub.publish(img)
        fx = (w / 2.0) / math.tan(math.radians(FOV / 2.0))
        self.di.width, self.di.height = w, h
        self.di.k = [fx, 0.0, w / 2.0, 0.0, fx, h / 2.0, 0.0, 0.0, 1.0]
        self.di.p = [fx, 0.0, w / 2.0, 0.0, 0.0, fx, h / 2.0, 0.0,
                     0.0, 0.0, 1.0, 0.0]
        self.di.header.stamp = now
        self.dinfo_pub.publish(self.di)

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
        if msg.name in ('simple_exploration_planner', 'frontier_info_structure',
                        'nvblox_node', 'visual_slam_node', 'odom_correction'):
            self.last_log = f'[{msg.name.split("_")[0]}] {msg.msg[:140]}'


def stereo_loop(bridge, cli, stop_evt):
    import io
    from PIL import Image as PILImage
    reqs = [airsim.ImageRequest('front_left', airsim.ImageType.Scene, False, True),
            airsim.ImageRequest('front_right', airsim.ImageType.Scene, False, True)]
    while not stop_evt.is_set():
        try:
            r = cli.simGetImages(reqs, vehicle_name=VEH)
            imgs = []
            for resp in r:
                pil = PILImage.open(io.BytesIO(resp.image_data_uint8)).convert('L')
                imgs.append(np.asarray(pil))
            bridge.pub_stereo(imgs[0], imgs[1])
        except Exception:
            pass
        time.sleep(0.005)


def imu_loop(bridge, cli, stop_evt):
    while not stop_evt.is_set():
        try:
            bridge.pub_imu(cli.getImuData('imu', VEH))
        except Exception:
            pass
        time.sleep(0.004)


def depth_loop(bridge, cli, stop_evt):
    req = [airsim.ImageRequest('front_center', airsim.ImageType.DepthPlanar,
                               True, False)]
    while not stop_evt.is_set():
        try:
            r = cli.simGetImages(req, vehicle_name=VEH)[0]
            d = airsim.get_pfm_array(r).reshape(r.height, r.width)
            bridge.pub_depth(d)
        except Exception:
            pass
        time.sleep(0.12)


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
                    'ned': [round(v, 2) for v in bridge.ned],
                    'imu_hz': bridge.imu_count, 'stereo_n': bridge.stereo_count}
            with open(f'{OUT}/meta_{n:05d}.json', 'w') as f:
                json.dump(meta, f)
            n += 1
        except Exception:
            pass
        time.sleep(0.22)
    print(f'captured {n} frames', flush=True)


def main():
    time.sleep(float(os.environ.get('NB_STAGGER', '0')))
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

    clis = [airsim.MultirotorClient() for _ in range(4)]
    for c in clis:
        c.confirmConnection()

    rclpy.init()
    bridge = CuvslamBridge(ctl)
    stop_evt = threading.Event()
    threads = [
        threading.Thread(target=stereo_loop, args=(bridge, clis[0], stop_evt), daemon=True),
        threading.Thread(target=imu_loop, args=(bridge, clis[1], stop_evt), daemon=True),
        threading.Thread(target=depth_loop, args=(bridge, clis[2], stop_evt), daemon=True),
    ]
    if os.environ.get('NB_CAPTURE', '1') == '1':
        threads.append(threading.Thread(
            target=capture_loop, args=(bridge, clis[3], stop_evt), daemon=True))
    for t in threads:
        t.start()

    t_end = time.time() + RUN_SECONDS
    try:
        while time.time() < t_end:
            rclpy.spin_once(bridge, timeout_sec=0.02)
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
    print(f'bridge done (stereo pairs={bridge.stereo_count}, imu={bridge.imu_count})',
          flush=True)


if __name__ == '__main__':
    main()
