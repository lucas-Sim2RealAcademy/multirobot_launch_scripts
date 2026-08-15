#!/usr/bin/env python3
"""Replay-time augmenter for the merged-map RViz re-shoot (runs ALONGSIDE
`ros2 bag play`, domain 42).  Upgrades the recorded visualization without
touching the bag:

  * /dN/quad  MarkerArray: real QUADROTOR mesh (AirSim QuadCopter export,
    quad_body.stl, 0.98 m span baked in meters, centered) at the drone's live
    pose from the bagged /tf (dN/odom -> dN/base), yaw-only orientation,
    QUAD_SCALE x real size, per-drone color, + name label + ground beacon.
    The bagged /dN/drone (old sphere markers) is simply not displayed by the
    updated fleet_map.rviz.
  * /dN/path_recent + /dN/path_hist: the bagged /dN/path split into a bright
    "last RECENT_BAG_SEC bag-seconds" tail and a dimmed history head, so
    current motion is instantly attributable.  Split is reconstructed from
    ARRIVAL time of path growth (path only appends; poses carry no stamps),
    converted with the bag play rate.

usage: augment_replay.py --rate 2.0 [--recent-bag-sec 30] [--scale 2.5]
       [--mesh-yaw-off 0.0]
"""
import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration as DurationMsg

MESH_URI = ("file:///home/lucas/UE5/hercules-sim-big/mrq_work/mapview/"
            "quad_body.stl")
DRONES = {  # n: (name, r, g, b)
    1: ("ghost",         0.25, 0.55, 1.00),
    2: ("delta",         0.25, 1.00, 0.45),
    3: ("buckshee",      1.00, 0.30, 0.95),
    4: ("thunderstrike", 1.00, 0.58, 0.12),
}
TF_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST, depth=100,
                    durability=DurabilityPolicy.VOLATILE)
LATCH_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Augment(Node):
    def __init__(self, a):
        super().__init__("mapview_augment")
        self.rate = a.rate
        self.recent_wall = a.recent_bag_sec / a.rate
        self.scale = a.scale
        self.yaw_off = math.radians(a.mesh_yaw_off)
        self.lock = threading.Lock()
        self.pose = {}          # n -> (x,y,z,yaw, stamp_msg)
        self.paths = {}         # n -> (last Path msg, [first_seen_wall per idx])
        self.pub_quad, self.pub_rec, self.pub_hist = {}, {}, {}
        for n in DRONES:
            self.pub_quad[n] = self.create_publisher(
                MarkerArray, f"/d{n}/quad", LATCH_QOS)
            self.pub_rec[n] = self.create_publisher(
                Path, f"/d{n}/path_recent", LATCH_QOS)
            self.pub_hist[n] = self.create_publisher(
                Path, f"/d{n}/path_hist", LATCH_QOS)
            self.create_subscription(
                Path, f"/d{n}/path",
                lambda m, n=n: self.on_path(n, m), LATCH_QOS)
        self.create_subscription(TFMessage, "/tf", self.on_tf, TF_QOS)
        self.create_timer(0.1, self.tick_quad)      # 10 Hz marker refresh
        self.create_timer(0.25, self.tick_paths)    # 4 Hz split refresh
        self.n_tf = 0
        self.get_logger().info(
            f"augment up: rate={self.rate} recent_wall={self.recent_wall:.1f}s "
            f"scale={self.scale} mesh={MESH_URI}")

    # ---------------- inputs ----------------
    def on_tf(self, msg):
        for t in msg.transforms:
            c = t.child_frame_id            # "dN/base"
            if len(c) > 2 and c[0] == "d" and c.endswith("/base"):
                try:
                    n = int(c[1:c.index("/")])
                except ValueError:
                    continue
                if n not in DRONES:
                    continue
                p, q = t.transform.translation, t.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                with self.lock:
                    self.pose[n] = (p.x, p.y, p.z, yaw, t.header.stamp)
                self.n_tf += 1
                if self.n_tf == 1:
                    self.get_logger().info("FIRST_TF")

    def on_path(self, n, msg):
        now = time.monotonic()
        with self.lock:
            old = self.paths.get(n)
            seen = list(old[1]) if old else []
            k = len(msg.poses)
            if len(seen) > k:               # shrunk (shouldn't happen) - reset
                seen = seen[:k]
            seen.extend([now] * (k - len(seen)))
            self.paths[n] = (msg, seen)

    # ---------------- outputs ----------------
    def tick_quad(self):
        with self.lock:
            poses = dict(self.pose)
        for n, (x, y, z, yaw, stamp) in poses.items():
            name, r, g, b = DRONES[n]
            frame = f"d{n}/odom"
            arr = MarkerArray()

            def mk(mid, mtype):
                m = Marker()
                m.header.frame_id = frame
                m.header.stamp = stamp
                m.ns = name
                m.id = mid
                m.type = mtype
                m.action = Marker.ADD
                m.lifetime = DurationMsg(sec=2)
                m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
                m.pose.orientation.w = 1.0
                return m

            quad = mk(0, Marker.MESH_RESOURCE)
            quad.mesh_resource = MESH_URI
            quad.mesh_use_embedded_materials = False
            quad.pose.position.x, quad.pose.position.y = x, y
            quad.pose.position.z = z
            h = yaw + self.yaw_off
            quad.pose.orientation.z = math.sin(h / 2.0)
            quad.pose.orientation.w = math.cos(h / 2.0)
            quad.scale.x = quad.scale.y = quad.scale.z = self.scale

            label = mk(1, Marker.TEXT_VIEW_FACING)
            label.text = name.upper()
            label.pose.position.x, label.pose.position.y = x, y
            label.pose.position.z = z + 1.45
            label.scale.z = 0.85
            label.color.r = label.color.g = label.color.b = 1.0

            beacon = mk(2, Marker.CYLINDER)
            beacon.pose.position.x, beacon.pose.position.y = x, y
            beacon.pose.position.z = z / 2.0
            beacon.scale.x = beacon.scale.y = 0.07
            beacon.scale.z = max(0.1, abs(z))
            beacon.color.a = 0.45

            arr.markers.extend([quad, label, beacon])
            self.pub_quad[n].publish(arr)

    def tick_paths(self):
        now = time.monotonic()
        with self.lock:
            snap = {n: (p, list(s)) for n, (p, s) in self.paths.items()}
        for n, (msg, seen) in snap.items():
            cut = 0
            for i, t0 in enumerate(seen):
                if now - t0 <= self.recent_wall:
                    cut = i
                    break
            else:
                cut = len(seen)
            hist, rec = Path(), Path()
            hist.header = msg.header
            rec.header = msg.header
            hist.poses = msg.poses[:cut + 1]          # +1 overlap: no gap
            rec.poses = msg.poses[max(0, cut - 1):]
            self.pub_hist[n].publish(hist)
            self.pub_rec[n].publish(rec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, required=True)
    ap.add_argument("--recent-bag-sec", type=float, default=30.0)
    ap.add_argument("--scale", type=float, default=2.5)
    ap.add_argument("--mesh-yaw-off", type=float, default=0.0)
    a = ap.parse_args()
    rclpy.init()
    n = Augment(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
