#!/usr/bin/env python3
"""Per-drone domain relay for the HERCULES merged-map RViz view.

One process per drone. Two rclpy contexts:
  SOURCE  (ROS_DOMAIN_ID = drone domain 1..4): subscribes the drone's live
          nvblox mesh stream (/nvblox_node/mesh) and its corrected TF
          (odom -> camera0_link, published by odom_correction -- the EXACT
          pose nvblox integrates depth against, so the marker sits where the
          mesh grows, by construction).
  SINK    (viewing domain, default 42): republishes
          /dN/mesh          nvblox_msgs/Mesh, frame dN/odom, per-drone TINTED
                            vertex colors (lambert shade from msg normals)
          /dN/path          nav_msgs/Path, colored trace of flown positions
          /dN/drone         visualization_msgs/MarkerArray: body sphere +
                            name label + ground beacon column at LIVE pose
          /tf               dN/odom -> dN/base (renamed live TF, throttled)
          /tf_static        world -> dN/odom at the AirSim spawn offset
                            (NED->FLU: x_flu=x_ned, y_flu=-y_ned, yaw 0)

The relay caches every tinted mesh block; when a NEW subscriber appears on
/dN/mesh (late-started rviz), it re-publishes the full cached mesh with
clear=True so late joiners see the whole map (upstream nvblox only sends a
full serialization when ITS subscriber count rises -- that is us, once).
"""
import argparse
import math
import threading
import time

import rclpy
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from rclpy.executors import SingleThreadedExecutor

from nvblox_msgs.msg import Mesh
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration as DurationMsg

COLORS = {  # r, g, b  (bright, distinct on dark bg)
    "ghost":         (0.25, 0.55, 1.00),   # blue
    "delta":         (0.25, 1.00, 0.45),   # green
    "buckshee":      (1.00, 0.30, 0.95),   # magenta
    "thunderstrike": (1.00, 0.58, 0.12),   # orange
}
SPAWNS_NED = {  # settings-fleet-4drone.json
    "ghost": (-7.0, -7.0), "delta": (-7.0, -4.0),
    "buckshee": (-7.0, -1.0), "thunderstrike": (-7.0, 2.0),
}

MESH_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=200,
                      durability=DurabilityPolicy.VOLATILE)
TF_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST, depth=100,
                    durability=DurabilityPolicy.VOLATILE)
LATCH_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


def tint_block(block, rgb):
    """Per-vertex lambert-ish tint from the block's own normals."""
    r0, g0, b0 = rgb
    cols = []
    norms = block.normals
    n = len(block.vertices)
    for i in range(n):
        if i < len(norms):
            nz = norms[i].z
            t = nz if nz > 0.0 else 0.0
        else:
            t = 0.7
        s = 0.30 + 0.70 * t          # walls dark, floors full color
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = r0 * s, g0 * s, b0 * s, 1.0
        cols.append(c)
    block.colors = cols
    return block


class Relay:
    def __init__(self, name, dom_src, dom_sink):
        self.name = name
        self.n = dom_src
        self.rgb = COLORS[name]
        self.frame = f"d{dom_src}/odom"
        self.base_frame = f"d{dom_src}/base"
        self.lock = threading.Lock()
        self.block_cache = {}          # idx tuple -> tinted MeshBlock
        self.block_size = 0.0
        self.pose = None               # (x, y, z, qx, qy, qz, qw)
        self.path_pts = []             # [(x,y,z)]
        self.last_sub_count = 0
        self.mesh_count = 0
        self.tf_count = 0

        # ---- contexts ----
        self.ctx_src = rclpy.Context()
        rclpy.init(context=self.ctx_src, domain_id=dom_src)
        self.ctx_sink = rclpy.Context()
        rclpy.init(context=self.ctx_sink, domain_id=dom_sink)

        self.src = rclpy.create_node(f"relay_src_{name}", context=self.ctx_src)
        self.sink = rclpy.create_node(f"relay_sink_{name}", context=self.ctx_sink)

        # ---- sink pubs ----
        self.pub_mesh = self.sink.create_publisher(Mesh, f"/d{self.n}/mesh", MESH_QOS)
        self.pub_path = self.sink.create_publisher(Path, f"/d{self.n}/path", LATCH_QOS)
        self.pub_marker = self.sink.create_publisher(MarkerArray, f"/d{self.n}/drone", LATCH_QOS)
        self.pub_tf = self.sink.create_publisher(TFMessage, "/tf", TF_QOS)
        self.pub_tf_static = self.sink.create_publisher(TFMessage, "/tf_static", LATCH_QOS)
        self._publish_static_tf()

        # ---- src subs ----
        self.src.create_subscription(Mesh, "/nvblox_node/mesh", self.on_mesh, MESH_QOS)
        self.src.create_subscription(TFMessage, "/tf", self.on_tf, TF_QOS)

        # ---- src-domain odom->base_link feeder (UNBLOCKS nvblox mesh streaming).
        # nvblox_node::publishLayers() hard-returns unless map_clearing_frame_id
        # ("base_link") resolves in the global frame; the fleet stack never
        # connects odom->base_link (cuVSLAM publish_odom_to_base_tf:=false,
        # odom_correction publishes odom->camera0_link, rsnode statics make
        # base_link a root). Verified root cause of mesh_msgs=0 in mapview1:
        # nvblox log spams "Lookup transform failed for frame base_link. Layer
        # pointclouds not published" every tick. We mirror the corrected camera
        # pose as odom->base_link (also keeps the drone inside nvblox's 7 m
        # layer_streamer_exclusion_radius). No fleet consumer reads this TF.
        self.pub_tf_src = self.src.create_publisher(TFMessage, "/tf", TF_QOS)
        self.src.create_timer(0.1, self.tick_src_tf)

        # ---- sink timers ----
        self.sink.create_timer(0.2, self.tick_marker)     # 5 Hz marker + tf
        self.sink.create_timer(0.5, self.tick_path)       # 2 Hz path
        self.sink.create_timer(1.0, self.tick_late_join)  # snapshot for late subs
        self.t0 = time.time()

    # ---------------- source side ----------------
    def on_mesh(self, msg):
        with self.lock:
            if msg.clear:
                self.block_cache.clear()
            self.block_size = msg.block_size_m
            for idx, blk in zip(msg.block_indices, msg.blocks):
                key = (idx.x, idx.y, idx.z)
                if len(blk.vertices) == 0:
                    self.block_cache.pop(key, None)   # deleted block
                    tint_block(blk, self.rgb)
                else:
                    self.block_cache[key] = tint_block(blk, self.rgb)
            msg.header.frame_id = self.frame
            self.mesh_count += 1
            if self.mesh_count == 1:
                self.sink.get_logger().info(f"FIRST_MESH {self.name}")
        self.pub_mesh.publish(msg)

    def on_tf(self, msg):
        for t in msg.transforms:
            # ignore our own mirrored odom->base_link (avoid self-feedback)
            if t.header.frame_id == "odom" and t.child_frame_id != "base_link":
                p = t.transform.translation
                q = t.transform.rotation
                with self.lock:
                    self.pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
                self.tf_count += 1

    def tick_src_tf(self):
        with self.lock:
            pose = self.pose
        if pose is None:
            return
        x, y, z, qx, qy, qz, qw = pose
        t = TransformStamped()
        t.header.stamp = self.src.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.pub_tf_src.publish(TFMessage(transforms=[t]))

    # ---------------- sink side ----------------
    def _publish_static_tf(self):
        xn, yn = SPAWNS_NED[self.name]
        t = TransformStamped()
        t.header.frame_id = "world"
        t.child_frame_id = self.frame
        t.transform.translation.x = xn          # FLU: x = NED x
        t.transform.translation.y = -yn         # FLU: y = -NED y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        m = TFMessage(transforms=[t])
        self.pub_tf_static.publish(m)

    def tick_marker(self):
        with self.lock:
            pose = self.pose
        if pose is None:
            return
        x, y, z, qx, qy, qz, qw = pose
        now = self.sink.get_clock().now().to_msg()

        # live TF into the viewing domain
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.pub_tf.publish(TFMessage(transforms=[t]))

        r0, g0, b0 = self.rgb
        arr = MarkerArray()

        def mk(mid, mtype):
            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = now
            m.ns = self.name
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.lifetime = DurationMsg(sec=2)
            m.color.r, m.color.g, m.color.b, m.color.a = r0, g0, b0, 1.0
            m.pose.orientation.w = 1.0
            return m

        body = mk(0, Marker.SPHERE)
        body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, z
        body.scale.x = body.scale.y = body.scale.z = 0.55

        halo = mk(3, Marker.SPHERE)
        halo.pose.position.x, halo.pose.position.y, halo.pose.position.z = x, y, z
        halo.scale.x = halo.scale.y = halo.scale.z = 1.0
        halo.color.a = 0.25

        label = mk(1, Marker.TEXT_VIEW_FACING)
        label.text = self.name.upper()
        label.pose.position.x, label.pose.position.y = x, y
        label.pose.position.z = z + 1.1
        label.scale.z = 0.85
        label.color.r = label.color.g = label.color.b = 1.0

        beacon = mk(2, Marker.CYLINDER)
        beacon.pose.position.x, beacon.pose.position.y = x, y
        beacon.pose.position.z = z / 2.0
        beacon.scale.x = beacon.scale.y = 0.07
        beacon.scale.z = max(0.1, abs(z))
        beacon.color.a = 0.45

        arr.markers.extend([body, halo, label, beacon])
        self.pub_marker.publish(arr)

    def tick_path(self):
        with self.lock:
            pose = self.pose
        if pose is None:
            return
        x, y, z = pose[0], pose[1], pose[2]
        if (not self.path_pts or
                math.dist(self.path_pts[-1], (x, y, z)) > 0.12):
            self.path_pts.append((x, y, z))
        p = Path()
        p.header.frame_id = self.frame
        p.header.stamp = self.sink.get_clock().now().to_msg()
        for px, py, pz in self.path_pts:
            ps = PoseStamped()
            ps.header.frame_id = self.frame
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = px, py, pz
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.pub_path.publish(p)

    def tick_late_join(self):
        cnt = self.pub_mesh.get_subscription_count()
        if cnt > self.last_sub_count:
            with self.lock:
                blocks = list(self.block_cache.items())
                bs = self.block_size
            if blocks:
                from nvblox_msgs.msg import Index3D
                msg = Mesh()
                msg.header.frame_id = self.frame
                msg.header.stamp = self.sink.get_clock().now().to_msg()
                msg.block_size_m = bs
                msg.clear = True
                for key, blk in blocks:
                    idx = Index3D()
                    idx.x, idx.y, idx.z = key
                    msg.block_indices.append(idx)
                    msg.blocks.append(blk)
                self.pub_mesh.publish(msg)
                self.sink.get_logger().info(
                    f"late-join snapshot: {len(blocks)} blocks -> /d{self.n}/mesh")
        self.last_sub_count = cnt
        el = time.time() - self.t0
        if int(el) % 15 == 0:
            self.sink.get_logger().info(
                f"[{self.name}] t={el:.0f}s mesh_msgs={self.mesh_count} "
                f"blocks={len(self.block_cache)} tf={self.tf_count} "
                f"path={len(self.path_pts)}")

    def spin(self):
        ex_src = SingleThreadedExecutor(context=self.ctx_src)
        ex_src.add_node(self.src)
        ex_sink = SingleThreadedExecutor(context=self.ctx_sink)
        ex_sink.add_node(self.sink)
        ts = threading.Thread(target=ex_src.spin, daemon=True)
        ts.start()
        try:
            ex_sink.spin()
        except KeyboardInterrupt:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name", choices=list(COLORS))
    ap.add_argument("--src-domain", type=int, required=True)
    ap.add_argument("--sink-domain", type=int, default=42)
    a = ap.parse_args()
    Relay(a.name, a.src_domain, a.sink_domain).spin()


if __name__ == "__main__":
    main()
