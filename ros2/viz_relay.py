#!/usr/bin/env python3
"""
Lightweight copies of heavy sensor topics, for remote viewing (Foxglove over
Tailscale). The original topics are untouched; only new *_viz topics are added.

    /unitree_go2/lidar/point_cloud  ->  /unitree_go2/lidar/point_cloud_viz
        rate-limited (--lidar_hz, sim-time based) and every Nth point kept
        (--lidar_stride).
    /zed2i/zed_node/depth/depth_registered  ->  /zed2i/zed_node/depth/depth_registered_viz
        rate-limited (--depth_hz), downscaled by --depth_step in x and y and
        converted to 16UC1 millimetres, no-data (inf/nan) = 0
        (1280x720 32FC1 = 3.6 MB -> 320x180 16UC1 = 0.12 MB with step 4).
        16UC1 because Foxglove shows 32FC1 with a 0-1 value range by default,
        which saturates everything beyond 1 m to white.
    /zed2i/zed_node/depth/depth_registered  ->  /zed2i/zed_node/depth/depth_color_viz/compressed
        same downscaled depth, colourised with the TURBO colormap
        (near = blue, far = red, --depth_max_m, no data = black), JPEG.
    /zed2i/zed_node/left/camera_info  ->  /zed2i/zed_node/depth/camera_info_viz
        intrinsics scaled to the viz depth resolution.

Started from docker/entrypoint.sh. Run manually with the system python:
    source /opt/ros/humble/setup.bash; /usr/bin/python3 viz_relay.py
"""
import argparse

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2


class RateGate:
    """Pass at most `hz` messages per second of message-stamp time."""

    def __init__(self, hz):
        self.min_dt = 1.0 / hz if hz > 0 else 0.0
        self.last = None

    def __call__(self, stamp):
        t = stamp.sec + stamp.nanosec * 1e-9
        # stamps jump backwards when the sim restarts; treat that as a fresh start
        if self.last is not None and 0.0 <= t - self.last < self.min_dt:
            return False
        self.last = t
        return True


class VizRelay(Node):
    def __init__(self, args):
        super().__init__("viz_relay")
        qos_in = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.stride = max(1, args.lidar_stride)
        self.lidar_gate = RateGate(args.lidar_hz)
        self.pub = self.create_publisher(PointCloud2, "/unitree_go2/lidar/point_cloud_viz", 1)
        self.create_subscription(PointCloud2, "/unitree_go2/lidar/point_cloud", self.on_cloud, qos_in)
        self.depth_step = max(1, args.depth_step)
        self.depth_gate = RateGate(args.depth_hz)
        self.depth_pub = self.create_publisher(Image, "/zed2i/zed_node/depth/depth_registered_viz", 1)
        self.create_subscription(Image, "/zed2i/zed_node/depth/depth_registered", self.on_depth, qos_in)
        self.depth_max_m = args.depth_max_m
        self.color_pub = self.create_publisher(
            CompressedImage, "/zed2i/zed_node/depth/depth_color_viz/compressed", 1)
        self.info_pub = self.create_publisher(CameraInfo, "/zed2i/zed_node/depth/camera_info_viz", 1)
        self.create_subscription(CameraInfo, "/zed2i/zed_node/left/camera_info", self.on_info, qos_in)
        self.info = None

    def on_info(self, msg):
        self.info = msg

    def scaled_info(self, header, width, height):
        src = self.info
        out = CameraInfo()
        out.header = header
        out.width, out.height = width, height
        out.distortion_model = src.distortion_model
        out.d = list(src.d)
        sx, sy = width / src.width, height / src.height
        k = list(src.k)
        k[0], k[2], k[4], k[5] = k[0] * sx, k[2] * sx, k[4] * sy, k[5] * sy
        out.k = k
        out.r = list(src.r)
        p = list(src.p)
        p[0], p[2], p[3], p[5], p[6] = p[0] * sx, p[2] * sx, p[3] * sx, p[5] * sy, p[6] * sy
        out.p = p
        return out

    def on_depth(self, msg):
        want_gray = self.depth_pub.get_subscription_count() > 0
        want_color = self.color_pub.get_subscription_count() > 0
        if not (want_gray or want_color) or not self.depth_gate(msg.header.stamp):
            return
        if msg.encoding != "32FC1":
            self.depth_pub.publish(msg)
            return
        k = self.depth_step
        d = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.step // 4)
        d = d[:, :msg.width][::k, ::k]
        valid = np.isfinite(d) & (d > 0)
        if want_color:
            scaled = np.clip(np.where(valid, d, 0) / self.depth_max_m * 255.0, 0, 255).astype(np.uint8)
            color = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
            color[~valid] = 0
            ok, jpg = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                cmsg = CompressedImage()
                cmsg.header = msg.header
                cmsg.format = "jpeg"
                cmsg.data = jpg.tobytes()
                self.color_pub.publish(cmsg)
        if not want_gray:
            return
        mm = np.where(valid, np.clip(d * 1000.0, 0, 65535), 0).astype(np.uint16)
        out = Image()
        out.header = msg.header
        out.encoding = "16UC1"
        out.is_bigendian = 0
        out.height, out.width = mm.shape
        out.step = out.width * 2
        out.data = mm.tobytes()
        self.depth_pub.publish(out)
        if self.info is not None:
            self.info_pub.publish(self.scaled_info(msg.header, out.width, out.height))

    def on_cloud(self, msg):
        if self.pub.get_subscription_count() == 0 or not self.lidar_gate(msg.header.stamp):
            return

        out = PointCloud2()
        out.header = msg.header
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.is_dense = msg.is_dense
        n_points = msg.width * msg.height
        if self.stride > 1 and n_points > 0:
            pts = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            pts = pts[:n_points * msg.point_step].reshape(n_points, msg.point_step)[::self.stride]
            out.data = pts.tobytes()
            out.height, out.width = 1, pts.shape[0]
        else:
            out.data = msg.data
            out.height, out.width = msg.height, msg.width
        out.row_step = out.point_step * out.width
        self.pub.publish(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lidar_hz", type=float, default=5.0, help="max publish rate (sim time)")
    ap.add_argument("--lidar_stride", type=int, default=4, help="keep every Nth point")
    ap.add_argument("--depth_hz", type=float, default=5.0, help="max publish rate (sim time)")
    ap.add_argument("--depth_step", type=int, default=4, help="keep every Nth pixel in x and y")
    ap.add_argument("--depth_max_m", type=float, default=10.0, help="depth mapped to the far (red) end of the colormap")
    args = ap.parse_args()
    rclpy.init()
    node = VizRelay(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
