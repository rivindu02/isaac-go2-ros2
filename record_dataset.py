#!/usr/bin/env python3
"""
Synthetic dataset recorder for the isaac-go2-ros2 sim.

Subscribes to the ZED2i RGB + depth (approx-time-synced), IMU, odom and
camera_info topics already published by the running simulation, and writes
a flat-folder dataset to disk:

    <output-dir>/
        rgb/000000.png ...            bgr8 left camera frames
        depth/000000.npy ...          float32 depth in meters (passthrough)
        depth_vis/000000.png ...      JET-colormapped depth, for quick QA
        frames.csv                    frame_idx, stamp_sec, stamp_nanosec, rgb_path, depth_path
        imu.csv                       stamp_sec, stamp_nanosec, wx, wy, wz, ax, ay, az
        odom.csv                      stamp_sec, stamp_nanosec, x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz
        metadata.json                 camera intrinsics + run provenance

Run inside the isaac-go2-ros2 container -- either interactively via
`./isaac-launch.sh enter` (plain `python record_dataset.py` works there, see
synthetic_dataset_guide.md), or non-interactively with the *system* python3.10
(not conda's), after sourcing the ROS2 setup:

    docker exec -it isaac-go2-ros2 bash -lc \\
      "source /opt/ros/humble/setup.bash; /usr/bin/python3 \\
       /home/ubuntu/isaac-go2-ros2/record_dataset.py"

No --output-dir needed for normal use: each run auto-picks the next
~/dataset/synthetic_dataset_NN folder (NN = highest existing + 1, starting at
01). Ctrl+C to stop, rerun the same command with no args to start the next
numbered capture -- see _next_synthetic_dataset_dir() below. Pass
--output-dir explicitly to override.

Stop with Ctrl+C (default). --max-frames / --duration-sec are optional
escape hatches for scripted/repeatable runs.
"""
import argparse
import csv
import json
import os
import re
import signal
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
import message_filters
from sensor_msgs.msg import Image, Imu, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge


def stamp_parts(header_stamp):
    return header_stamp.sec, header_stamp.nanosec


class BufferedCsvWriter:
    """Small buffered CSV writer so a high-rate callback (IMU) stays cheap."""

    def __init__(self, path, header, flush_every=50):
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(header)
        self._flush_every = flush_every
        self._count = 0

    def write_row(self, row):
        self._w.writerow(row)
        self._count += 1
        if self._count % self._flush_every == 0:
            self._f.flush()

    def close(self):
        self._f.flush()
        self._f.close()


class DatasetRecorder(Node):
    def __init__(self, args):
        super().__init__("dataset_recorder")
        self.args = args
        self.bridge = CvBridge()

        self.output_dir = args.output_dir
        self.rgb_dir = os.path.join(self.output_dir, "rgb")
        self.depth_dir = os.path.join(self.output_dir, "depth")
        self.depth_vis_dir = os.path.join(self.output_dir, "depth_vis")
        for d in (self.output_dir, self.rgb_dir, self.depth_dir, self.depth_vis_dir):
            os.makedirs(d, exist_ok=True)

        self.frame_idx = 0
        self.imu_count = 0
        self.odom_count = 0
        self.start_wall_time = time.time()
        self.start_sim_time = None
        self.last_sim_time = None
        self._camera_info_written = False

        self.frames_csv = BufferedCsvWriter(
            os.path.join(self.output_dir, "frames.csv"),
            ["frame_idx", "stamp_sec", "stamp_nanosec", "rgb_path", "depth_path"],
            flush_every=1,
        )
        self.imu_csv = BufferedCsvWriter(
            os.path.join(self.output_dir, "imu.csv"),
            ["stamp_sec", "stamp_nanosec", "wx", "wy", "wz", "ax", "ay", "az"],
            flush_every=args.imu_flush_every,
        )
        self.odom_csv = BufferedCsvWriter(
            os.path.join(self.output_dir, "odom.csv"),
            ["stamp_sec", "stamp_nanosec", "x", "y", "z", "qx", "qy", "qz", "qw",
             "vx", "vy", "vz", "wx", "wy", "wz"],
            flush_every=args.imu_flush_every,
        )

        # --- RGB + depth, approx-time-synced ---
        rgb_sub = message_filters.Subscriber(
            self, Image, "/zed2i/zed_node/left/image_rect_color",
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
        )
        depth_sub = message_filters.Subscriber(
            self, Image, "/zed2i/zed_node/depth/depth_registered",
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.05
        )
        self.sync.registerCallback(self.synced_cb)

        # --- IMU, independent (much higher rate than camera) ---
        self.create_subscription(
            Imu, "/zed2i/zed_node/imu/data", self.imu_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        # --- Odom ground truth, independent ---
        self.create_subscription(
            Odometry, "/unitree_go2/odom", self.odom_cb, 10,
        )

        # --- camera_info, one-shot ---
        self.create_subscription(
            CameraInfo, "/zed2i/zed_node/left/camera_info", self.camera_info_cb, 10,
        )

        # --- optional stop conditions ---
        self._stop_deadline = None
        if args.duration_sec is not None:
            self._stop_deadline = time.time() + args.duration_sec
            self.create_timer(1.0, self._check_duration)

        self.get_logger().info(f"Recording to {self.output_dir} (Ctrl+C to stop)")

    def _check_duration(self):
        if self._stop_deadline is not None and time.time() >= self._stop_deadline:
            self.get_logger().info("Duration limit reached, shutting down.")
            raise KeyboardInterrupt

    def _track_sim_time(self, sec, nanosec):
        t = sec + nanosec * 1e-9
        if self.start_sim_time is None:
            self.start_sim_time = t
        self.last_sim_time = t

    # ------------------------------------------------------------------

    def synced_cb(self, rgb_msg, depth_msg):
        sec, nanosec = stamp_parts(rgb_msg.header.stamp)
        self._track_sim_time(sec, nanosec)

        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype(np.float32)

        idx = self.frame_idx
        rgb_name = f"{idx:06d}.png"
        depth_name = f"{idx:06d}.npy"

        cv2.imwrite(os.path.join(self.rgb_dir, rgb_name), rgb)
        np.save(os.path.join(self.depth_dir, depth_name), depth)

        depth_clipped = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        depth_norm = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        cv2.imwrite(os.path.join(self.depth_vis_dir, f"{idx:06d}.png"), depth_color)

        self.frames_csv.write_row([idx, sec, nanosec,
                                    os.path.join("rgb", rgb_name),
                                    os.path.join("depth", depth_name)])
        self.frame_idx += 1

        if self.frame_idx % 25 == 0:
            self.get_logger().info(
                f"frames={self.frame_idx} imu={self.imu_count} odom={self.odom_count}"
            )

        if self.args.max_frames is not None and self.frame_idx >= self.args.max_frames:
            self.get_logger().info("max-frames reached, shutting down.")
            raise KeyboardInterrupt

    def imu_cb(self, msg):
        sec, nanosec = stamp_parts(msg.header.stamp)
        self._track_sim_time(sec, nanosec)
        av, la = msg.angular_velocity, msg.linear_acceleration
        self.imu_csv.write_row([sec, nanosec, av.x, av.y, av.z, la.x, la.y, la.z])
        self.imu_count += 1

    def odom_cb(self, msg):
        sec, nanosec = stamp_parts(msg.header.stamp)
        self._track_sim_time(sec, nanosec)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        lv = msg.twist.twist.linear
        av = msg.twist.twist.angular
        self.odom_csv.write_row([sec, nanosec, p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                                  lv.x, lv.y, lv.z, av.x, av.y, av.z])
        self.odom_count += 1

    def camera_info_cb(self, msg):
        if self._camera_info_written:
            return
        self._camera_info_written = True
        meta = {
            "generated_at": datetime.now().isoformat(),
            "topics": {
                "rgb": "/zed2i/zed_node/left/image_rect_color",
                "depth": "/zed2i/zed_node/depth/depth_registered",
                "imu": "/zed2i/zed_node/imu/data",
                "odom": "/unitree_go2/odom",
                "camera_info": "/zed2i/zed_node/left/camera_info",
            },
            "camera": {
                "width": msg.width,
                "height": msg.height,
                "distortion_model": msg.distortion_model,
                "D": list(msg.d),
                "K": list(msg.k),
                "R": list(msg.r),
                "P": list(msg.p),
                "frame_id": msg.header.frame_id,
            },
            "depth_units": "meters (float32)",
            "notes": "depth_registered is Isaac Sim's ground-truth depth on the ZED left "
                     "camera (a shortcut, not simulated stereo matching).",
        }
        with open(os.path.join(self.output_dir, "metadata.json"), "w") as f:
            json.dump(meta, f, indent=2)
        self.get_logger().info("Wrote metadata.json (camera intrinsics).")

    # ------------------------------------------------------------------

    def close(self):
        self.frames_csv.close()
        self.imu_csv.close()
        self.odom_csv.close()
        elapsed_sim = (self.last_sim_time - self.start_sim_time) if self.start_sim_time else 0.0
        elapsed_wall = time.time() - self.start_wall_time
        summary = (
            f"\n--- recording summary ---\n"
            f"output dir     : {self.output_dir}\n"
            f"frames (rgb/d) : {self.frame_idx}\n"
            f"imu samples    : {self.imu_count}\n"
            f"odom samples   : {self.odom_count}\n"
            f"sim time span  : {elapsed_sim:.2f}s\n"
            f"wall time span : {elapsed_wall:.2f}s\n"
        )
        print(summary)
        with open(os.path.join(self.output_dir, "summary.txt"), "w") as f:
            f.write(summary)


_SYNTHETIC_DATASET_RE = re.compile(r"^synthetic_dataset_(\d+)$")


def _next_synthetic_dataset_dir(base_dir):
    """Pick <base_dir>/synthetic_dataset_NN, one past the highest NN already
    present in base_dir (00 if none exist yet). Lets you just Ctrl+C and rerun
    the script with no args each time you start a new capture -- every run
    lands in its own fresh, numbered folder instead of overwriting/mixing
    into the previous one."""
    max_idx = 0
    if os.path.isdir(base_dir):
        for name in os.listdir(base_dir):
            m = _SYNTHETIC_DATASET_RE.match(name)
            if m:
                max_idx = max(max_idx, int(m.group(1)))
    return os.path.join(base_dir, f"synthetic_dataset_{max_idx + 1:02d}")


def parse_args():
    p = argparse.ArgumentParser(description="Record synthetic RGB+depth+IMU+odom dataset from isaac-go2-ros2")
    dataset_root = os.path.join(os.path.expanduser("~"), "dataset")
    default_out = _next_synthetic_dataset_dir(dataset_root)
    p.add_argument("--output-dir", type=str, default=default_out,
                    help="Defaults to the next auto-numbered "
                         "~/dataset/synthetic_dataset_NN folder (scans for the "
                         "highest existing NN and uses NN+1). Pass explicitly "
                         "to override.")
    p.add_argument("--max-frames", type=int, default=None)
    p.add_argument("--duration-sec", type=float, default=None)
    p.add_argument("--imu-flush-every", type=int, default=50)
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = DatasetRecorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
