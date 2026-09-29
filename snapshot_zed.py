#!/usr/bin/env python3
"""
One-shot grab of a single ZED left image + depth frame, saved to disk as PNGs
so you can inspect them without any GUI/Qt tooling on a headless server.

Usage:
    python3 snapshot_zed.py
Outputs (in the current directory):
    zed_left.png   - raw left color frame
    zed_depth.png  - depth frame, normalized + colorized for visual inspection
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class Snapshot(Node):
    def __init__(self):
        super().__init__('zed_snapshot')
        self.bridge = CvBridge()
        self.got_left = False
        self.got_depth = False
        self.create_subscription(Image, '/zed2i/zed_node/left/image_rect_color', self.left_cb, 1)
        self.create_subscription(Image, '/zed2i/zed_node/depth/depth_registered', self.depth_cb, 1)

    def left_cb(self, msg):
        if self.got_left:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite('zed_left.png', img)
        print(f"Saved zed_left.png  shape={img.shape}")
        self.got_left = True

    def depth_cb(self, msg):
        if self.got_depth:
            return
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
        finite = depth[np.isfinite(depth)]
        if finite.size > 0:
            print(f"Depth stats: min={finite.min():.3f}m max={finite.max():.3f}m "
                  f"mean={finite.mean():.3f}m  (expect within ~0.2-20m for ZED 2i)")
        else:
            print("Depth frame has no finite values -- something upstream is producing NaNs/zeros only.")
        depth_clipped = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        depth_norm = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        cv2.imwrite('zed_depth.png', depth_color)
        print(f"Saved zed_depth.png  shape={depth.shape}")
        self.got_depth = True

def main():
    rclpy.init()
    node = Snapshot()
    try:
        while rclpy.ok() and not (node.got_left and node.got_depth):
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
