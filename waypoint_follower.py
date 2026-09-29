#!/usr/bin/env python3
"""
A* waypoint follower for the Unitree Go2 in the isaac-go2-ros2 sim.

Takes a navigation plan written by the main repo's `scripts/run_navigation.py`
(Scene-Graph-Navigation-with-Dynamic-Sensing) and walks the Go2 through every
waypoint, one by one, until it reaches the target.

Method (DovSG, `hardcode/base/ranger_mini_3.py::motion`): for each segment,
rotate in place towards the next waypoint, then drive straight to it, each
with a PID controller. DovSG measures every segment relative to the odometry
at the segment start (open loop, drift accumulates). Here every segment
targets the *absolute* waypoint using the sim's ground-truth odometry, so
errors do not accumulate. A small heading correction runs while driving, and
the robot falls back to rotating if it drifts too far off the line.

Frames: plan waypoints are in the scene graph's gravity-aligned map frame.
The synthetic datasets take camera poses straight from `/unitree_go2/odom`,
so the pre-alignment map frame is the sim world frame, and
    p_sim = inv(T_align) @ p_map
with T_align from `data/processed/<scene>/gravity_alignment.json`.

Topics: subscribes /unitree_go2/odom (nav_msgs/Odometry, GT root pose, frame
`map`), publishes /unitree_go2/cmd_vel (geometry_msgs/Twist -> RL policy).

Run inside the isaac-go2-ros2 container with ROS sourced, e.g.

    docker exec -it isaac-go2-ros2 bash -lc \\
      "source /opt/ros/humble/setup.bash; /usr/bin/python3 \\
       /home/ubuntu/isaac-go2-ros2/waypoint_follower.py \\
       --plan /home/ubuntu/dataset/navigation_plans/<plan>.json \\
       --alignment /home/ubuntu/dataset/navigation_plans/<scene>_gravity_alignment.json"

or use go_to_waypoints.sh on the host, which does the copying and this call.
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np


# --------------------------------------------------------------------------
# Frame conversion (no ROS)
# --------------------------------------------------------------------------

def load_alignment(path):
    """4x4 T_align (sim world -> map). Identity if no file is given."""
    if not path:
        print("[follower] no alignment file given, assuming map frame == sim frame")
        return np.eye(4)
    with open(path) as f:
        T = np.asarray(json.load(f)["transform"], dtype=float)
    assert T.shape == (4, 4), f"bad transform shape {T.shape} in {path}"
    return T


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def map_to_sim(waypoints, T_align, floor_z=0.0):
    """[(x, y, theta)] in map frame -> [(x, y, theta)] in sim world frame."""
    T_inv = np.linalg.inv(T_align)
    R = T_inv[:3, :3]
    yaw_offset = math.atan2(R[1, 0], R[0, 0])
    out = []
    for x, y, theta in waypoints:
        p = T_inv @ np.array([x, y, floor_z, 1.0])
        out.append((float(p[0]), float(p[1]), wrap(float(theta) + yaw_offset)))
    return out


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def trim_to_standoff(waypoints, goal_xy, stop_distance):
    """Cut the path at the first point that is within `stop_distance` of the
    goal, so the robot stops short of the object instead of walking into it.
    Returns a new waypoint list; the last waypoint's theta faces the goal."""
    if goal_xy is None or stop_distance <= 0.0:
        return list(waypoints)
    gx, gy = goal_xy

    def face(x, y):
        return math.atan2(gy - y, gx - x)

    out = [waypoints[0]]
    if math.hypot(gx - waypoints[0][0], gy - waypoints[0][1]) <= stop_distance:
        x, y, _ = waypoints[0]
        return [(x, y, face(x, y))]
    for (ax, ay, _), b in zip(waypoints, waypoints[1:]):
        bx, by, _ = b
        if math.hypot(gx - bx, gy - by) > stop_distance:
            out.append(b)
            continue
        # first point on segment a->b at distance stop_distance from goal
        dx, dy = bx - ax, by - ay
        fx, fy = ax - gx, ay - gy
        qa, qb, qc = dx * dx + dy * dy, 2 * (fx * dx + fy * dy), fx * fx + fy * fy - stop_distance ** 2
        disc = max(qb * qb - 4 * qa * qc, 0.0)
        u = (-qb - math.sqrt(disc)) / (2 * qa) if qa > 1e-12 else 1.0
        u = min(max(u, 0.0), 1.0)
        x, y = ax + u * dx, ay + u * dy
        if math.hypot(x - ax, y - ay) < 1e-3:
            out[-1] = (ax, ay, face(ax, ay))
        else:
            out.append((x, y, face(x, y)))
        return out
    # path never gets within stop_distance: keep it, face the goal at the end
    x, y, _ = out[-1]
    out[-1] = (x, y, face(x, y))
    return out


# --------------------------------------------------------------------------
# Controller (no ROS, unit-testable)
# --------------------------------------------------------------------------

class PID:
    """Same structure as DovSG's simple_pid use, with integral clamping."""

    def __init__(self, kp, ki, kd, i_limit=1.0):
        self.kp, self.ki, self.kd, self.i_limit = kp, ki, kd, i_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_err = None

    def __call__(self, err, dt):
        dt = max(dt, 1e-3)
        self.integral = float(np.clip(self.integral + err * dt, -self.i_limit, self.i_limit))
        deriv = 0.0 if self.prev_err is None else (err - self.prev_err) / dt
        self.prev_err = err
        return self.kp * err + self.ki * self.integral + self.kd * deriv


ROTATE, DRIVE, FINAL_ROTATE, DONE, FAILED = "ROTATE", "DRIVE", "FINAL_ROTATE", "DONE", "FAILED"


class WaypointController:
    """Rotate-then-drive state machine over a list of (x, y, theta) waypoints.

    step(x, y, yaw, t) -> (vx, wz), called with every new pose. t is sim time
    in seconds (the sim often runs well below real time, so wall time is wrong).
    """

    def __init__(self, waypoints, max_lin=0.5, max_ang=0.8, min_lin=0.2, min_ang=0.4,
                 pos_tol=0.15, yaw_tol_deg=6.0, realign_deg=30.0, heading_kp=1.0,
                 face_goal=True, stall_time=20.0, stall_dist=0.1):
        assert len(waypoints) >= 1
        self.wps = [tuple(map(float, w)) for w in waypoints]
        self.max_lin, self.max_ang = max_lin, max_ang
        self.min_lin, self.min_ang = min_lin, min_ang
        self.pos_tol = pos_tol
        self.yaw_tol = math.radians(yaw_tol_deg)
        self.realign = math.radians(realign_deg)
        self.heading_kp = heading_kp
        self.face_goal = face_goal
        self.stall_time, self.stall_dist = stall_time, stall_dist
        # DovSG gains (ranger_mini_3.py)
        self.lin_pid = PID(0.8, 0.1, 0.05, i_limit=0.5)
        self.ang_pid = PID(0.85, 0.05, 0.05, i_limit=0.5)
        self.idx = 0
        self.state = ROTATE
        self.reason = ""
        self._t_prev = None
        self._reset_progress()

    # ---- helpers ----
    def _reset_progress(self):
        self._best = math.inf
        self._t_progress = None

    def _stalled(self, metric, min_gain, t):
        """True if `metric` (distance or |yaw error|) has not dropped by
        `min_gain` for stall_time sim seconds."""
        if self._t_progress is None or metric < self._best - min_gain:
            self._best = min(self._best, metric)
            self._t_progress = t
            return False
        return t - self._t_progress > self.stall_time

    def _fail(self, why):
        self.reason = why
        self._set_state(FAILED)
        return 0.0, 0.0

    def _set_state(self, s):
        if s != self.state:
            self.state = s
            self.lin_pid.reset()
            self.ang_pid.reset()
            self._reset_progress()

    def _advance(self):
        self.idx += 1
        if self.idx >= len(self.wps):
            self._set_state(FINAL_ROTATE if self.face_goal else DONE)
        else:
            self._set_state(ROTATE)

    def _turn(self, err, dt):
        w = self.ang_pid(err, dt)
        w = float(np.clip(w, -self.max_ang, self.max_ang))
        if abs(w) < self.min_ang:
            w = math.copysign(self.min_ang, err)
        return w

    @property
    def finished(self):
        return self.state in (DONE, FAILED)

    # ---- main ----
    def step(self, x, y, yaw, t):
        dt = 0.05 if self._t_prev is None else t - self._t_prev
        self._t_prev = t
        if dt <= 0.0:          # duplicate / out-of-order stamp
            dt = 1e-3

        if self.finished:
            return 0.0, 0.0

        if self.state == FINAL_ROTATE:
            err = wrap(self.wps[-1][2] - yaw)
            if abs(err) < self.yaw_tol:
                self._set_state(DONE)
                return 0.0, 0.0
            if self._stalled(abs(err), math.radians(3), t):
                return self._fail(f"stalled turning to final heading ({math.degrees(err):.0f} deg off)")
            return 0.0, self._turn(err, dt)

        gx, gy, _ = self.wps[self.idx]
        dist = math.hypot(gx - x, gy - y)

        if dist < self.pos_tol:
            self._advance()
            return self.step(x, y, yaw, t) if not self.finished else (0.0, 0.0)

        heading_err = wrap(math.atan2(gy - y, gx - x) - yaw)

        if self.state == ROTATE:
            if abs(heading_err) < self.yaw_tol:
                self._set_state(DRIVE)
            else:
                if self._stalled(abs(heading_err), math.radians(3), t):
                    return self._fail(f"stalled turning towards waypoint {self.idx}"
                                      f" ({math.degrees(heading_err):.0f} deg off)")
                return 0.0, self._turn(heading_err, dt)

        # DRIVE
        if abs(heading_err) > self.realign:
            self._set_state(ROTATE)
            return 0.0, self._turn(heading_err, dt)
        if self._stalled(dist, self.stall_dist, t):
            return self._fail(f"stalled driving to waypoint {self.idx} ({dist:.2f} m away)")
        v = float(np.clip(self.lin_pid(dist, dt), self.min_lin, self.max_lin))
        v *= max(0.0, math.cos(heading_err))
        w = float(np.clip(self.heading_kp * heading_err, -self.max_ang, self.max_ang))
        return v, w


# --------------------------------------------------------------------------
# ROS node
# --------------------------------------------------------------------------

def load_plan(path):
    with open(path) as f:
        plan = json.load(f)
    if not plan.get("success", True) or not plan.get("waypoints"):
        sys.exit(f"[follower] plan {path} has no path: {plan.get('message', '')}")
    return plan


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plan", help="navigation plan json from run_navigation.py")
    ap.add_argument("--alignment", default="", help="gravity_alignment.json of the plan's scene")
    ap.add_argument("--print_pose", action="store_true", help="print current sim pose and exit")
    ap.add_argument("--max_lin", type=float, default=0.5)
    ap.add_argument("--max_ang", type=float, default=0.8)
    ap.add_argument("--pos_tol", type=float, default=0.15)
    ap.add_argument("--yaw_tol_deg", type=float, default=6.0)
    ap.add_argument("--min_lin", type=float, default=0.2, help="policy barely walks below ~0.2 m/s")
    ap.add_argument("--min_ang", type=float, default=0.4, help="policy barely turns below ~0.4 rad/s")
    ap.add_argument("--stop_distance", type=float, default=1.0,
                    help="stop this far (m) from the plan's goal_xyz, then face it (0 = drive to last waypoint)")
    ap.add_argument("--no_face_goal", action="store_true", help="skip final turn towards the object")
    ap.add_argument("--stall_time", type=float, default=20.0, help="sim seconds without progress before abort")
    ap.add_argument("--log", default="", help="csv of executed trajectory (default: next to the plan)")
    ap.add_argument("--dry_run", action="store_true", help="print sim-frame waypoints and exit")
    args = ap.parse_args()

    if not args.print_pose and not args.plan:
        ap.error("--plan is required (or use --print_pose)")

    wps_sim = None
    if args.plan:
        plan = load_plan(args.plan)
        T = load_alignment(args.alignment)
        wps_sim = map_to_sim(plan["waypoints"], T)
        print(f"[follower] plan: {args.plan}")
        print(f"[follower] target: {plan.get('receptacle_id') or plan.get('goal_xyz')}")
        for i, (a, b) in enumerate(zip(plan["waypoints"], wps_sim)):
            print(f"  wp{i}: map ({a[0]:7.3f}, {a[1]:7.3f}, {math.degrees(a[2]):7.1f} deg)"
                  f"  -> sim ({b[0]:7.3f}, {b[1]:7.3f}, {math.degrees(b[2]):7.1f} deg)")
        goal_sim = None
        if plan.get("goal_xyz"):
            gx, gy, _ = map_to_sim([(plan["goal_xyz"][0], plan["goal_xyz"][1], 0.0)], T)[0]
            goal_sim = (gx, gy)
            wps_sim = trim_to_standoff(wps_sim, goal_sim, args.stop_distance)
            print(f"[follower] goal (sim): ({gx:.3f}, {gy:.3f}); stopping {args.stop_distance:.2f} m short")
            for i, b in enumerate(wps_sim):
                print(f"  drive wp{i}: sim ({b[0]:7.3f}, {b[1]:7.3f}, {math.degrees(b[2]):7.1f} deg)")
        if args.dry_run:
            return

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    class Follower(Node):
        def __init__(self):
            super().__init__("go2_waypoint_follower")
            # The sim bridge latches the last cmd_vel, so a lost final zero
            # leaves the robot walking. Keep it reliable and send a stop burst.
            self.pub = self.create_publisher(Twist, "/unitree_go2/cmd_vel", 10)
            self.sub = self.create_subscription(Odometry, "/unitree_go2/odom", self.on_odom, 10)
            self.ctrl = None
            self.done = False
            self.last_state = None
            self.log_writer = None
            if wps_sim is not None:
                self.ctrl = WaypointController(
                    wps_sim, max_lin=args.max_lin, max_ang=args.max_ang,
                    min_lin=args.min_lin, min_ang=args.min_ang, pos_tol=args.pos_tol,
                    yaw_tol_deg=args.yaw_tol_deg, face_goal=not args.no_face_goal,
                    stall_time=args.stall_time)
                log_path = args.log or os.path.splitext(args.plan)[0] + "_executed.csv"
                self.log_file = open(log_path, "w", newline="")
                self.log_writer = csv.writer(self.log_file)
                self.log_writer.writerow(["t", "x", "y", "yaw", "state", "wp_idx", "vx", "wz"])
                self.get_logger().info(f"logging trajectory to {log_path}")
            self.get_logger().info("waiting for /unitree_go2/odom ...")

        def send(self, vx, wz):
            msg = Twist()
            msg.linear.x = float(vx)
            msg.angular.z = float(wz)
            self.pub.publish(msg)

        def on_odom(self, msg):
            if self.done:
                return
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            if self.ctrl is None:      # --print_pose
                print(f"{p.x:.4f} {p.y:.4f} {p.z:.4f} {yaw:.4f}")
                self.done = True
                return

            vx, wz = self.ctrl.step(p.x, p.y, yaw, t)
            self.send(vx, wz)
            self.log_writer.writerow([f"{t:.3f}", f"{p.x:.4f}", f"{p.y:.4f}", f"{yaw:.4f}",
                                      self.ctrl.state, self.ctrl.idx, f"{vx:.3f}", f"{wz:.3f}"])

            key = (self.ctrl.state, self.ctrl.idx)
            if key != self.last_state:
                self.last_state = key
                n = len(self.ctrl.wps)
                where = f"waypoint {min(self.ctrl.idx, n - 1)}/{n - 1}"
                self.get_logger().info(
                    f"[{self.ctrl.state:12s}] {where}  pose ({p.x:.2f}, {p.y:.2f}, {math.degrees(yaw):.0f} deg)")

            if self.ctrl.finished:
                self.send(0.0, 0.0)
                self.done = True
                if self.ctrl.state == DONE:
                    self.get_logger().info("reached target")
                else:
                    self.get_logger().error(f"aborted: {self.ctrl.reason}")

        def stop_robot(self, seconds=1.0):
            """Publish zero Twist repeatedly so the latched command is cleared."""
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                self.send(0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.05)

        def close(self):
            self.stop_robot()
            if self.log_writer is not None:
                self.log_file.close()

    try:   # keep the context alive on Ctrl+C so the stop burst can still publish
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:
        rclpy.init()
    node = Follower()
    t0 = time.monotonic()
    while node.pub.get_subscription_count() == 0 and time.monotonic() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.pub.get_subscription_count() == 0:
        node.get_logger().warn("no subscriber on /unitree_go2/cmd_vel -- is the sim running?")
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted, stopping robot")
    finally:
        node.close()
        ok = node.ctrl is None or node.ctrl.state == DONE
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
