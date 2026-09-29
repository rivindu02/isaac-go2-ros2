"""Unit tests for waypoint_follower.WaypointController against a unicycle model.

Run: python3 tests/test_waypoint_follower.py   (or pytest)
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from waypoint_follower import (DONE, FAILED, WaypointController, map_to_sim, trim_to_standoff, wrap)  # noqa: E402


def simulate(ctrl, x=0.0, y=0.0, yaw=0.0, dt=0.025, t_max=300.0, noise=0.0, seed=0,
             lin_deadband=0.0, ang_deadband=0.0):
    rng = np.random.default_rng(seed)
    t = 0.0
    visited = []
    while t < t_max and not ctrl.finished:
        vx, wz = ctrl.step(x, y, yaw, t)
        vx *= 1.0 + noise * rng.standard_normal()
        wz *= 1.0 + noise * rng.standard_normal()
        # Go2 RL policy ignores small commands and under-tracks turns
        vx = 0.0 if abs(vx) < lin_deadband else vx
        wz = 0.0 if abs(wz) < ang_deadband else 0.6 * wz
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        yaw = wrap(yaw + wz * dt)
        t += dt
        visited.append((x, y))
    return x, y, yaw, t, visited


def test_forklift_plan_reaches_every_waypoint():
    wps = [(0.0, 0.0, 1.378), (1.5, 7.7, 1.551), (1.65, 15.4, 0.021)]
    ctrl = WaypointController(wps)
    x, y, yaw, t, visited = simulate(ctrl, noise=0.2)
    assert ctrl.state == DONE, ctrl.reason
    for gx, gy, _ in wps:
        assert min(math.hypot(px - gx, py - gy) for px, py in visited) < 0.2
    assert math.hypot(x - 1.65, y - 15.4) < 0.2
    assert abs(wrap(yaw - 0.021)) < math.radians(6)


def test_start_away_from_first_waypoint_walks_to_it_first():
    ctrl = WaypointController([(1.0, 1.0, 0.0), (3.0, 1.0, 0.0)])
    _, _, _, _, visited = simulate(ctrl)
    assert ctrl.state == DONE
    assert min(math.hypot(px - 1.0, py - 1.0) for px, py in visited) < 0.2


def test_turn_around():
    ctrl = WaypointController([(0.0, 0.0, 0.0), (-2.0, 0.0, math.pi)])
    x, y, _, _, _ = simulate(ctrl, yaw=0.0)
    assert ctrl.state == DONE
    assert math.hypot(x + 2.0, y) < 0.2


def test_policy_deadband():
    wps = [(0.0, 0.0, 1.378), (1.5, 7.7, 1.551), (1.65, 15.4, 0.021)]
    ctrl = WaypointController(wps)
    x, y, _, _, _ = simulate(ctrl, noise=0.2, lin_deadband=0.15, ang_deadband=0.35)
    assert ctrl.state == DONE, ctrl.reason
    assert math.hypot(x - 1.65, y - 15.4) < 0.2


def test_rotation_stall_aborts():
    ctrl = WaypointController([(0.0, 5.0, 0.0)], stall_time=5.0)
    t = 0.0
    while not ctrl.finished and t < 60.0:
        ctrl.step(0.0, 0.0, 0.0, t)   # robot never turns
        t += 0.05
    assert ctrl.state == FAILED and "turning" in ctrl.reason


def test_stall_aborts():
    ctrl = WaypointController([(5.0, 0.0, 0.0)], stall_time=5.0)   # already facing it
    t = 0.0
    while not ctrl.finished and t < 60.0:
        ctrl.step(0.0, 0.0, 0.0, t)   # robot never moves
        t += 0.05
    assert ctrl.state == FAILED and t < 10.0


def test_trim_to_standoff_stops_short_and_faces_goal():
    wps = [(0.0, 0.0, 1.378), (1.5, 7.7, 1.551), (1.65, 15.4, 0.021)]
    goal = (1.85, 15.40)
    trimmed = trim_to_standoff(wps, goal, 1.0)
    assert len(trimmed) == 3
    x, y, th = trimmed[-1]
    assert abs(math.hypot(goal[0] - x, goal[1] - y) - 1.0) < 1e-6
    assert abs(wrap(th - math.atan2(goal[1] - y, goal[0] - x))) < 1e-9
    ctrl = WaypointController(trimmed)
    x, y, yaw, _, _ = simulate(ctrl, noise=0.2, lin_deadband=0.15, ang_deadband=0.35)
    assert ctrl.state == DONE
    assert 0.75 < math.hypot(goal[0] - x, goal[1] - y) < 1.25
    # a long stop distance cuts earlier segments too
    assert len(trim_to_standoff(wps, goal, 9.0)) == 2
    assert trim_to_standoff(wps, goal, 0.0) == wps


def test_map_to_sim_identity_and_inverse():
    wps = [(1.0, 2.0, 0.5)]
    assert np.allclose(map_to_sim(wps, np.eye(4))[0], wps[0])
    c, s = math.cos(0.3), math.sin(0.3)
    T = np.array([[c, -s, 0, 1.0], [s, c, 0, -2.0], [0, 0, 1, 0.2], [0, 0, 0, 1]])
    x, y, th = map_to_sim(wps, T)[0]
    # T maps sim -> map, so mapping the sim point back must give the map point
    assert np.allclose((T @ np.array([x, y, -0.2, 1.0]))[:3], [1.0, 2.0, 0.0])
    assert abs(wrap(th - (0.5 - 0.3))) < 1e-9


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok ", name)
