"""Unit tests for object_placement (client-side checks and the sim's mesh-to-boxes maths).

Run: python3 tests/test_object_placement.py   (or pytest)
"""
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from object_placement import (at_home, column_boxes, footprint, gap, inside, local_reasons,  # noqa: E402
                              overlaps, parse_move, random_poses, ring_poses, sample_triangles, supports)

BOUNDS = [-10.0, -10.0, 10.0, 10.0]


def test_footprint_rotation():
    assert footprint(0, 0, 0, [2, 1]) == [-2, -1, 2, 1]
    box = footprint(1, 1, 90, [2, 1])
    assert [round(v, 6) for v in box] == [0, -1, 2, 3]


def test_overlaps_and_gap():
    a, b = [0, 0, 1, 1], [1.2, 0, 2, 1]
    assert not overlaps(a, b)
    assert overlaps(a, b, clearance=0.3)
    assert abs(gap(a, b) - 0.2) < 1e-9 and gap(a, [0.5, 0.5, 2, 2]) == 0


def test_local_reasons():
    box = footprint(0, 0, 0, [0.5, 0.5])
    assert local_reasons(box, BOUNDS) == []
    assert "outside" in local_reasons(footprint(9.8, 0, 0, [0.5, 0.5]), BOUNDS)[0]
    assert "robot" in local_reasons(box, BOUNDS, robot_xy=(1.2, 0), robot_clearance=1.0)[0]
    assert local_reasons(box, BOUNDS, robot_xy=(3.0, 0), robot_clearance=1.0) == []
    placed = [("Box_1", [0.55, -1, 2, 1])]
    assert local_reasons(box, BOUNDS, placed=placed, clearance=0.1) == ["overlaps the new place of Box_1"]
    assert local_reasons(box, BOUNDS, placed=[("Box_1", [0.7, -1, 2, 1])], clearance=0.1) == []


def test_ring_poses_nearest_first():
    poses = ring_poses(1.0, 2.0, 90.0, step=0.25, max_radius=1.0)
    d = [((p[0] - 1) ** 2 + (p[1] - 2) ** 2) ** 0.5 for p in poses]
    assert all(b >= a - 0.02 for a, b in zip(d, d[1:]))          # ring by ring, nearest first
    assert abs(d[0] - 0.25) < 0.01 and max(d) < 1.01
    assert all(p[2] == 90.0 for p in poses)


def test_random_poses_inside_bounds():
    rng = random.Random(0)
    poses = random_poses([1.0, 0.4], BOUNDS, rng, 100)
    assert len(poses) == 100
    assert all(inside(footprint(*p, [1.0, 0.4]), BOUNDS) for p in poses)
    assert random_poses([20, 20], BOUNDS, rng, 5) == []


def test_supports():
    pallet = {"id": "p", "x": 0, "y": 0, "yaw": 0, "half": [0.6, 0.6], "zmin": 0.0, "zmax": 0.15}
    box = {"id": "b", "x": 0.1, "y": 0, "yaw": 0, "half": [0.3, 0.3], "zmin": 0.15, "zmax": 0.6}
    far = dict(box, id="f", x=5)
    assert supports(pallet, [pallet, box])
    assert not supports(pallet, [pallet, far])
    assert not supports(box, [pallet, box])


def test_at_home():
    o = {"x": 3.18, "y": 10.8, "yaw": 90.0, "home": [3.18, 10.8, 90.0]}
    assert at_home(o)
    assert not at_home(dict(o, x=0.0))
    assert not at_home(dict(o, yaw=0.0))
    assert not at_home(dict(o, home=None))


def test_parse_move():
    assert parse_move("Box") == ("Box", None)
    assert parse_move("Box=dock") == ("Box", "dock")
    assert parse_move("Box=home") == ("Box", "home")
    assert parse_move("Box=1,2") == ("Box", [1.0, 2.0, None])
    assert parse_move("Box=1,-2.5,90") == ("Box", [1.0, -2.5, 90.0])
    try:
        parse_move("Box=1,2,3,4")
        raise AssertionError("expected ValueError")
    except ValueError:
        pass


def _cube(lo, hi):
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    pts = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                    [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    quads = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    return pts, np.array([4] * 6), np.array(sum(quads, []))


def test_sample_triangles_covers_faces():
    pts, counts, idx = _cube((0, 0, 0), (1, 1, 1))
    s = sample_triangles(pts, counts, idx, 0.1)
    assert np.allclose(s.min(axis=0), 0) and np.allclose(s.max(axis=0), 1)
    # the middle of the top face is sampled, not only the 8 corners
    assert np.any(np.all(np.abs(s - [0.5, 0.5, 1.0]) < 0.08, axis=1))


def test_column_boxes_keep_l_shape():
    # an "L": a low 2 m long plate (forks) plus a tall 0.3 m post (mast)
    p1, c1, i1 = _cube((0, 0, 0), (2.0, 0.3, 0.1))
    p2, c2, i2 = _cube((0, 0, 0), (0.3, 0.3, 2.0))
    s = np.concatenate([sample_triangles(p1, c1, i1, 0.05), sample_triangles(p2, c2, i2, 0.05)])
    boxes = column_boxes(s, 0.15)
    tall = boxes[boxes[:, 5] > 1.0]
    low = boxes[boxes[:, 5] <= 0.11]
    assert len(tall) and tall[:, 3].max() <= 0.3 + 1e-9            # tall boxes only over the post
    assert len(low) and low[:, 3].max() >= 2.0 - 1e-9               # low boxes reach the fork tips
    assert boxes[:, :3].min() >= -1e-9 and boxes[:, 3:].max() <= 2.0 + 1e-9


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"ok {name}")
