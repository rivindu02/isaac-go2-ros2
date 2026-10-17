"""
Pure helpers for the object mover (no ROS, no Isaac; see tests/test_object_placement.py).

Whether an object has room at a pose is decided by the sim (ros2/object_mover.py,
PhysX overlap test of the object's mesh). This module only has the cheap checks
the client does itself, candidate generators, and the mesh-to-boxes maths the
sim uses.

Footprints are 2-D axis-aligned boxes [xmin, ymin, xmax, ymax] in sim world
metres. An object at pose (x, y, yaw deg) with half extents (hx, hy) in its
own yaw frame covers the axis-aligned box of that rotated rectangle.
"""
import math

import numpy as np


def footprint(x, y, yaw_deg, half):
    c, s = abs(math.cos(math.radians(yaw_deg))), abs(math.sin(math.radians(yaw_deg)))
    ex, ey = c * half[0] + s * half[1], s * half[0] + c * half[1]
    return [x - ex, y - ey, x + ex, y + ey]


def overlaps(a, b, clearance=0.0):
    return not (a[2] + clearance <= b[0] or b[2] + clearance <= a[0]
                or a[3] + clearance <= b[1] or b[3] + clearance <= a[1])


def inside(box, bounds):
    return box[0] >= bounds[0] and box[1] >= bounds[1] and box[2] <= bounds[2] and box[3] <= bounds[3]


def gap(a, b):
    """Distance between two boxes (0 if they touch or overlap)."""
    dx = max(b[0] - a[2], a[0] - b[2], 0.0)
    dy = max(b[1] - a[3], a[1] - b[3], 0.0)
    return math.hypot(dx, dy)


def shrink(bounds, margin):
    return [bounds[0] + margin, bounds[1] + margin, bounds[2] - margin, bounds[3] - margin]


def local_reasons(box, bounds=None, robot_xy=None, robot_clearance=1.0, placed=(), clearance=0.1):
    """Reasons the client can see without the sim: outside the walls, too close to
    the robot, or on top of another object's new place in the same command
    (`placed` = [(name, footprint), ...]; the sim does not know those yet)."""
    reasons = []
    if bounds is not None and not inside(box, bounds):
        reasons.append(f"outside the warehouse walls (x [{bounds[0]:.1f}, {bounds[2]:.1f}] "
                       f"y [{bounds[1]:.1f}, {bounds[3]:.1f}])")
    if robot_xy is not None:
        d = gap(box, [robot_xy[0], robot_xy[1], robot_xy[0], robot_xy[1]])
        if d < robot_clearance:
            reasons.append(f"{d:.2f} m from the robot (need {robot_clearance} m)")
    for name, other in placed:
        if overlaps(box, other, clearance):
            reasons.append(f"overlaps the new place of {name}")
    return reasons


def ring_poses(x, y, yaw, step=0.25, max_radius=6.0):
    """Poses around (x, y) at the same yaw, nearest first (for 'nearest point with room')."""
    out, r = [], step
    while r <= max_radius + 1e-9:
        n = max(8, int(round(2 * math.pi * r / step)))
        out += [[round(x + r * math.cos(2 * math.pi * k / n), 2), round(y + r * math.sin(2 * math.pi * k / n), 2), yaw]
                for k in range(n)]
        r += step
    return out


def random_poses(half, bounds, rng, n):
    """n random poses whose footprint lies inside bounds, random yaw."""
    out = []
    for _ in range(20 * n):
        if len(out) == n:
            break
        yaw = rng.uniform(-180.0, 180.0)
        box = footprint(0.0, 0.0, yaw, half)
        if box[2] - box[0] >= bounds[2] - bounds[0] or box[3] - box[1] >= bounds[3] - bounds[1]:
            continue
        out.append([round(rng.uniform(bounds[0] - box[0], bounds[2] - box[2]), 3),
                    round(rng.uniform(bounds[1] - box[1], bounds[3] - box[3]), 3), round(yaw, 1)])
    return out


def supports(obj, others, tol=0.1):
    """True if another object rests on top of obj (moving obj would leave it floating)."""
    base = footprint(obj["x"], obj["y"], obj["yaw"], obj["half"])
    for o in others:
        if o["id"] == obj["id"]:
            continue
        if abs(o["zmin"] - obj["zmax"]) < tol and overlaps(base, footprint(o["x"], o["y"], o["yaw"], o["half"])):
            return True
    return False


def at_home(obj, tol=0.01):
    """True if the object is still at its original pose."""
    h = obj.get("home")
    return bool(h) and math.hypot(obj["x"] - h[0], obj["y"] - h[1]) <= tol and \
        abs((obj["yaw"] - h[2] + 180.0) % 360.0 - 180.0) <= 0.5


def parse_move(arg):
    """'ID' -> (ID, None); 'ID=spot' or 'ID=home' -> (ID, name); 'ID=x,y[,yaw]' -> (ID, [x, y, yaw])."""
    obj_id, _, target = arg.partition("=")
    if not target:
        return obj_id, None
    parts = target.split(",")
    try:
        nums = [float(p) for p in parts]
    except ValueError:
        return obj_id, target
    if len(nums) not in (2, 3):
        raise ValueError(f"bad target '{target}', want x,y or x,y,yaw")
    return obj_id, (nums + [None])[:3]


def sample_triangles(w, counts, idx, spacing):
    """Points on every polygon (fan-triangulated) of a mesh, at most `spacing` apart."""
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    tris = []
    for k in range(3, int(counts.max()) + 1):
        f = starts[counts >= k]
        tris.append(np.stack([idx[f], idx[f + k - 2], idx[f + k - 1]], axis=1))
    tris = np.concatenate(tris)
    a, b, c = w[tris[:, 0]], w[tris[:, 1]], w[tris[:, 2]]
    edge = np.max([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1),
                   np.linalg.norm(a - c, axis=1)], axis=0)
    n = np.clip(np.ceil(edge / spacing), 1, 200).astype(int)
    out = [a, b, c]
    for k in np.unique(n[n > 1]):
        sel = n == k
        i, j = np.meshgrid(np.arange(k + 1), np.arange(k + 1))
        keep = i + j <= k
        bi, bj = (i[keep] / k)[None, :, None], (j[keep] / k)[None, :, None]
        pa, pb, pc = a[sel][:, None], b[sel][:, None], c[sel][:, None]
        out.append((pa + bi * (pb - pa) + bj * (pc - pa)).reshape(-1, 3))
    return np.concatenate(out)


def column_boxes(local, voxel, tol=0.02):
    """Boxes [umin, vmin, wmin, umax, vmax, wmax] that cover the points `local` (N x 3).

    One box per occupied voxel, fitted to its points; then, in each vertical column,
    boxes of neighbouring voxels are stacked into one while their u/v outline stays
    within `tol`. So a tall thin part next to a low wide part stays two boxes.
    """
    cell = np.floor(local / voxel).astype(np.int64)
    order = np.lexsort((cell[:, 2], cell[:, 1], cell[:, 0]))
    cell, local = cell[order], local[order]
    first = np.flatnonzero(np.r_[True, np.any(cell[1:] != cell[:-1], axis=1)])
    lo = np.minimum.reduceat(local, first)
    hi = np.maximum.reduceat(local, first)
    cells = cell[first]
    boxes = []
    for i in range(len(cells)):
        b = [lo[i][0], lo[i][1], lo[i][2], hi[i][0], hi[i][1], hi[i][2]]
        if boxes and i > 0 and np.all(cells[i, :2] == cells[i - 1, :2]) and cells[i, 2] - cells[i - 1, 2] == 1:
            last = boxes[-1]
            if all(abs(b[k] - last[k]) <= tol for k in (0, 1, 3, 4)):
                last[0], last[1] = min(last[0], b[0]), min(last[1], b[1])
                last[3], last[4] = max(last[3], b[3]), max(last[4], b[4])
                last[5] = b[5]
                continue
        boxes.append(b)
    return np.array(boxes).reshape(-1, 6)
