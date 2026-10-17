#!/usr/bin/env python3
"""
Re-place a few objects in the running sim (instant teleport, no animation).

    move_sim_objects.py                          # 3 random objects to places with room
    move_sim_objects.py --random 5 --seed 1      # 5 random objects, repeatable
    move_sim_objects.py --random 1 --label forklift
    move_sim_objects.py --move SM_CardBoxA_01                # any place with room
    move_sim_objects.py --move SM_CardBoxA_01=dock           # named spot from the spot file
    move_sim_objects.py --move forklift=0,-5,90              # x, y [, yaw deg]
    move_sim_objects.py --move forklift=home     # one object back to its original place
    move_sim_objects.py --reset                  # every object back to the original layout
    move_sim_objects.py --dry-run ...            # check and show the plan, move nothing
    move_sim_objects.py --probe=-3,4             # which collider is at (-3, 4) (debug)

Is there room? The sim answers that with PhysX: the object's own mesh (as
small column boxes) is put at the target pose and overlap-tested against every
collider in the scene, in 3-D. So a place counts as free if the object really
fits there, e.g. a forklift's forks may reach under a shelf. On top of that the
target must be inside the warehouse walls (--margin), --robot-clearance from
the Go2, and not on another object's new place in the same command.

A given x,y or spot is printed as OK, or REJECTED with what it would hit and
the nearest point with room at the same yaw; rejected targets are not moved
unless --force is given. Random and unnamed targets only ever go where there
is room: a spot from the object's `spots` in cfg/object_spots.yaml (written
by list_sim_objects.py --save) if one is free, else a random place.

=home and --reset restore the authored transform. In the original scene some
objects touch others (the forklift's forks are inside the pallet it carries),
so =home reports those contacts but still goes; it is refused only if another
moved object now stands there.

Every move is logged to logs/object_moves/<time>.json with before/after
poses, as ground truth for change detection.

Run inside the container with ROS sourced (or use ./sim_objects.sh move).
"""
import argparse
import json
import os
import random
import sys
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from list_sim_objects import DEFAULT_SPOTS, get_inventory, load_spots, matches, wait_for
from object_placement import (at_home, footprint, local_reasons, parse_move, random_poses, ring_poses,
                              shrink, supports)

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "object_moves")
CHUNK = 150          # candidates per room-check request (keeps each sim step short)
RANDOM_ROUNDS = 6    # random search: rounds of CHUNK candidates before giving up


class Sim:
    """Request/reply over /sim/move_objects -> /sim/move_objects/result."""

    def __init__(self, node, timeout):
        self.node, self.timeout = node, timeout
        self.pub = node.create_publisher(String, "/sim/move_objects", 10)
        self.results = {}
        node.create_subscription(String, "/sim/move_objects/result", self._on_result, 10)
        end = time.time() + 5.0
        while self.pub.get_subscription_count() == 0 and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.3)  # let the result subscription match before the first request

    def _on_result(self, msg):
        data = json.loads(msg.data)
        self.results[data.get("request_id")] = data

    def call(self, req):
        req = dict(req, request_id=req.get("request_id") or uuid.uuid4().hex[:8])
        self.pub.publish(String(data=json.dumps(req)))
        end = time.time() + self.timeout
        while req["request_id"] not in self.results and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if req["request_id"] not in self.results:
            sys.exit(f"no reply from the sim within {self.timeout} s (request {req['request_id']})")
        return self.results.pop(req["request_id"])

    def room(self, obj_id, poses, clearance, ignore, first):
        """Sim room check. first=True: poses in order, return the first with room (or None).
        first=False: return the full check entry (with every hit) of each pose."""
        out = []
        for i in range(0, len(poses), CHUNK):
            res = self.call({"check": [{"id": obj_id, "x": p[0], "y": p[1], "yaw": p[2]} for p in poses[i:i + CHUNK]],
                             "clearance": clearance, "ignore": ignore, "first": first})
            if "check" not in res:
                sys.exit("the running sim cannot check for room yet: restart it (docker restart isaac-go2-ros2)")
            if first:
                ok = [c for c in res["check"] if c["ok"]]
                if ok:
                    return [ok[0]["x"], ok[0]["y"], ok[0]["yaw"]]
            else:
                out += res["check"]
        return None if first else out


def plan_moves(inv, spots, args, rng, robot_xy, sim):
    """Return (moves, problems). moves = [{"id", "x", "y", "yaw", "source"}]."""
    objs = {o["id"]: o for o in inv["objects"]}
    bounds = shrink(inv["bounds"], args.margin)
    problems = []

    chosen = []   # (id, target) target = None | spot name | "home" | [x, y, yaw]
    for arg in args.move:
        obj_id, target = parse_move(arg)
        if obj_id not in objs:
            problems.append(f"unknown object '{obj_id}' (see list_sim_objects.py)")
            continue
        chosen.append((obj_id, target))
    if args.random:
        taken = {c[0] for c in chosen}
        pool = [o for o in inv["objects"] if o["id"] not in taken and matches(o, args.label)
                and not supports(o, inv["objects"])]
        if len(pool) < args.random:
            problems.append(f"only {len(pool)} objects match for --random {args.random}")
        chosen += [(o["id"], None) for o in rng.sample(pool, min(args.random, len(pool)))]

    ignore = [c[0] for c in chosen]   # these leave their current places
    placed = []                       # (name, footprint) of new places planned so far
    moves = []

    def local(o, pose):
        return local_reasons(footprint(*pose, o["half"]), bounds, robot_xy, args.robot_clearance,
                             placed, args.clearance)

    def nearest(o, pose):
        cands = [p for p in ring_poses(*pose) if not local(o, p)]
        return sim.room(o["id"], cands, args.clearance, ignore, first=True)

    for obj_id, target in chosen:
        o = objs[obj_id]
        obj_spots = (spots.get(obj_id) or {}).get("spots") or {}

        if target == "home" and "home" not in obj_spots:
            pose = o.get("home")
            if not pose:
                problems.append(f"{obj_id}: original pose unknown, restart the sim (or use --reset)")
                continue
            hits = sim.room(obj_id, [pose], args.clearance, ignore, first=False)[0]["hits"]
            moved_ids = {i for i in objs if not at_home(objs[i]) and i not in ignore}
            blocking = [h for h in hits if h in moved_ids]
            if blocking and not args.force:
                problems.append(f"{obj_id}: its original place is taken by moved object(s) {blocking} "
                                f"(move them first, use --reset, or add --force)")
                continue
            print(f"OK: {obj_id} back to its original place ({pose[0]:.2f}, {pose[1]:.2f}, yaw {pose[2]:.0f})"
                  + (f", touching as in the original scene: {', '.join(hits[:6])}" if hits else ""))
            source = "home"

        elif isinstance(target, (list, str)):
            if isinstance(target, list):
                pose, source = [target[0], target[1], o["yaw"] if target[2] is None else target[2]], "given"
            elif target in obj_spots:
                pose, source = (list(obj_spots[target]) + [o["yaw"]])[:3], target
            else:
                problems.append(f"{obj_id}: no spot '{target}' in the spot file (has: {sorted(obj_spots)})")
                continue
            reasons = local(o, pose)
            hits = sim.room(obj_id, [pose], args.clearance, ignore, first=False)[0]["hits"]
            reasons += [f"hits {h}" for h in hits]
            where = f"{obj_id} at ({pose[0]:.2f}, {pose[1]:.2f}, yaw {pose[2]:.0f})"
            if not reasons:
                print(f"OK: {where} has room")
            else:
                print(f"{'FORCED' if args.force else 'REJECTED'}: {where}")
                for r in reasons[:8]:
                    print(f"    - {r}")
                if len(reasons) > 8:
                    print(f"    - ... {len(reasons) - 8} more")
                if not args.force:
                    near = nearest(o, pose)
                    hint = (f"nearest point with room: --move {obj_id}={near[0]:g},{near[1]:g},{near[2]:g}"
                            if near else "no point with room within 6 m at this yaw")
                    problems.append(f"{obj_id}: {hint}  (or add --force)")
                    continue

        else:   # any place with room: a free spot from the spot file first, else random
            names = list(obj_spots)
            rng.shuffle(names)
            spot_poses = [(list(obj_spots[n]) + [o["yaw"]])[:3] for n in names]
            ok_spots = [(n, p) for n, p in zip(names, spot_poses) if not local(o, p)]
            pose, source = None, None
            if ok_spots:
                pose = sim.room(obj_id, [p for _, p in ok_spots], args.clearance, ignore, first=True)
                source = next((n for n, p in ok_spots if p == pose), None) if pose else None
            for _ in range(RANDOM_ROUNDS if pose is None else 0):
                cands = [p for p in random_poses(o["half"], bounds, rng, CHUNK) if not local(o, p)]
                pose = sim.room(obj_id, cands, args.clearance, ignore, first=True)
                if pose:
                    source = "random"
                    break
            if pose is None:
                problems.append(f"{obj_id}: no place with room found")
                continue

        placed.append((obj_id, footprint(*pose, o["half"])))
        moves.append({"id": obj_id, "x": pose[0], "y": pose[1], "yaw": pose[2], "source": source})
    return moves, problems


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--random", type=int, help="move N random objects (default 3 if nothing else is asked)")
    ap.add_argument("--move", action="append", default=[], metavar="ID[=SPOT|=X,Y[,YAW]|=home]")
    ap.add_argument("--label", help="--random picks only objects whose label or id contains this text")
    ap.add_argument("--reset", action="store_true", help="every object back to its original pose")
    ap.add_argument("--spots", default=DEFAULT_SPOTS, help="spot file (default %(default)s)")
    ap.add_argument("--seed", type=int, help="random seed for repeatable moves")
    ap.add_argument("--clearance", type=float, default=0.1, help="sideways gap kept to everything else, m")
    ap.add_argument("--robot-clearance", type=float, default=1.0, help="gap to the robot, m")
    ap.add_argument("--margin", type=float, default=0.5, help="keep this far inside the warehouse walls, m")
    ap.add_argument("--force", action="store_true", help="move a given X,Y / spot / home even if rejected")
    ap.add_argument("--probe", action="append", default=[], metavar="X,Y[,Z]",
                    help="after the move, ray-cast down at X,Y (from height Z, default 3 m) and print "
                         "which collider is there; write --probe=X,Y when X is negative")
    ap.add_argument("--no-resync", action="store_true",
                    help="skip rebuilding the physics colliders (debug; contacts may stay at the old pose)")
    ap.add_argument("--dry-run", action="store_true", help="check and print the plan, move nothing")
    ap.add_argument("--timeout", type=float, default=15.0)
    args = ap.parse_args()
    if args.random is None and not args.move and not args.reset and not args.probe:
        args.random = 3

    rclpy.init()
    node = Node("move_sim_objects")
    try:
        inv = get_inventory(node)
        if inv is None:
            sys.exit("no inventory on /sim/objects: is the sim running with object_mover.enable: True?")
        sim = Sim(node, args.timeout)
        if args.reset:
            req, moves = {"reset": True}, []
            print("reset all objects to their original poses")
        else:
            pose_msg = wait_for(node, "/unitree_go2/pose", PoseStamped, 3.0)
            robot_xy = (pose_msg.pose.position.x, pose_msg.pose.position.y) if pose_msg else None
            if robot_xy is None:
                print("warning: no /unitree_go2/pose, robot clearance not checked")
            moves, problems = plan_moves(inv, load_spots(args.spots), args, random.Random(args.seed),
                                         robot_xy, sim)
            for p in problems:
                print(f"skip: {p}")
            before = {o["id"]: o for o in inv["objects"]}
            for m in moves:
                b = before[m["id"]]
                print(f"{m['id']:32s} ({b['x']:7.2f}, {b['y']:7.2f}, {b['yaw']:6.1f}) -> "
                      f"({m['x']:7.2f}, {m['y']:7.2f}, {m['yaw']:6.1f})  [{m['source']}]")
            if not moves and not args.probe:
                sys.exit("nothing to move")
            req = {"moves": [{k: m[k] for k in ("id", "x", "y", "yaw")} for m in moves if m["source"] != "home"],
                   "home": [m["id"] for m in moves if m["source"] == "home"]}
        if args.probe:
            req["probe"] = [[float(v) for v in p.split(",")] for p in args.probe]
        if args.no_resync:
            req["resync"] = False
        if args.dry_run:
            print("dry run, nothing sent")
            return

        req["request_id"] = uuid.uuid4().hex[:8]
        result = sim.call(req)
        after = get_inventory(node, 5.0)
        print(f"moved {len(result['moved'])}, failed {len(result['failed'])}")
        for f in result["failed"]:
            print(f"  failed {f['id']}: {f['reason']}")
        for p in result.get("probe", []):
            print(f"  probe ({p['x']:.2f}, {p['y']:.2f}): {p['hit'] or 'nothing'}"
                  + (f" at z {p['z']:.2f}" if p["hit"] else ""))

        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, time.strftime("%Y%m%d_%H%M%S") + f"_{req['request_id']}.json")
        with open(path, "w") as f:
            json.dump({"time": time.time(), "request": req, "planned": moves, "result": result,
                       "before": inv["objects"], "after": after["objects"] if after else None}, f, indent=2)
        print(f"log: {path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
