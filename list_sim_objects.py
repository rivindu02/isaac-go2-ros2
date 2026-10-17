#!/usr/bin/env python3
"""
List the objects the sim can move (published by ros2/object_mover.py on
/sim/objects) and optionally write a spot file to fill in.

    list_sim_objects.py                       # table of movable objects
    list_sim_objects.py --label forklift      # filter by label or id
    list_sim_objects.py --save cfg/object_spots.yaml
    list_sim_objects.py --json                # raw inventory

--save merges into the YAML file: new objects get their current pose as
`home` and an empty `spots` map; objects already in the file keep their
spots. Add spots by hand as `name: [x, y, yaw_deg]` (sim world metres).

Run inside the container with ROS sourced (or use ./sim_objects.sh list).
"""
import argparse
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

DEFAULT_SPOTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cfg", "object_spots.yaml")


def wait_for(node, topic, msg_type, timeout, qos=10, accept=lambda m: True):
    """Spin until one message on `topic` passes `accept`, return it or None."""
    box = []
    sub = node.create_subscription(msg_type, topic, lambda m: box.append(m) if accept(m) else None, qos)
    end = time.time() + timeout
    while not box and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return box[0] if box else None


def get_inventory(node, timeout=10.0):
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    msg = wait_for(node, "/sim/objects", String, timeout, qos)
    return json.loads(msg.data) if msg else None


def matches(obj, label):
    return label is None or label.lower() in obj["label"] or label.lower() in obj["id"].lower()


def load_spots(path):
    import yaml
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def save_spots(path, inv):
    import yaml
    data = load_spots(path)
    added = 0
    for o in inv["objects"]:
        entry = data.setdefault(o["id"], {})
        if not entry:
            added += 1
        entry.setdefault("label", o["label"])
        entry.setdefault("home", [o["x"], o["y"], o["yaw"]])
        entry.setdefault("spots", {})
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write("# Movable sim objects. Add target spots per object as  name: [x, y, yaw_deg]\n"
                "# (sim world frame, x/y = object centre). Written by list_sim_objects.py --save,\n"
                "# re-running it keeps existing spots.\n")
        yaml.safe_dump(data, f, sort_keys=True, default_flow_style=None)
    print(f"wrote {path} ({added} new, {len(data)} total)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", help="only objects whose label or id contains this text")
    ap.add_argument("--save", nargs="?", const=DEFAULT_SPOTS, help=f"write spot file (default {DEFAULT_SPOTS})")
    ap.add_argument("--json", action="store_true", help="print the raw inventory JSON")
    ap.add_argument("--timeout", type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("list_sim_objects")
    inv = get_inventory(node, args.timeout)
    node.destroy_node()
    rclpy.shutdown()
    if inv is None:
        sys.exit("no inventory on /sim/objects: is the sim running with object_mover.enable: True?")

    if args.json:
        print(json.dumps(inv, indent=2))
    else:
        objs = [o for o in inv["objects"] if matches(o, args.label)]
        print(f"{'id':32s} {'label':10s} {'x':>8s} {'y':>8s} {'yaw':>7s} {'size x*y':>11s}")
        for o in sorted(objs, key=lambda o: o["id"]):
            size = f"{2 * o['half'][0]:.1f}*{2 * o['half'][1]:.1f}"
            print(f"{o['id']:32s} {o['label']:10s} {o['x']:8.2f} {o['y']:8.2f} {o['yaw']:7.1f} {size:>11s}")
        b = inv["bounds"]
        print(f"{len(objs)} movable objects, {len(inv['obstacles'])} static obstacles, "
              f"bounds x [{b[0]:.1f}, {b[2]:.1f}] y [{b[1]:.1f}, {b[3]:.1f}]")
    if args.save:
        save_spots(args.save, inv)


if __name__ == "__main__":
    main()
