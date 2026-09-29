#!/usr/bin/env python3
"""Plot planned A* waypoints vs the Go2's executed path (sim world frame).

    python plot_executed_path.py --plan <plan.json> [--alignment <gravity_alignment.json>]
                                 [--log <plan>_executed.csv] [--out <png>]

Needs matplotlib (e.g. the `scene_graph` conda env on the entc host).
"""
import argparse
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from waypoint_follower import load_alignment, load_plan, map_to_sim, trim_to_standoff  # noqa: E402


def cross_track(px, py, wps):
    best = math.inf
    for (ax, ay, _), (bx, by, _) in zip(wps, wps[1:]):
        dx, dy = bx - ax, by - ay
        u = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / max(dx * dx + dy * dy, 1e-9)))
        best = min(best, math.hypot(px - ax - u * dx, py - ay - u * dy))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", required=True)
    ap.add_argument("--alignment", default="")
    ap.add_argument("--log", default="")
    ap.add_argument("--out", default="")
    ap.add_argument("--stop_distance", type=float, default=1.0, help="same value the follower used")
    args = ap.parse_args()
    stem = os.path.splitext(args.plan)[0]
    log = args.log or stem + "_executed.csv"
    out = args.out or stem + "_executed.png"

    plan = load_plan(args.plan)
    wps = map_to_sim(plan["waypoints"], load_alignment(args.alignment))
    rows = list(csv.DictReader(open(log)))
    xs = [float(r["x"]) for r in rows]
    ys = [float(r["y"]) for r in rows]
    ts = [float(r["t"]) for r in rows]

    fig, ax = plt.subplots(figsize=(6, 8))
    ax.plot([w[0] for w in wps], [w[1] for w in wps], "o--", color="tab:blue", label="A* waypoints")
    goal = None
    if plan.get("goal_xyz"):
        g = map_to_sim([(plan["goal_xyz"][0], plan["goal_xyz"][1], 0.0)], load_alignment(args.alignment))[0]
        goal = (g[0], g[1])
        wps = trim_to_standoff(wps, goal, args.stop_distance)
        ax.plot(wps[-1][0], wps[-1][1], "*", ms=12, color="tab:green",
                label=f"stop point ({args.stop_distance:.1f} m standoff)")
    ax.plot(xs, ys, color="tab:orange", lw=1.5, label="Go2 executed (GT odom)")
    if goal is not None:
        ax.plot(goal[0], goal[1], "s", color="tab:red", label=f"goal {plan.get('receptacle_id') or ''}")
    ax.set_aspect("equal", adjustable="datalim")
    ax.margins(0.15)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=2)
    ax.set_title("Go2 A* waypoint following")
    fig.tight_layout()
    fig.savefig(out, dpi=150)

    dev = [cross_track(x, y, wps) for x, y in zip(xs, ys)]
    walked = sum(math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]) for i in range(len(xs) - 1))
    print(f"saved {out}")
    print(f"sim time {ts[-1] - ts[0]:.1f} s, walked {walked:.2f} m, "
          f"cross-track max {max(dev):.2f} m / mean {sum(dev) / len(dev):.2f} m, "
          f"final distance to goal {math.hypot(xs[-1] - goal[0], ys[-1] - goal[1]) if goal else float('nan'):.2f} m, "
          f"final position error {math.hypot(xs[-1] - wps[-1][0], ys[-1] - wps[-1][1]):.2f} m, "
          f"final yaw error {math.degrees(float(rows[-1]['yaw']) - wps[-1][2]):.1f} deg")


if __name__ == "__main__":
    main()
