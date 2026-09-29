# Synthetic Dataset Construction Guide

How this repo captures a synchronized RGB + depth + IMU + ground-truth-pose dataset
from the Go2 sim, and how to reproduce/extend it. Written after building and validating
the pipeline end-to-end against the NVIDIA `full-warehouse` environment.

## Why this exists

The end goal is real-world deployment data: a timestamped sequence of camera frames
combined with other sensor readings (IMU, eventually real depth). The dataset here is
built to have the *same shape* as that — an image sequence plus an independent
higher-rate IMU stream, joined by timestamp — so the alignment/parsing code written
against sim data is the same code you'll run against real robot logs later. This is the
layout used by standard robotics datasets (EuRoC MAV, TUM-VI, KITTI): flat folders of
files, not a database or bag.

## Pipeline overview

```
Isaac Sim + Isaac Lab (Go2 quadruped, ZED2i stereo camera, IMU)
        │  publishes over ROS2 (rclpy, use_sim_time)
        ▼
record_dataset.py  (repo root — plain rclpy node, run directly, no colcon build)
        │  subscribes + writes to disk
        ▼
<output-dir>/
    rgb/, depth/, depth_vis/, frames.csv, imu.csv, odom.csv, metadata.json, summary.txt
```

Everything runs inside the `isaac-go2-ros2` Docker container. The recorder is a
*separate* ROS2 node/process from the sim itself — it only talks to the sim over DDS
topics, never touches the sim's CUDA tensors directly. That matters: an earlier version
of this repo crashed with CUDA illegal-memory-access errors from a background ROS timer
reading GPU tensors concurrently with the sim's physics thread (see
`setup_changes_log.md` item 1) — running the recorder as its own process avoids that
whole class of bug by construction.

## 1. Environment: NVIDIA standard assets

`cfg/sim.yaml` selects the sim environment via `env_name`. We use NVIDIA's standard
Isaac Sim environment assets (shipped with Isaac Sim, not a custom mesh):

```yaml
env_name: office
```

Options (see `env/sim_env.py`), pulled from `Isaac/Environments/` on the Isaac Sim
asset root (confirmed live against the actual S3 bucket this Isaac Sim 4.5 install
uses — `omniverse-content-production/Assets/Isaac/4.5/Isaac/Environments/`):

| `env_name`             | Asset                                  | Notes                                  |
|-------------------------|-----------------------------------------|-----------------------------------------|
| `warehouse`              | `Simple_Warehouse/warehouse.usd`        | empty shell, fastest to render          |
| `warehouse-shelves`      | `Simple_Warehouse/warehouse_multiple_shelves.usd` | racks, no forklifts           |
| `warehouse-forklifts`    | `Simple_Warehouse/warehouse_with_forklifts.usd` | forklifts, simpler shelving     |
| `full-warehouse`         | `Simple_Warehouse/full_warehouse.usd`   | richest warehouse-domain scene, used before switching to office |
| `office`                 | `Office/office.usd`                     | **what we use now** — carpeted interior, glass-walled meeting rooms, city-skyline windows, varied furniture/prop materials. Visually much more diverse than any warehouse variant. |
| `dataset`                | custom `.usd` at `cfg.dataset_path`     | previously used, now replaced by the above |

Other assets exist on the same asset root but aren't wired into `env_name` yet (no
`elif` branch in `isaac_go2_ros2.py`, though `sim_env.py` may already have the
`create_*_env()` function): `hospital` (`Hospital/hospital.usd`, function already
exists — just needs an `elif` added), `Digital_Twin_Warehouse/small_warehouse_digital_twin.usd`
(higher-fidelity single warehouse), `Outdoor/Rivermark/rivermark.usd` (full outdoor
city-block scene), `Terrains/*.usd` (flat/rough/slope/stairs, for locomotion testing
rather than perception), `Jetracer`, `Simple_Room`, `Grid`. Wiring any of these in
follows the exact same ~5-line pattern as the existing `create_*_env()` functions.

Changing `env_name` requires restarting the sim process (Hydra reads `cfg/sim.yaml`
once at startup) — see §4.

> **First-load cost varies wildly by scene.** `full-warehouse` boots in ~75–95s.
> `office` took **~21 minutes** on its first-ever load on this machine — not a hang,
> confirmed via the Kit log (`/isaac-sim/kit/logs/Kit/Isaac-Sim/4.5/*.log`, not
> `docker logs`, which goes quiet during this phase): Office has hundreds of unique
> small-prop materials (`MI_TrashCan`, `MI_Board`, `MI_Sofa`, `MI_PC`, ...), each
> needing its own one-time RTX shader compile, vs. a warehouse's much smaller set of
> large repeated materials. This is a one-time-per-machine cost — Isaac Sim caches
> compiled shaders to `~/App/isaac-sim/cache/*` (bind-mounted), so a later reboot into
> `office` should be fast. If a restart into a new/first-seen environment looks stuck,
> check the Kit log's most recent line before assuming it's hung — if the material/prim
> counter embedded in the log lines is still climbing, it's working, just slowly.

## 2. Camera fix: ZED FOV bug

**Symptom:** every RGB/depth frame had the bottom ~40% as a flat grey band, in every
environment (custom mesh and every warehouse variant alike).

**Root cause:** `go2/go2_sensors.py`'s `add_zed_camera()` only calls
`Camera.set_horizontal_aperture()` when `horizontal_aperture_mm` is explicitly passed —
which it never was. Without that call, the USD Camera prim keeps Isaac's raw schema
default aperture (`horizontalAperture=20.955mm`, `verticalAperture=15.2908mm`). Combined
with the ZED's real 2.1mm focal length, that default aperture produces a **~157°
near-fisheye horizontal FOV** — nowhere near a real ZED 2i's ~110°. The camera was
effectively staring almost straight down; the "grey band" was point-blank/past-the-edge
floor directly under the robot, not a rendering glitch.

Confirmed via the published `fx`: before the fix, `fx = fy = 128.27` (→ ~157° FOV);
after, `fx = fy = 448.0` (→ ~110° FOV, matches spec). A small grey sliver remains at the
very bottom edge of the frame even after the fix — that's expected, it's the camera's
near-field blind zone right under the mount (a real ZED also has a ~20cm minimum
sensing distance).

**Fix applied** (`go2/go2_sensors.py`): always call `set_horizontal_aperture()`, with a
new `DEFAULT_ZED_HORIZONTAL_APERTURE_MM = 6.0` (derived from the real ZED 2i wide-lens
spec: `2 * focal_length_mm * tan(110°/2) ≈ 6.0mm`) used whenever the caller doesn't pass
a real-calibration `horizontal_aperture_mm`. `set_horizontal_aperture()` also conforms
`verticalAperture` to the requested resolution's aspect ratio as a side effect, which is
required regardless of FOV correctness.

If you later calibrate a real ZED 2i unit, pull `fx` and sensor width from
`zed.get_camera_information()...left_cam` and pass
`horizontal_aperture_mm = focal_length_mm * resolution[0] / fx` to `add_zed_camera()` —
see the docstring in `go2_sensors.py` for the exact call chain.

## 3. Main-viewport/WebRTC fix: follow-camera ceiling clipping

**Symptom:** switching to `office` (2026-09-22), the Omniverse Streaming Client (WebRTC)
showed a permanently white/blank screen, and keyboard teleop appeared completely
unresponsive — while Foxglove kept showing the ZED camera feed fine and ROS2 topics kept
publishing normally.

**Don't be misled by this combination** — it looks like a broken network/streaming
session (video in and input out both dead) but isn't. The actual cause: `camera_follow()`
in `go2/go2_env.py` (called every sim step to point the *main* viewport — the one WebRTC
streams — at the robot) had a **hardcoded** offset of 4m behind / **5m above** the robot's
root position. That's fine under the warehouse's ~8-10m roof trusses, but an office
ceiling is ~2.4-3m — a camera 5m above the robot's ~0.4m root height (≈5.4m absolute) ends
up **above the ceiling, outside the building**, rendering nothing but exterior/void. Since
that camera is stuck outside the building regardless of what the robot does, WASD input
could be working perfectly and you'd still see zero visible change — hence "teleop not
working" was actually just "no visual feedback from a camera that isn't looking at the
robot," not an input-path failure. Meanwhile the ZED sensor camera (mounted only ~0.15m
off the robot's own body, deep inside the room) kept rendering correctly, hence Foxglove
being fine the whole time. **Always check ROS2 topics/Foxglove before suspecting the sim
itself** — if sensor data is flowing, the sim is loaded and stepping; a blank *viewport*
specifically points at camera placement, not a hang or network issue.

**Fix:** `camera_follow()` now takes an `offset` parameter (default unchanged, so nothing
else regresses) instead of a hardcoded tuple, driven from `cfg/sim.yaml`:

```yaml
camera_follow_offset: [-2.5, 0.0, 1.3]  # ~1.7m absolute camera height given robot root ~0.41m
```

Tuned so the absolute camera height stays safely under a ~2.4m ceiling regardless of the
robot's exact root height in a given scene. **Re-tune this per environment** if you switch
to something with an unusual ceiling height (very low, e.g. `Simple_Room`, or very tall
again, e.g. back to a warehouse variant) — check the robot's current root Z first:

```bash
docker exec isaac-go2-ros2 bash -lc \
  "source /opt/ros/humble/setup.bash; timeout 4 ros2 topic echo /unitree_go2/odom --once 2>/dev/null" \
  | sed -n '/position:/,/orientation:/p'
```
then keep the offset's Z component to roughly `(target_ceiling_height - root_z - 0.5)` or
lower for margin.

## 4. Running the sim

The container is normally started via `./isaac-launch.sh run`, which runs
`docker compose up` **attached** in the foreground, then `docker compose down`
immediately after it returns. That matters for restarts:

> **Gotcha:** `entrypoint.sh` has `set -e`. If you kill the sim process
> (`isaac_go2_ros2.py`) with `SIGTERM`/`SIGKILL`, the entrypoint script exits
> non-zero, the container stops — and if it was launched via
> `./isaac-launch.sh run`, that script's trailing `docker compose down` then tears
> the container down completely (not just stops it). Config/cache survive (all
> bind-mounted), but you have to bring the container back up from scratch.

**To restart after changing `cfg/sim.yaml` or any sensor/env code**, prefer launching
detached so a process restart doesn't cascade into a full teardown:

```bash
cd ~/isaac_ws/src/isaac-go2-ros2/docker
AUTO_BUILD=false AUTO_RUN=true ROS_DOMAIN_ID=100 docker compose up -d
```

To reload code after editing (bind-mounted, so files update live — only the running
Python process needs restarting):

```bash
docker exec isaac-go2-ros2 pkill -9 -f isaac_go2_ros2.py   # SIGTERM can take a while / get swallowed, just use -9
# then bring the container back up (it will have exited):
cd ~/isaac_ws/src/isaac-go2-ros2/docker && AUTO_BUILD=false AUTO_RUN=true ROS_DOMAIN_ID=100 docker compose up -d
```

Boot time for a fresh warehouse load is **~75–95 seconds** before ROS2 topics
(especially `/zed2i/zed_node/left/camera_info`, the last to come up) are live. Poll for
it rather than assuming a fixed delay:

```bash
until docker exec isaac-go2-ros2 bash -lc \
  "source /opt/ros/humble/setup.bash; timeout 4 ros2 topic list 2>/dev/null | grep -q camera_info"; \
  do sleep 15; done
```

## 5. The recorder: `record_dataset.py`

Plain `rclpy` script at the repo root (same convention as `snapshot_zed.py`,
`convert_dataset.py` — no colcon package/build needed).

**Interactive use — the simple way.** `./isaac-launch.sh enter` drops you into the
container as the `ubuntu` user with a full login shell, and `.bashrc` automatically
sources everything needed (ROS2 Humble + the workspace overlay, *then* activates the
`isaaclab` conda env, *then* Isaac Sim's own conda setup script — see
`source_conda_environment`/`source_ros_environment` in `.bashrc`). In that shell, plain
`python` already has working `rclpy`/`cv_bridge`/`message_filters`/`cv2` — just run:

```bash
./isaac-launch.sh enter
cd isaac-go2-ros2
python record_dataset.py   # no args needed for normal use, see naming below
```

**Non-interactive use (`docker exec ... bash -lc "cmd"` without a real TTY, e.g.
scripted/detached runs).** `.bashrc` is only sourced for *interactive* shells, so none of
the above happens automatically — plain `python3` on `PATH` resolves to conda **base**
(Python 3.13, no compiled `rclpy` bindings) and fails on `import cv2/rclpy`. Either
source everything explicitly, or take the shortcut of using the system interpreter
(3.10, matches ROS Humble's compiled extensions) with just ROS sourced:

```bash
docker exec -it isaac-go2-ros2 bash -lc \
  "source /opt/ros/humble/setup.bash; /usr/bin/python3 /home/ubuntu/isaac-go2-ros2/record_dataset.py"
```

> **Permission gotcha (hit 2026-09-22):** the container's `ubuntu` user is hardcoded to
> uid/gid 1000 (`docker/.env`'s `LOCAL_USER_ID`/`LOCAL_GROUP_ID`, and the UID-sync block
> in `entrypoint.sh` that would normally reconcile this against the host user is
> commented out). If your host user isn't actually uid 1000, `~/dataset` (bind-mounted
> from the host) ends up owned by a *different* uid than the container's `ubuntu` user,
> and `record_dataset.py` fails with `PermissionError` on `os.makedirs()` for the output
> dir — even though the exact same command works fine via `docker exec` without
> `--user` (defaults to root, which bypasses the check). Fixed by
> `docker exec isaac-go2-ros2 chmod 777 /home/ubuntu/dataset` (top-level dir only, so
> existing `3Rscan`/`Replica` ownership is untouched). Because `~/dataset` is
> bind-mounted, this permission change lives on the host directory itself and survives
> container restarts/recreation — it would only need redoing if the host directory is
> ever deleted and recreated from scratch with default permissions again.
>
> **Corollary:** files a `docker exec` process creates are owned by whichever user ran
> it (root by default, `ubuntu` if you pass `--user ubuntu` / use `enter`). Deleting a
> run's output from the *host* shell (plain `rm -rf ~/dataset/synthetic_dataset_03`) can
> fail with the same permission mismatch in reverse. Always delete via
> `docker exec isaac-go2-ros2 rm -rf /home/ubuntu/dataset/synthetic_dataset_03` instead.

### Output naming — auto-incrementing, no flags needed

Each run with no `--output-dir` picks the next `~/dataset/synthetic_dataset_NN` folder
automatically (`NN` = highest existing one + 1, starting at `01`) — see
`_next_synthetic_dataset_dir()` in the script. So the normal workflow is just: run it,
Ctrl+C whenever you're done driving the robot around, rerun the exact same command to
start the next capture — no need to think up a name or pass a flag each time:

```bash
python record_dataset.py    # -> ~/dataset/synthetic_dataset_01
# ... teleop the robot around, Ctrl+C when done ...
python record_dataset.py    # -> ~/dataset/synthetic_dataset_02
# ... and so on
```

Pass `--output-dir` explicitly only if you want a specific/non-numbered location.

Run it **detached** for a real capture session (survives independent of any one
terminal/ssh connection), while you separately connect via WebRTC/Foxglove and teleop
the Go2 around (`W`/`A`/`S`/`D`/`Z`/`C`) — the recorder doesn't drive the robot, it just
captures whatever the sim publishes:

```bash
docker exec -d isaac-go2-ros2 bash -lc \
  "source /opt/ros/humble/setup.bash; /usr/bin/python3 /home/ubuntu/isaac-go2-ros2/record_dataset.py > /home/ubuntu/isaac-go2-ros2/record_dataset.log 2>&1"
```

Check progress:
```bash
tail -f ~/isaac_ws/src/isaac-go2-ros2/record_dataset.log
```

Stop it (use `SIGINT`, not `-9` — the shutdown handler flushes CSVs and writes
`summary.txt`; a hard kill skips that):
```bash
docker exec isaac-go2-ros2 pkill -SIGINT -f record_dataset.py
```

(You may see a harmless `RCLError: rcl_shutdown already called` traceback right at exit
— it happens *after* the summary is written and all files are flushed/closed, so no
data is lost. Known double-shutdown race between `rclpy.spin()`'s `KeyboardInterrupt`
handling and the explicit `rclpy.shutdown()` in `finally`.)

Default stop condition is **manual** (Ctrl+C / SIGINT) — record for as long as you're
driving the robot around. `--max-frames` / `--duration-sec` exist as optional escape
hatches for scripted, repeatable runs.

### What it subscribes to

| Topic | Type | Notes |
|---|---|---|
| `/zed2i/zed_node/left/image_rect_color` | `sensor_msgs/Image` | RGB, synced with depth |
| `/zed2i/zed_node/depth/depth_registered` | `sensor_msgs/Image`, 32FC1 meters | Isaac ground-truth depth (not simulated stereo noise) |
| `/zed2i/zed_node/left/camera_info` | `sensor_msgs/CameraInfo` | read once, written to `metadata.json` |
| `/zed2i/zed_node/imu/data` | `sensor_msgs/Imu` | independent stream, much higher rate than camera |
| `/unitree_go2/odom` | `nav_msgs/Odometry` | free ground-truth pose+twist — sim-only luxury, useful for evaluating any VIO/SLAM/odometry model against perfect ground truth |

RGB and depth are joined via `message_filters.ApproximateTimeSynchronizer` (slop 0.05s)
so every saved frame pair is guaranteed time-aligned. IMU and odom are logged
independently at their own native rates (IMU/odom timestamps must *contain* the frame
timestamp range — see §7 for how to check this).

## 6. Dataset format

```
<output-dir>/
    rgb/000000.png ...          bgr8, 1280x720
    depth/000000.npy ...        float32 meters, 720x1280 (np.load to read)
    depth_vis/000000.png ...    JET colormap, for fast visual QA without loading .npy
    frames.csv                  frame_idx, stamp_sec, stamp_nanosec, rgb_path, depth_path
    imu.csv                     stamp_sec, stamp_nanosec, wx, wy, wz, ax, ay, az
    odom.csv                    stamp_sec, stamp_nanosec, x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz
    metadata.json                camera intrinsics (K, D, R, P), topic names, depth units
    summary.txt                  frame/sample counts, sim vs wall time span
```

Notes:
- All timestamps are **sim time** (`use_sim_time` is on, `/clock` is published), not
  wall-clock — sim runs well below real-time under render load (observed real-time
  factor as low as ~0.06–0.4 depending on scene complexity), so don't assume a fixed
  frame rate; always join by timestamp.
- Depth is stored as raw float32 meters, not quantized 16-bit millimeters — exact,
  simplest, matches how `snapshot_zed.py` already handles depth units.
- IMU/odom rows are independent of `frames.csv` rows — join by nearest timestamp
  (standard EuRoC/TUM-VI style split), don't assume row-index alignment.

## 7. Verifying a capture

After any run, a quick sanity pass:

```bash
D=~/dataset/<run_dir>

# frame timestamps must fall inside the imu/odom timestamp range (no orphaned frames)
head -2 "$D/frames.csv" | tail -1 && tail -1 "$D/frames.csv"
head -2 "$D/imu.csv"    | tail -1 && tail -1 "$D/imu.csv"
head -2 "$D/odom.csv"   | tail -1 && tail -1 "$D/odom.csv"

# depth sanity (finite, plausible range for an indoor warehouse scene)
python3 -c "
import numpy as np, glob
for f in sorted(glob.glob('$D/depth/*.npy'))[:3]:
    d = np.load(f); finite = d[np.isfinite(d)]
    print(f, 'min=%.2f max=%.2f mean=%.2f' % (finite.min(), finite.max(), finite.mean()))
"
```

For a visual check, just look at a few `rgb/*.png` / `depth_vis/*.png` directly rather
than trusting numbers alone — this is exactly how the FOV bug in §2 was actually caught
(the intrinsics alone looked plausible; only viewing the image revealed the problem).

## 8. Known issues / things to revisit

- **Depth storage cost dominates.** float32 `.npy` depth is ~3.7MB/frame uncompressed —
  a 10-minute session at a few Hz can be several GB. Fine at current disk headroom, but
  if `entc`'s disk (was 92% full / 76G free) gets tight, switch `depth/*.npy` to
  compressed `.npz` (`np.savez_compressed`) — same precision, smaller, one-line change
  in `record_dataset.py`'s `synced_cb`.
- **IMU is currently noiseless / near-ideal** (straight from Isaac's `IMUSensor`, no
  bias/drift/white-noise injected). Fine for now; if sim-to-real transfer matters later,
  inject a noise model matching the real Go2's IMU characteristics before training on
  this data, or the gap will show up as a distribution shift.
- **Single static scene.** `full-warehouse` is one fixed layout. For anything beyond a
  quick generalization test, add domain randomization on top (lighting, textures,
  distractor props via Omniverse Replicator randomizers) rather than relying on this
  scene alone.
- **`add_camera()`'s `front_cam`** (the other, currently-disabled camera path — gated by
  `sensor.enable_camera: False` in `cfg/sim.yaml`) has the same missing
  `set_horizontal_aperture()` call as the ZED fix in §2. Not fixed since it's unused, but
  fix it the same way before ever enabling it.
