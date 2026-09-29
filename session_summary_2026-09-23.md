# Session Summary — Synthetic Dataset Pipeline & Sim Debugging (2026-09-19 → 2026-09-23)

Narrative log of a multi-day session building a synthetic RGB+depth+IMU dataset
recorder for the `isaac-go2-ros2` Isaac Sim project on the remote server `entc`
(`ravindu@192.248.10.117`, Tailscale `100.110.199.98`), and debugging several issues
along the way. Companion to `synthetic_dataset_guide.md` (the technical reference for
the recorder itself) — this file is the story of how we got there and what's still
open.

## Context

Project: `~/isaac_ws/src/isaac-go2-ros2` on `entc` — a fork of
[Zhefan-Xu/isaac-go2-ros2](https://github.com/Zhefan-Xu/isaac-go2-ros2), running a
Unitree Go2 quadruped in Isaac Sim 4.5 / Isaac Lab 2.1, ROS2 Humble, inside Docker
(`seanchangx/isaac-go2-ros2:humble`). The robot carries a simulated ZED2i stereo
camera + IMU, publishes over ROS2, and can be teleoperated by keyboard or the sim
viewed live over WebRTC (Tailscale). Goal: generate a synthetic dataset (RGB, depth,
IMU, ground-truth pose) shaped like real-world robot sensor logs, to validate a
perception/SLAM pipeline later against real hardware.

## 1. Built the dataset recorder (`record_dataset.py`)

New file at the repo root (same "plain rclpy script" convention as the existing
`snapshot_zed.py`). Subscribes to:
- `/zed2i/zed_node/left/image_rect_color` + `/zed2i/zed_node/depth/depth_registered`
  (approx-time-synced via `message_filters`)
- `/zed2i/zed_node/imu/data` (independent, higher-rate stream)
- `/unitree_go2/odom` (free ground-truth pose+twist — a sim-only luxury, valuable for
  evaluating any downstream VIO/SLAM model)
- `/zed2i/zed_node/left/camera_info` (once, for `metadata.json`)

Output format chosen deliberately to mirror real-world robot logs (EuRoC/TUM-VI
style): flat folders of files, camera frames + independent IMU stream, joined by
timestamp — not a single synchronized blob — so the same join/alignment code works
against real sensor data later.

```
<output-dir>/
    rgb/000000.png ...        depth/000000.npy ...      depth_vis/000000.png ...
    frames.csv   imu.csv   odom.csv   metadata.json   summary.txt
```

Later refinement: output directory now **auto-increments** —
`~/dataset/synthetic_dataset_NN` (scans for the highest existing `NN`, uses `NN+1`,
starting at `01`) — so the workflow is just "run, Ctrl+C, rerun" with no flags needed
each time.

**Runtime gotcha discovered:** the container's default `python3` resolves to conda
base (no compiled `rclpy` bindings). Interactively via `./isaac-launch.sh enter`,
`.bashrc` auto-activates the right environment (ROS2 + isaaclab conda + Isaac Sim
conda setup) so plain `python` works. Non-interactively (`docker exec ... bash -lc`),
none of that runs automatically — must use `/usr/bin/python3` (3.10, matches ROS
Humble) with ROS sourced manually.

## 2. Fixed the ZED camera FOV bug

**Symptom:** every RGB/depth frame had the bottom ~40% as a flat grey band.

**Root cause:** `go2/go2_sensors.py`'s `add_zed_camera()` only called
`Camera.set_horizontal_aperture()` when a real calibration value was passed — never
the case in practice. Isaac's raw default aperture (`20.955mm`) combined with the
ZED's real `2.1mm` focal length produced a **~157° near-fisheye FOV** instead of the
real ZED 2i's ~110°. The camera was staring almost straight down at its own near-field
floor.

**Fix:** always call `set_horizontal_aperture()`, defaulting to a new
`DEFAULT_ZED_HORIZONTAL_APERTURE_MM = 6.0` (derived from the real spec:
`2 * focal_length_mm * tan(110°/2)`) when no real calibration is supplied. Confirmed
via published `fx`: `128.27` (157° FOV) → `448.0` (110° FOV, correct) — and visually,
the grey band shrank to a thin, expected near-field sliver at the very bottom edge.

## 3. Switched environments: `dataset` (custom mesh) → `full-warehouse` → `office`

Surveyed NVIDIA's actual Isaac Sim 4.5 asset bucket
(`omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Environments/`)
for what's available beyond the warehouse set: `Digital_Twin_Warehouse`, `Hospital`,
`Office`, `Outdoor/Rivermark`, `Terrains`, `Jetracer`, `Simple_Room`, `Grid`. Found
`sim_env.py` already had `create_hospital_env()`/`create_office_env()` written but
unreachable (no `elif` branch in `isaac_go2_ros2.py`'s `env_name` dispatch).

Switched to `office` (richer visual diversity for the dataset: carpeted interior,
glass-walled rooms, city-skyline windows, varied furniture materials) — added the
missing `elif (cfg.env_name == "office")` branch, set `cfg/sim.yaml`.

**Gotcha:** first-time load into Office took **~21 minutes** (hundreds of unique small
prop materials, each needing a one-time RTX shader compile — vs. warehouse's much
smaller repeated material set). Not a hang; confirmed by watching the Kit log's
material counter climb (`docker logs` goes quiet during this phase, the actual Kit log
file at `/isaac-sim/kit/logs/...` keeps updating). One-time cost — shaders cache to
`~/App/isaac-sim/cache/*` (bind-mounted), so later reboots into Office are fast
(~60-95s).

## 4. Fixed a dataset-directory permission bug

**Symptom:** running `python record_dataset.py` interactively (as the container's
`ubuntu` user via `./isaac-launch.sh enter`) failed with `PermissionError` creating
the output dir.

**Root cause:** the container's `ubuntu` user is hardcoded to uid/gid 1000
(`docker/.env`, and the UID-sync logic in `entrypoint.sh` that would normally
reconcile this against the host user is commented out). The actual host user
(`ravindu`) is uid **1012** — so `~/dataset` (bind-mounted) was never writable by
`ubuntu`. It only worked when I ran things via `docker exec` (defaults to root,
bypasses the check).

**Fix:** `chmod 777` on the top-level `~/dataset` dir only (existing `3Rscan`/`Replica`
subdirectory ownership left untouched). Also noted the corollary: cleaning up dataset
output must go through `docker exec ... rm -rf`, not a plain host `rm`, since files
created inside the container are owned by whichever user ran the process.

## 5. Fixed a WebRTC white-screen / "teleop not working" bug

**Symptom:** after switching to Office, the Omniverse Streaming Client showed a
permanently white/blank screen, and keyboard teleop appeared unresponsive — while
Foxglove kept showing the ZED feed fine and ROS2 topics kept publishing normally.

Investigated network transport first (this looked identical to a broken WebRTC
session): confirmed `tailscale status` showed a **direct** (non-relayed) connection
with **3.16GB already transferred** — ruling out a network/transport explanation
entirely.

**Real root cause:** `camera_follow()` in `go2/go2_env.py` — which points the *main
viewport* (the one WebRTC streams) at the robot every sim step — had a **hardcoded**
offset of 4m behind / **5m above** the robot's root position. Fine under the
warehouse's ~8-10m roof trusses; way above an office's ~2.4-3m ceiling. The
main-viewport camera ended up **outside the building**, rendering nothing but white
void — regardless of what the robot did, hence "teleop not working" (no visible
feedback, not necessarily broken input). The ZED sensor camera, mounted low on the
robot's own body, kept rendering correctly the whole time — hence Foxglove being fine.

**Fix:** `camera_follow()` now takes a configurable `offset` (default unchanged, so
nothing else regresses), driven from `cfg/sim.yaml`:
```yaml
camera_follow_offset: [-2.5, 0.0, 1.3]  # ~1.7m absolute camera height given robot root ~0.41m
```
Tuned to stay safely under a ~2.4m ceiling. Documented how to re-tune per environment
if switching to something with an unusual ceiling height later.

**Lesson recorded in the guide:** always check ROS2 topics/Foxglove before suspecting
the sim itself is stuck — if sensor data is flowing, the sim is loaded and stepping; a
blank *viewport* specifically points at camera placement, not a hang.

## 6. Open issue: robot doesn't move via `cmd_vel` in Office (unresolved)

**Symptom:** `run_teleop.sh` produces no robot movement. User recalled having fixed
"a similar issue" before (a **different** bug, it turned out — see below) and worried
it was lost when a Docker image got replaced.

**Ruled out:**
- **Not the old "teleop needs a server-side TTY, not remote PC" issue.** That's a
  `teleop_twist_keyboard` stdin/TTY capture problem; bypassed entirely by publishing
  raw `Twist` messages directly via `ros2 topic pub` from the SSH session — still no
  movement, proving the current bug is unrelated to input method.
- **Not a lost Docker-image commit.** Checked `docker history`/`docker images -a`:
  only one custom-committed layer exists (dated Sept 9, 166MB, matches the
  `image-transport-plugins` fix documented in `setup_changes_log.md` item 5) and it's
  intact and in use. No evidence of an orphaned/dangling image holding a different,
  more recent commit.
- **Not a leftover zombie process racing the command.** Found and killed a stale
  `teleop_twist_keyboard` process still registered as a DDS publisher on
  `/unitree_go2/cmd_vel` — didn't fix it (so it wasn't the sole cause, though worth
  having cleaned up regardless).
- **Not a ROS/observation wiring bug.** Added temporary diagnostic logging at both
  ends: `cmd_vel_callback` (write side, in `go2_ros2_bridge.py`) and
  `go2_ctrl.base_vel_cmd()` (read side, the actual function the policy's observation
  manager calls). Confirmed conclusively: publishing `linear.x=0.5` correctly appears
  in `base_vel_cmd_input` immediately, and the observation function reads that exact
  value on every subsequent call — sustained for 280+ consecutive calls (~7 sim-seconds).
  Position moved by 0.000003m (pure float noise) over that entire window.
- **Not an odom-publishing bug.** `publish_odom()`'s `base_pos` comes straight from
  `env.unwrapped.scene["unitree_go2"].data.root_state_w` — the same ground-truth
  physics state `camera_follow()` uses — not a separately-integrated/buggy odometry
  estimate.
- **Not a fundamentally broken policy/checkpoint.** `ckpts/unitree_go2/rough_model_7850.pt`
  exists, is a plausible size (6.88MB) for a trained network.

**Key finding that reframes the whole bug:** checked `odom.csv` from the *earlier*
`full-warehouse`-era captures (`synthetic_dataset_01`-`04`, made before switching to
Office) — the robot **did** travel real distances then (x/y spans from 1.4m up to
17m). So the cmd_vel → policy → actuation pipeline is fundamentally sound in general.
**This bug is specific to something that changed alongside the Office switch** — most
likely the "rough" locomotion policy's height-scanner (terrain raycaster) behaving
badly against Office's floor/geometry (feeding it out-of-distribution observations it
freezes defensively on), or the robot's fixed spawn point physically colliding with
something in Office's layout that an open warehouse floor never had.

**Next step (not yet run):** temporarily switch back to `full-warehouse`, restart, and
confirm `cmd_vel` still produces movement under *today's* current code — this isolates
"Office-specific" from "something else broke today" (today's other changes —
`camera_follow_offset`, the ZED aperture fix — are visual/rendering-only and shouldn't
be able to affect physics, but this test would confirm that assumption rather than
just asserting it).

## Cheat sheet — commands used throughout this session

```bash
# Enter interactively (auto-activates the right env; plain `python` works here)
./isaac-launch.sh enter

# Restart the sim after editing code/cfg (bind-mounted, so files update live —
# only the running process needs restarting). SIGTERM can be slow/swallowed; -9 is
# reliable. NOTE: if launched via `./isaac-launch.sh run` (foreground), killing the
# sim process cascades into a full `docker compose down` via that script's own logic
# — bring it back with `docker compose up -d` (detached) to avoid repeating that.
docker exec isaac-go2-ros2 pkill -9 -f isaac_go2_ros2.py
cd ~/isaac_ws/src/isaac-go2-ros2/docker && \
  AUTO_BUILD=false AUTO_RUN=true ROS_DOMAIN_ID=100 docker compose up -d

# Check whether the sim has finished loading (don't guess from a blank viewport)
docker exec isaac-go2-ros2 bash -lc \
  "source /opt/ros/humble/setup.bash; ros2 topic list 2>/dev/null | grep camera_info"

# Watch first-load shader-compile progress on a new/heavy environment
docker exec isaac-go2-ros2 tail -f \
  $(docker exec isaac-go2-ros2 bash -lc "ls -t /isaac-sim/kit/logs/Kit/Isaac-Sim/4.5/*.log | head -1")

# Record a dataset (auto-numbered folder, Ctrl+C to stop)
python record_dataset.py

# Clean up dataset output (must go through docker exec, not host rm — uid mismatch)
docker exec isaac-go2-ros2 rm -rf /home/ubuntu/dataset/<name>
```

## Files changed this session (all in `~/isaac_ws/src/isaac-go2-ros2/` on `entc`)

| File | Change |
|---|---|
| `record_dataset.py` | **New.** The dataset recorder itself. |
| `synthetic_dataset_guide.md` | **New.** Full technical reference (companion to this file). |
| `go2/go2_sensors.py` | ZED FOV fix (`set_horizontal_aperture()` always called). |
| `isaac_go2_ros2.py` | Added `office` env dispatch branch; `camera_follow_offset` wired through. |
| `env/sim_env.py` | (pre-existing `create_office_env()`/`create_hospital_env()`, now reachable for office) |
| `go2/go2_env.py` | `camera_follow()` now takes a configurable `offset` param. |
| `cfg/sim.yaml` | `env_name: office`; added `camera_follow_offset`. |
| `go2/go2_ctrl.py` | **Temporary diagnostic logging still present** — remove before considering this "clean" (see §6). |
| `ros2/go2_ros2_bridge.py` | **Temporary diagnostic logging still present** in `cmd_vel_callback` — remove once §6 is resolved. |

## Status as of end of session

- ✅ Dataset recorder built, tested, working (auto-numbered output folders).
- ✅ ZED FOV bug fixed and verified.
- ✅ Switched to Office environment, verified visually.
- ✅ Dataset-directory permissions fixed.
- ✅ WebRTC white-screen / viewport camera bug fixed.
- ❌ **Robot does not move via `cmd_vel` in Office** — root cause narrowed to
  Office-specific (height-scanner or spawn collision), full ROS/policy/odom pipeline
  otherwise proven correct. Isolation test (temporarily reconfirm movement under
  `full-warehouse` with today's code) proposed but not yet run.
- ⚠️ Temporary diagnostic print statements left in `go2/go2_ctrl.py` and
  `ros2/go2_ros2_bridge.py` — remove once the movement bug is resolved.
