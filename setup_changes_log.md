# Isaac Go2 ROS2 Simulation - Fixes & Setup Documentation
*Generated on: 2026-09-09*

This document outlines the modifications made to the `isaac-go2-ros2` repository to achieve a stable, automated, and remote-accessible simulation environment.

## 1. Resolved CUDA Runtime Crash (PyTorch Threading Conflict)
**Issue:** The simulation would randomly crash with `RuntimeError: CUDA error: an illegal memory access was encountered` during operation. 
**Cause:** The ROS2 `foxglove_bridge` background thread (running at 400Hz via a ROS Timer) was trying to read PyTorch GPU tensors (`imu_data.ang_vel_b`) at the exact same time the main simulation loop was writing physics updates to them.
**Solution Applied in `ros2/go2_ros2_bridge.py`:**
- Removed the background asynchronous timer for IMU publishing (`self.create_timer(1.0/400.0, self.imu_timer_callback)`).
- Migrated IMU publishing into the synchronous `pub_ros2_data` loop.
- The IMU is now safely published inline with the main simulation physics steps, entirely eliminating the CUDA memory collisions.

## 2. Optimized Camera Streaming for Remote Visualization (Foxglove over Tailscale)
**Issue:** The 3D camera panels in Foxglove would freeze and state "Waiting for image messages". The raw image frames (6MB each) were overwhelming the Foxglove WebSocket when streaming over a remote Tailscale connection.
**Solution Applied:**
- Installed the `ros-humble-image-transport-plugins` package into the container.
- Configured a ROS2 `image_transport` republisher node to automatically compress the raw camera feed into JPEG format before it hits Foxglove.
- **Result:** The camera stream is now buttery smooth and fully responsive over remote networks by subscribing to the `/zed2i/zed_node/left/image_rect_color/compressed` topic.

## 3. Automated Foxglove & Compression Background Launch
**Issue:** The user had to manually spawn `foxglove_bridge` and the `image_transport` nodes every time the simulation was started.
**Solution Applied in `docker/entrypoint.sh`:**
- Injected the launch commands to spawn both `foxglove_bridge` and `image_transport` in the background (`&`) at container boot.
- Restructured the execution order so the background tasks are initialized *before* the main blocking `isaac_go2_ros2.py` simulation loop takes over the terminal.
- **Result:** The entire visualization stack is 100% automated and ready the second the container boots.

## 4. Localized Docker Entrypoint Execution
**Issue:** Any edits made to `entrypoint.sh` on the host machine were ignored because the script was rigidly baked into the Docker image during the initial `docker build`.
**Solution Applied in `docker/docker-compose.yaml`:**
- Added a dynamic volume mount: `- ./entrypoint.sh:/entrypoint.sh:ro`.
- **Result:** The container now strictly uses the live `entrypoint.sh` from the host repository, allowing instantaneous script edits without ever needing to rebuild the massive Isaac Sim image.

## 5. Permanent Docker Image Baking (Bypassing Broken IsaacLab Upstreams)
**Issue:** The official `IsaacLab` repository `main` branch recently introduced a strict block on `Isaac Sim 4.5.0` and bumped their minimum Python requirement to 3.11 (`tomllib`). Rebuilding the Dockerfile from scratch caused fatal compilation errors inside the container.
**Solution Applied:**
- Reverted the `Dockerfile` to its original safe state to prevent upstream dependency corruption.
- Utilized `docker commit` on the live, running container to physically bake the installed `image-transport-plugins` directly into the user's `seanchangx/isaac-go2-ros2:humble` base image.
- **Result:** The user possesses a perfectly functioning, customized Docker image that maintains all new features without being subjected to IsaacLab's currently broken installation pipeline.
