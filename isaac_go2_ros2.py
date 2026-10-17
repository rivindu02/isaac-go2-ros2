import os
import hydra
import rclpy
import torch
import time
import math
import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on running the cartpole RL environment.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, extra_args = parser.parse_known_args()
# ISAAC_GUI=1 (isaac-launch.sh --gui): open a window on $DISPLAY instead of the WebRTC livestream
isaac_gui = os.environ.get("ISAAC_GUI", "0") == "1"
args_cli.headless = not isaac_gui
args_cli.enable_cameras = True
args_cli.livestream = 0 if isaac_gui else 2

import sys
import os
original_argv = sys.argv.copy()
os.environ["PUBLIC_IP"] = os.environ.get("SERVER_IP", "100.110.199.98")

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
sys.argv = original_argv + ["hydra.run.dir=/tmp/outputs/${now:%Y-%m-%d}/${now:%H-%M-%S}"]

"""Rest everything follows."""

import torch

from go2.go2_env import Go2RSLEnvCfg, camera_follow
import env.sim_env as sim_env
import go2.go2_sensors as go2_sensors
import omni
import carb
import go2.go2_ctrl as go2_ctrl
import ros2.go2_ros2_bridge as go2_ros2_bridge

FILE_PATH = os.path.join(os.path.dirname(__file__), "cfg")
@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):

    # Go2 Environment setup
    go2_env_cfg = Go2RSLEnvCfg()
    go2_env_cfg.scene.num_envs = cfg.num_envs
    go2_env_cfg.decimation = math.ceil(1./go2_env_cfg.sim.dt/cfg.freq)
    go2_env_cfg.sim.render_interval = go2_env_cfg.decimation
    if cfg.env_name == "office":
        # The default spawn (0,0,0.4) coincides with a cluster of phantom
        # furniture-prototype collision registered near world origin in this
        # asset (confirmed via per-foot raycast diagnostics: hits a drinks
        # machine/toilet/printer/etc that are nowhere near this location
        # visually -- a floor-prim bbox scan confirms all of Office's real
        # furniture sits 10+ meters away from the origin). That phantom
        # collision pins the robot in place regardless of cmd_vel. Relocate
        # spawn to open floor confirmed clear by that scan, before the
        # articulation is constructed (default_root_state is baked from
        # init_state at construction time, so this must happen here rather
        # than after get_rsl_rough_policy()). dataclasses.replace (not direct
        # attribute mutation) avoids mutating the init_state object shared by
        # reference with the global UNITREE_GO2_CFG.
        import dataclasses
        go2_env_cfg.scene.unitree_go2.init_state = dataclasses.replace(
            go2_env_cfg.scene.unitree_go2.init_state, pos=(2.0, 5.0, 0.4)
        )
    go2_ctrl.init_base_vel_cmd(cfg.num_envs)
    # env, policy = go2_ctrl.get_rsl_flat_policy(go2_env_cfg)
    env, policy = go2_ctrl.get_rsl_rough_policy(go2_env_cfg)

    # Simulation environment
    if (cfg.env_name == "obstacle-dense"):
        sim_env.create_obstacle_dense_env() # obstacles dense
    elif (cfg.env_name == "obstacle-medium"):
        sim_env.create_obstacle_medium_env() # obstacles medium
    elif (cfg.env_name == "obstacle-sparse"):
        sim_env.create_obstacle_sparse_env() # obstacles sparse
    elif (cfg.env_name == "warehouse"):
        sim_env.create_warehouse_env() # warehouse
    elif (cfg.env_name == "warehouse-forklifts"):
        sim_env.create_warehouse_forklifts_env() # warehouse forklifts
    elif (cfg.env_name == "warehouse-shelves"):
        sim_env.create_warehouse_shelves_env() # warehouse shelves
    elif (cfg.env_name == "full-warehouse"):
        sim_env.create_full_warehouse_env() # full warehouse
    elif (cfg.env_name == "office"):
        sim_env.create_office_env() # office
    elif (cfg.env_name == "dataset"):
        sim_env.create_dataset_env(cfg.dataset_path) # custom dataset mesh
    # Sensor setup
    sm = go2_sensors.SensorManager(cfg.num_envs)
    # Each sensor adds a render product that renders every step, so only create enabled ones.
    lidar_annotators = sm.add_rtx_lidar() if cfg.sensor.enable_lidar else []
    cameras = sm.add_camera(cfg.freq) if cfg.sensor.enable_camera else []

    # ZED 2i stereo pair — only created if enabled in cfg/sim.yaml (sensor.enable_zed: True).
    # Fill in the real per-unit baseline/focal-length/mount values, see go2_sensors.py.
    zed_cameras = None
    if getattr(cfg.sensor, "enable_zed", False):
        zed_left, zed_right = sm.add_zed_camera(cfg.freq)
        zed_cameras = (zed_left, zed_right)

    # Keyboard control
    system_input = carb.input.acquire_input_interface()
    system_input.subscribe_to_keyboard_events(
        omni.appwindow.get_default_app_window().get_keyboard(), go2_ctrl.sub_keyboard_event)
    
    # ROS2 Bridge
    rclpy.init()
    dm = go2_ros2_bridge.RobotDataManager(env, lidar_annotators, cameras, cfg, zed_cameras=zed_cameras)
    
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(dm,), daemon=True)
    spin_thread.start()

    # Object mover: re-place scene objects on command (see ros2/object_mover.py)
    mover = None
    mover_roots = {"warehouse": "/World/Warehouse", "warehouse-forklifts": "/World/Warehouse",
                   "warehouse-shelves": "/World/Warehouse", "full-warehouse": "/World/Warehouse",
                   "office": "/World/Office", "hospital": "/World/Hospital"}
    mover_cfg = cfg.get("object_mover", None)
    if mover_cfg is not None and mover_cfg.get("enable", False) and cfg.env_name in mover_roots:
        from rclpy.executors import SingleThreadedExecutor
        from ros2.object_mover import ObjectMover
        mover = ObjectMover(mover_cfg, mover_roots[cfg.env_name])
        # own executor: the global one is already spun by spin_thread
        mover_executor = SingleThreadedExecutor()
        mover_executor.add_node(mover)
        threading.Thread(target=mover_executor.spin, daemon=True).start()

    # Run simulation
    sim_step_dt = float(go2_env_cfg.sim.dt * go2_env_cfg.decimation)
    obs, _ = env.reset()
    while simulation_app.is_running():
        start_time = time.time()
        with torch.inference_mode():            
            # control joints
            actions = policy(obs)

            # step the environment
            obs, _, _, _ = env.step(actions)

            # # ROS2 data
            dm.pub_ros2_data()

            # apply queued object moves (USD edits must run on this thread)
            if mover is not None:
                mover.step()

            # Camera follow
            if (cfg.camera_follow):
                camera_follow(env, offset=tuple(getattr(cfg, "camera_follow_offset", (-4.0, 0.0, 5.0))))

            # limit loop time
            elapsed_time = time.time() - start_time
            if elapsed_time < sim_step_dt:
                sleep_duration = sim_step_dt - elapsed_time
                time.sleep(sleep_duration)
        actual_loop_time = time.time() - start_time
        rtf = min(1.0, sim_step_dt/elapsed_time)
        print(f"\rStep time: {actual_loop_time*1000:.2f}ms, Real Time Factor: {rtf:.2f}", end='', flush=True)
    
    dm.destroy_node()
    rclpy.shutdown()
    simulation_app.close()

if __name__ == "__main__":
    run_simulator()
    