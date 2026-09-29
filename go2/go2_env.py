from isaaclab.scene import InteractiveSceneCfg
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

from isaaclab.sensors import RayCasterCfg, patterns, ContactSensorCfg, ImuCfg
from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as mdp
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.noise import UniformNoiseCfg
from isaacsim.core.utils.viewports import set_camera_view
import numpy as np
from scipy.spatial.transform import Rotation as R
import go2.go2_ctrl as go2_ctrl


@configclass
class Go2SimCfg(InteractiveSceneCfg):
    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(color=(0.1, 0.1, 0.1), size=(300.0, 300.0)),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0, 0, 1e-4)
        )
    )
    
    # lights
    # Lights
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )
    sky_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )
    # dome_light = AssetBaseCfg(
    #     prim_path="/World/DomeLight",
    #     spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    # )

    # Go2 Robot
    unitree_go2: ArticulationCfg = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Go2")
    # Go2 foot contact sensor
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Go2/.*_foot", history_length=3, track_air_time=True)

    # Go2 height scanner
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Go2/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20)), 
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]), 
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )

    # ZED 2i IMU — attached at the Go2 base for now. If your voxel-mapping model
    # is sensitive to the true IMU-to-base offset (real ZED 2i IMU sits inside the
    # camera housing, not at the robot's base_link), add a dedicated child Xform
    # prim at the measured offset and point prim_path at that instead.
    imu_zed = ImuCfg(
        prim_path="{ENV_REGEX_NS}/Go2/base",
        update_period=0.0,  # sample every physics step; downsample to 400Hz on publish
    )


@configclass
class ActionsCfg:
    """Action specifications for the environment."""
    joint_pos = mdp.JointPositionActionCfg(asset_name="unitree_go2", joint_names=[".*"])

@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel,
                               params={"asset_cfg": SceneEntityCfg(name="unitree_go2")})
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel,
                               params={"asset_cfg": SceneEntityCfg(name="unitree_go2")})
        projected_gravity = ObsTerm(func=mdp.projected_gravity,
                                    params={"asset_cfg": SceneEntityCfg(name="unitree_go2")},
                                    noise=UniformNoiseCfg(n_min=-0.05, n_max=0.05))
        # velocity command
        base_vel_cmd = ObsTerm(func=go2_ctrl.base_vel_cmd)

        joint_pos = ObsTerm(func=mdp.joint_pos_rel,
                            params={"asset_cfg": SceneEntityCfg(name="unitree_go2")})
        joint_vel = ObsTerm(func=mdp.joint_vel_rel,
                            params={"asset_cfg": SceneEntityCfg(name="unitree_go2")})
        actions = ObsTerm(func=mdp.last_action)
        
        # Height scan
        height_scan = ObsTerm(func=mdp.height_scan,
                              params={"sensor_cfg": SceneEntityCfg("height_scanner")},
                              clip=(-1.0, 1.0))

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()

@configclass
class CommandsCfg:
    """Command specifications for the MDP."""
    base_vel_cmd = mdp.UniformVelocityCommandCfg(
        asset_name="unitree_go2",
        resampling_time_range=(0.0, 0.0),
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 0.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.0, 0.0), heading=(0, 0)
        ),
    )

@configclass
class EventCfg:
    """Configuration for events."""
    pass

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""
    pass


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""
    pass

@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""
    pass



@configclass
class Go2RSLEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the Go2 environment."""
    # scene settings
    scene = Go2SimCfg(num_envs=2, env_spacing=2.0)

    # basic settings
    observations = ObservationsCfg()
    actions = ActionsCfg()
    
    # dummy settings
    commands = CommandsCfg()
    rewards = RewardsCfg()
    terminations = TerminationsCfg()
    events = EventCfg()
    curriculum = CurriculumCfg()

    def __post_init__(self):
        # viewer settings
        self.viewer.eye = [-4.0, 0.0, 5.0]
        self.viewer.lookat = [0.0, 0.0, 0.0]

        # step settings
        self.decimation = 8  # step

        # simulation settings
        self.sim.dt = 0.005  # sim step every 
        self.sim.render_interval = self.decimation  
        self.sim.disable_contact_processing = True
        self.sim.render.antialiasing_mode = None
        # self.sim.physics_material = self.scene.terrain.physics_material

        # settings for rsl env control
        self.episode_length_s = 20.0 # can be ignored
        self.is_finite_horizon = False
        self.actions.joint_pos.scale = 0.25

        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt

def camera_follow(env, offset=(-4.0, 0.0, 5.0)):
    """Position the main viewport (follow) camera relative to the robot.

    `offset` was hardcoded to (-4.0, 0.0, 5.0) -- 4m behind, 5m above the
    robot's root position. That's fine under the warehouse's tall roof
    trusses (~8-10m ceiling), but for a lower-ceiling indoor scene (office,
    hospital, ~2.4-3m ceiling) a 5m-high camera ends up ABOVE the roof,
    outside the building, rendering nothing but exterior/void -- a
    permanently blank/white viewport no matter what the robot does. Pass a
    lower offset (e.g. (-3.0, 0.0, 2.0)) for indoor low-ceiling
    environments; see cfg.camera_follow_offset in cfg/sim.yaml.
    """
    if (env.unwrapped.scene.num_envs == 1):
        robot_position = env.unwrapped.scene["unitree_go2"].data.root_state_w[0, :3].cpu().numpy()
        robot_orientation = env.unwrapped.scene["unitree_go2"].data.root_state_w[0, 3:7].cpu().numpy()
        rotation = R.from_quat([robot_orientation[1], robot_orientation[2],
                                robot_orientation[3], robot_orientation[0]])
        yaw = rotation.as_euler('zyx')[0]
        yaw_rotation = R.from_euler('z', yaw).as_matrix()
        set_camera_view(
            yaw_rotation.dot(np.asarray(offset)) + robot_position,
            robot_position
        )