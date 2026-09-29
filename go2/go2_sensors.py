import omni
import numpy as np
from pxr import Gf
import omni.replicator.core as rep
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils

class SensorManager:
    def __init__(self, num_envs):
        self.num_envs = num_envs

    def add_rtx_lidar(self):
        lidar_annotators = []
        for env_idx in range(self.num_envs):
            _, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path="/lidar",
                parent=f"/World/envs/env_{env_idx}/Go2/base",
                config="Hesai_XT32_SD10",
                # config="Velodyne_VLS128",
                translation=(0.2, 0, 0.2),
                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
            )

            annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
            hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")
            annotator.attach(hydra_texture.path)
            lidar_annotators.append(annotator)
        return lidar_annotators

    def add_camera(self, freq):
        cameras = []
        for env_idx in range(self.num_envs):
            camera = Camera(
                prim_path=f"/World/envs/env_{env_idx}/Go2/base/front_cam",
                translation=np.array([0.4, 0.0, 0.2]),
                frequency=freq,
                resolution=(640, 480),
                orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
            )
            camera.initialize()
            camera.set_focal_length(1.5)
            cameras.append(camera)
        return cameras

    # Real ZED 2i wide-lens spec (Stereolabs datasheet): ~110 deg horizontal FOV
    # at focal_length_mm=2.1. Solving horizontal_aperture = 2*f*tan(FOV/2) for
    # FOV=110deg gives ~6.0mm. This is what Isaac Sim's Camera class needs to
    # actually reproduce that FOV -- its own USD schema default (20.955mm) is
    # ~3.5x too wide and was silently in effect whenever horizontal_aperture_mm
    # wasn't overridden, producing a near-fisheye ~157deg FOV. That's why the
    # camera was effectively looking almost straight down: the bottom of every
    # frame was point-blank/past-the-edge floor, not a rendering glitch.
    DEFAULT_ZED_HORIZONTAL_APERTURE_MM = 6.0

    def add_zed_camera(self, freq, baseline=0.12, resolution=(1280, 720),
                        mount_translation=np.array([0.35, 0.0, 0.15]),
                        mount_orientation_deg=np.array([0.0, 0.0, 0.0]),
                        focal_length_mm=2.1, horizontal_aperture_mm=None):
        """
        ZED 2i stereo pair.

        baseline: 0.12 m is the real ZED 2i spec, but pull the exact measured
            baseline for YOUR unit from:
                zed.get_camera_information().camera_configuration.calibration_parameters.stereo_transform
        resolution: match whichever mode you actually run on the real robot
            (2208x1242@15, 1920x1080@30, 1280x720@60, 672x376@100).
        mount_translation / mount_orientation_deg: measure these on the real
            Go2 relative to base_link — these are placeholders.
        focal_length_mm: 2.1 is the real ZED 2i wide-lens spec (use 4.0 if you
            have the narrow-lens variant).
        horizontal_aperture_mm: defaults to DEFAULT_ZED_HORIZONTAL_APERTURE_MM
            (~110 deg FOV, matches the real ZED 2i wide lens) when not given.
            For exact intrinsics matching YOUR unit's real calibration instead
            of just the nominal spec, pull fx and the sensor width from:
                zed.get_camera_information().camera_configuration.calibration_parameters.left_cam
            and pass horizontal_aperture_mm = focal_length_mm * resolution[0] / fx
            so IsaacLab's pinhole model reproduces the same fx after publish_camera_info().
            Either way, this is always applied via set_horizontal_aperture(),
            which also conforms verticalAperture to match `resolution`'s aspect
            ratio -- required so the render doesn't fill only part of the frame.
        """
        zed_left, zed_right = [], []
        orientation = rot_utils.euler_angles_to_quats(mount_orientation_deg, degrees=True)
        for env_idx in range(self.num_envs):
            left = Camera(
                prim_path=f"/World/envs/env_{env_idx}/Go2/base/zed_left_cam",
                translation=mount_translation + np.array([0.0, baseline / 2.0, 0.0]),
                frequency=freq,
                resolution=resolution,
                orientation=orientation,
            )
            right = Camera(
                prim_path=f"/World/envs/env_{env_idx}/Go2/base/zed_right_cam",
                translation=mount_translation + np.array([0.0, -baseline / 2.0, 0.0]),
                frequency=freq,
                resolution=resolution,
                orientation=orientation,
            )
            left.initialize()
            right.initialize()
            left.set_focal_length(focal_length_mm / 10.0)   # Camera API expects cm
            right.set_focal_length(focal_length_mm / 10.0)

            # Always call set_horizontal_aperture() -- it's the only thing that
            # makes Isaac Sim conform verticalAperture to `resolution`'s aspect
            # ratio too (see Camera.set_horizontal_aperture in
            # isaacsim.sensors.camera). Use the real calibrated aperture if
            # given, otherwise the realistic ZED-like default above -- NOT
            # Isaac's raw ~21mm USD schema default, which produces a near-
            # fisheye FOV far wider than the real sensor.
            h_aperture_mm = (
                horizontal_aperture_mm
                if horizontal_aperture_mm is not None
                else self.DEFAULT_ZED_HORIZONTAL_APERTURE_MM
            )
            left.set_horizontal_aperture(h_aperture_mm / 10.0)   # Camera API expects cm
            right.set_horizontal_aperture(h_aperture_mm / 10.0)

            zed_left.append(left)
            zed_right.append(right)
        return zed_left, zed_right