from isaacsim.core.utils.prims import define_prim, get_prim_at_path
import omni.usd
try:
    import isaacsim.storage.native as nucleus_utils
except ModuleNotFoundError:
    import isaacsim.core.utils.nucleus as nucleus_utils
from isaaclab.terrains import TerrainImporterCfg, TerrainImporter
from isaaclab.terrains import TerrainGeneratorCfg
from env.terrain_cfg import HfUniformDiscreteObstaclesTerrainCfg
import omni.replicator.core as rep

def add_semantic_label():
    ground_plane = rep.get.prims("/World/GroundPlane")
    with ground_plane:
    # Add a semantic label
        rep.modify.semantics([("class", "floor")])

def create_obstacle_sparse_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="random",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=100 ,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain) 

def create_obstacle_medium_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="random",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=200 ,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain) 


def create_obstacle_dense_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="random",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=400,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain) 

def create_dataset_env(dataset_path: str):
    from pxr import UsdPhysics, PhysxSchema
    
    prim = get_prim_at_path("/World/Dataset")
    if not prim.IsValid():
        prim = define_prim("/World/Dataset", "Xform")
        prim.GetReferences().AddReference(dataset_path)
        
        # Apply physics collision so the robot can walk on the mesh
        collision_api = UsdPhysics.CollisionAPI.Apply(prim)
        mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_collision_api.GetApproximationAttr().Set("meshSimplification")

        # Load Semantics and physics OBBs
        load_3rscan_semantics()

def load_3rscan_semantics():
    import json
    import os
    import numpy as np
    import isaacsim.core.utils.semantics as semantics_utils
    from pxr import UsdGeom, Gf, UsdPhysics
    import isaacsim.core.utils.prims as prim_utils
    
    semseg_path = "/home/ubuntu/dataset/3Rscan/3rscan_data/0cac75f4-8d6f-2d13-8c10-ce688f7ac0af/semseg.v2.json"
    if not os.path.exists(semseg_path):
        print(f"[Dataset] Semantics file not found: {semseg_path}")
        return
        
    print(f"[Dataset] Loading 3RScan semantics from {semseg_path}")
    with open(semseg_path, 'r') as f:
        data = json.load(f)
        
    parent_path = "/World/DatasetSemantics"
    define_prim(parent_path, "Xform")
    
    for obj in data.get("segGroups", []):
        obj_id = obj.get("objectId")
        label = obj.get("label")
        obb = obj.get("obb")
        if not obb: continue
        
        centroid = obb.get("centroid")
        axesLengths = obb.get("axesLengths")
        norm_axes = obb.get("normalizedAxes")
        
        # Convert 3x3 rot matrix to quaternion
        orientation = None
        if norm_axes and len(norm_axes) == 9:
            rot_mat = Gf.Matrix3d(
                float(norm_axes[0]), float(norm_axes[1]), float(norm_axes[2]),
                float(norm_axes[3]), float(norm_axes[4]), float(norm_axes[5]),
                float(norm_axes[6]), float(norm_axes[7]), float(norm_axes[8])
            )
            quat = rot_mat.ExtractRotation().GetQuaternion()
            orientation = np.array([quat.GetReal(), quat.GetImaginary()[0], quat.GetImaginary()[1], quat.GetImaginary()[2]])
            
        # Spawn Cube
        clean_label = label.replace(' ', '_').replace('-', '_')
        cube_path = f"{parent_path}/obj_{obj_id}_{clean_label}"
        
        prim = prim_utils.create_prim(
            prim_path=cube_path,
            prim_type="Cube",
            position=np.array(centroid),
            scale=np.array(axesLengths) / 2.0, # UsdGeom.Cube size is 2.0 by default, so we divide by 2
            orientation=orientation
        )
        
        # Make the box invisible so it doesn't block the visual mesh
        UsdGeom.Imageable(prim).MakeInvisible()
        
        # Apply physics collision to the box (Option B)
        UsdPhysics.CollisionAPI.Apply(prim)
        
        # Apply semantic label
        semantics_utils.add_update_semantics(prim, label)
    print(f"[Dataset] Successfully spawned semantic bounding boxes!")

def create_warehouse_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

def create_warehouse_forklifts_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
    prim.GetReferences().AddReference(asset_path)

def create_warehouse_shelves_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
    prim.GetReferences().AddReference(asset_path)

def create_full_warehouse_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

def create_hospital_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Hospital")
    prim = define_prim("/World/Hospital", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Hospital/hospital.usd"
    prim.GetReferences().AddReference(asset_path)

def create_office_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Office")
    prim = define_prim("/World/Office", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Office/office.usd"
    prim.GetReferences().AddReference(asset_path)

    # The Office sample scene ships its own bundled ground-plane collider
    # (/World/Office/GroundPlane), sitting almost exactly coincident with our
    # own scene's ground plane (/World/ground, from Go2SimCfg.ground). Two
    # overlapping static colliders at nearly the same height cause PhysX
    # contact-resolution instability -- foot contact forces end up wildly
    # asymmetric (several times body weight), which pins the robot in place
    # even though cmd_vel is correctly applied. Deactivating the whole prim
    # removes it (and everything nested under it, including instanced
    # descendants) from physics/rendering, leaving only our own ground plane.
    stage = omni.usd.get_context().get_stage()
    ground_plane = stage.GetPrimAtPath("/World/Office/GroundPlane")
    if ground_plane.IsValid():
        ground_plane.SetActive(False)