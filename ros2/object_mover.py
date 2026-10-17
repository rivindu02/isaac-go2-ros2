"""
Teleport selected scene objects (boxes, pallets, forklifts, ...) to new floor
spots on command, to give the robot a changing environment.

Runs inside the sim process (created in isaac_go2_ros2.py when
`object_mover.enable` is set in cfg/sim.yaml). USD edits must happen on the
Kit main thread, so ROS callbacks only queue requests and `step()`, called
once per sim step from the main loop, applies them.

Topics (std_msgs/String, JSON payload):
    /sim/objects              (pub, transient local) inventory:
        {"root", "floor_z", "bounds": [xmin, ymin, xmax, ymax],   # outline of the building's walls
         "objects":   [{"id", "label", "path", "x", "y", "yaw", "half": [hx, hy],
                        "zmin", "zmax", "rigid", "collider", "home": [x, y, yaw]}],
         "obstacles": [[xmin, ymin, xmax, ymax], ...],   # static, not movable
         "obstacle_names": [prim name, ...]}             # same order as obstacles
        x, y = centre of the object's yaw-aligned box, yaw in degrees.
    /sim/move_objects         (sub) request:
        {"request_id": str, "moves": [{"id", "x", "y", "yaw"}]}  or
        {"request_id": str, "reset": true}                 # every object back home
        {"request_id": str, "home": [id, ...], "moves": [...]}   # these back home (+ moves)
        optional "probe": [[x, y(, z)], ...] -> after the move, ray-cast straight
        down from height z (default floor_z + 3 m) and report the collider hit
        {"request_id": str, "check": [{"id", "x", "y", "yaw"}], "clearance": m,
         "ignore": [id, ...], "first": bool}  -> is there room? nothing is moved (see _check);
         with "first" the candidates are tried in order and the reply stops at the first with room
    /sim/move_objects/result  (pub) {"request_id", "moved": [...], "failed": [...],
                                     "probe": [{"x", "y", "hit": collider path or null, "z"}],
                                     "check": [{"id", "x", "y", "yaw", "ok", "hits": [name, ...]}]}

Objects are placed, never animated: the prim transform is overwritten so the
object is rotated by (yaw - current yaw) about its centre, its centre is put
at (x, y), and its base is dropped onto floor_z. The moves are not saved to
the USD file; restarting the sim restores the original layout.

PhysX scene queries follow the edited transform, but robot contacts with a
moved convex-hull collider (e.g. the forklift) stayed at the old pose. So with
resync_physics (default) moved prims are deactivated for one step and then
reactivated, which makes PhysX rebuild their colliders at the new pose. The
result message is sent once that is done (two sim steps after the request).

Room checks use PhysX itself: the object's visible mesh, as small column
boxes, is put at the target pose and overlap-tested against every collider in
the scene. It is 3-D, so a forklift's forks may reach under a shelf. The
clients ask this before every move.

Client tools: list_sim_objects.py, move_sim_objects.py (see sim_objects.sh).
"""
import json
import math
import queue
import re

import carb
import numpy as np
import omni.usd
from omni.physx import get_physx_scene_query_interface
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from object_placement import column_boxes, sample_triangles

SCAN_RETRY_STEPS = 100
SCAN_MAX_TRIES = 10
CHECK_LIFT = 0.03          # m; test boxes are raised this much so they do not touch the floor
VOXEL = 0.15               # m; column size of the object shapes used for room checks
ROBOT_PATH = "/World/envs"


def _get(cfg, key, default):
    return cfg.get(key, default) if hasattr(cfg, "get") else getattr(cfg, key, default)


def _union(boxes):
    return [round(min(b[0] for b in boxes), 3), round(min(b[1] for b in boxes), 3),
            round(max(b[2] for b in boxes), 3), round(max(b[3] for b in boxes), 3)]


class ObjectMover(Node):
    def __init__(self, cfg, root):
        super().__init__("object_mover")
        self.root = root
        self.allow = re.compile(_get(cfg, "allow", "Forklift|CardBox|Pallet|Palette"), re.IGNORECASE)
        self.deny = re.compile(_get(cfg, "deny", "Rack|Shelf|Wall|Floor"), re.IGNORECASE)
        self.floor_z = float(_get(cfg, "floor_z", 0.0))
        self.max_base_z = float(_get(cfg, "max_base_z", 0.3))
        self.max_obstacle_area = float(_get(cfg, "max_obstacle_area", 40.0))
        self.bounds_cfg = _get(cfg, "bounds", None)
        self.wall = re.compile(_get(cfg, "wall", "Wall"), re.IGNORECASE)
        self.resync_physics = bool(_get(cfg, "resync_physics", True))

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.inv_pub = self.create_publisher(String, "/sim/objects", latched)
        self.res_pub = self.create_publisher(String, "/sim/move_objects/result", 10)
        self.create_subscription(String, "/sim/move_objects", self._on_request, 10)

        self._requests = queue.Queue()
        self.objects = {}     # id -> {"path", "label", "home": Gf.Matrix4d local transform}
        self.obstacles = []
        self.obstacle_names = []
        self.bounds = None
        self._steps = 0
        self._scan_tries = 0
        self._scanned = False
        self._pending = None  # (prims to reactivate, request, result) while a resync runs

    # ROS thread
    def _on_request(self, msg):
        self._requests.put(msg.data)

    # Kit main thread
    def step(self):
        self._steps += 1
        if not self._scanned:
            if self._steps % SCAN_RETRY_STEPS == 1 and self._scan_tries < SCAN_MAX_TRIES:
                self._scan_tries += 1
                self._scan()
            return
        if self._pending is not None:
            prims, req, result = self._pending
            if prims:  # step after deactivation: add the prims back
                for prim in prims:
                    prim.SetActive(True)
                self._pending = ([], req, result)
            else:      # one more step so PhysX has rebuilt the colliders before probes run
                self._pending = None
                self._finish(req, result)
        while self._pending is None and not self._requests.empty():
            self._handle(self._requests.get())

    @property
    def stage(self):
        return omni.usd.get_context().get_stage()

    def _bbox_cache(self):
        return UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    def _scan(self):
        root = self.stage.GetPrimAtPath(self.root)
        if not root.IsValid():
            print(f"[ObjectMover] root {self.root} not found")
            return
        cache = self._bbox_cache()
        objects, obstacles, names, floors, seen = {}, [], [], [], {}
        it = iter(Usd.PrimRange(root))
        for prim in it:
            if prim == root or not prim.IsA(UsdGeom.Xformable):
                continue
            name = prim.GetName()
            match = self.allow.search(name)
            is_asset = (prim.IsInstance() or prim.HasAuthoredReferences()
                        or prim.HasAuthoredPayloads() or prim.IsA(UsdGeom.Gprim))
            if not (match or is_asset):
                continue
            it.PruneChildren()
            rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if rng.IsEmpty():
                continue
            lo, hi = rng.GetMin(), rng.GetMax()
            if match and not self.deny.search(name) and lo[2] < self.floor_z + self.max_base_z:
                obj_id = name if name not in seen else f"{name}_{seen[name] + 1}"
                seen[name] = seen.get(name, 0) + 1
                p = self._pose(prim, cache)
                objects[obj_id] = {"path": str(prim.GetPath()), "label": match.group(0).lower(),
                                   "home": UsdGeom.Xformable(prim).GetLocalTransformation(),
                                   "home_pose": [round(p["x"], 3), round(p["y"], 3), round(p["yaw"], 1)]}
            elif (hi[2] > self.floor_z + 0.1 and lo[2] < self.floor_z + 2.0
                  and (hi[0] - lo[0]) * (hi[1] - lo[1]) < self.max_obstacle_area):
                obstacles.append([round(lo[0], 3), round(lo[1], 3), round(hi[0], 3), round(hi[1], 3)])
                names.append(name)
            elif hi[2] <= self.floor_z + 0.1 and (hi[0] - lo[0]) * (hi[1] - lo[1]) >= 1.0:
                floors.append([lo[0], lo[1], hi[0], hi[1]])
        walls = [o for o, n in zip(obstacles, names) if self.wall.search(n)]
        if self.bounds_cfg is not None:
            self.bounds, source = [float(v) for v in self.bounds_cfg], "cfg"
        elif walls:
            # the building's outer walls; the root's bounds include the outer shell
            # and the floor prims reach far past the walls
            self.bounds, source = _union(walls), f"{len(walls)} wall prims"
        elif floors:
            self.bounds, source = _union(floors), f"{len(floors)} floor prims"
        else:
            rng = cache.ComputeWorldBound(root).ComputeAlignedRange()
            self.bounds, source = _union([[rng.GetMin()[0], rng.GetMin()[1], rng.GetMax()[0], rng.GetMax()[1]]]), "root"
        self.objects, self.obstacles, self.obstacle_names = objects, obstacles, names
        if not objects:
            print(f"[ObjectMover] no movable objects under {self.root} yet (try {self._scan_tries})")
            return
        self._scanned = True
        print(f"[ObjectMover] {len(objects)} movable objects, {len(obstacles)} static obstacles under {self.root}, "
              f"placement bounds {self.bounds} (from {source})")
        for obj_id, o in objects.items():
            print(f"[ObjectMover]   {obj_id:30s} {o['path']}")
        self._publish_inventory()

    def _pose(self, prim, cache):
        """Centre (x, y), yaw (deg), half extents in the yaw frame, z range, world matrix."""
        world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        yaw = math.atan2(world[0][1], world[0][0])
        box = cache.ComputeWorldBound(prim)
        m, r = box.GetMatrix(), box.GetRange()
        c, s = math.cos(yaw), math.sin(yaw)
        us, vs, zs = [], [], []
        for i in range(8):
            p = m.Transform(r.GetCorner(i))
            us.append(c * p[0] + s * p[1])
            vs.append(-s * p[0] + c * p[1])
            zs.append(p[2])
        u, v = (min(us) + max(us)) / 2, (min(vs) + max(vs)) / 2
        return {"x": c * u - s * v, "y": s * u + c * v, "yaw": math.degrees(yaw),
                "half": [(max(us) - min(us)) / 2, (max(vs) - min(vs)) / 2],
                "zmin": min(zs), "zmax": max(zs), "world": world}

    def _publish_inventory(self, request_id=None):
        cache = self._bbox_cache()
        objs = []
        for obj_id, o in self.objects.items():
            prim = self.stage.GetPrimAtPath(o["path"])
            p = self._pose(prim, cache)
            sub = list(Usd.PrimRange(prim, Usd.TraverseInstanceProxies()))
            rigid = any(q.HasAPI(UsdPhysics.RigidBodyAPI) for q in sub)
            collider = any(q.HasAPI(UsdPhysics.CollisionAPI) for q in sub)
            objs.append({"id": obj_id, "label": o["label"], "path": o["path"],
                         "x": round(p["x"], 3), "y": round(p["y"], 3), "yaw": round(p["yaw"], 1),
                         "half": [round(h, 3) for h in p["half"]],
                         "zmin": round(p["zmin"], 3), "zmax": round(p["zmax"], 3), "rigid": rigid, "collider": collider,
                         "home": o["home_pose"]})
        msg = {"root": self.root, "floor_z": self.floor_z, "bounds": self.bounds,
               "objects": objs, "obstacles": self.obstacles, "obstacle_names": self.obstacle_names,
               "request_id": request_id}
        self.inv_pub.publish(String(data=json.dumps(msg)))

    def _handle(self, data):
        try:
            req = json.loads(data)
        except json.JSONDecodeError as e:
            self.res_pub.publish(String(data=json.dumps({"request_id": None, "error": str(e)})))
            return
        if req.get("check") is not None:
            clearance = float(req.get("clearance", 0.1))
            ignore = [self.objects[i]["path"] for i in req.get("ignore", []) if i in self.objects]
            first = bool(req.get("first"))   # search: stop at the first candidate with room
            result = {"request_id": req.get("request_id"), "moved": [], "failed": [], "check": []}
            for c in req["check"]:
                try:
                    hits = self._check(c["id"], float(c["x"]), float(c["y"]), float(c["yaw"]),
                                       clearance, ignore, stop_at_first=first)
                    result["check"].append(dict(c, ok=not hits, hits=hits))
                except Exception as e:  # keep the sim alive on a bad request
                    result["check"].append(dict(c, ok=False, hits=[f"error: {e!r}"]))
                if first and result["check"][-1]["ok"]:
                    break
            self.res_pub.publish(String(data=json.dumps(result)))
            return
        moved, failed, prims = [], [], []
        home = list(self.objects) if req.get("reset") else req.get("home", [])
        for obj_id in home:
            o = self.objects.get(obj_id)
            if o is None:
                failed.append({"id": obj_id, "reason": "unknown id"})
                continue
            prim = self.stage.GetPrimAtPath(o["path"])
            xf = UsdGeom.Xformable(prim)
            if xf.GetLocalTransformation() != o["home"]:
                xf.MakeMatrixXform().Set(o["home"])
                prims.append(prim)
            if not req.get("reset") or prim in prims:
                moved.append({"id": obj_id, "home": True})
        if not req.get("reset"):
            cache = self._bbox_cache()
            for mv in req.get("moves", []):
                obj = self.objects.get(mv.get("id"))
                if obj is None:
                    failed.append({"id": mv.get("id"), "reason": "unknown id"})
                    continue
                try:
                    prim = self.stage.GetPrimAtPath(obj["path"])
                    self._place(prim, cache, float(mv["x"]), float(mv["y"]), float(mv.get("yaw", 0.0)))
                    cache.Clear()
                    prims.append(prim)
                    moved.append(mv)
                except Exception as e:  # keep the sim alive on a bad request
                    failed.append({"id": mv.get("id"), "reason": repr(e)})
        result = {"request_id": req.get("request_id"), "moved": moved, "failed": failed}
        if prims and req.get("resync", self.resync_physics):
            # Scene queries follow a moved static collider, but contacts (GPU
            # pipeline) can keep using the old pose: the robot then walks
            # through the object. Removing the prim for one step and adding it
            # back makes PhysX rebuild its colliders at the new pose.
            for prim in prims:
                prim.SetActive(False)
            self._pending = (prims, req, result)
            return
        self._finish(req, result)

    def _finish(self, req, result):
        if req.get("probe"):
            result["probe"] = [self._probe(*[float(v) for v in p]) for p in req["probe"]]
        print(f"[ObjectMover] request {req.get('request_id')}: moved {len(result['moved'])}, "
              f"failed {len(result['failed'])}")
        self._publish_inventory(req.get("request_id"))
        self.res_pub.publish(String(data=json.dumps(result)))

    def _probe(self, x, y, z=None):
        """Collider hit by a PhysX ray cast straight down from (x, y, z), or None.
        Collider edits are picked up by PhysX directly, so this checks that the
        physics shape moved along with the visual one."""
        z = self.floor_z + 3.0 if z is None else z
        hit = get_physx_scene_query_interface().raycast_closest(
            carb.Float3(x, y, z), carb.Float3(0.0, 0.0, -1.0), z - self.floor_z + 0.5)
        if not hit["hit"]:
            return {"x": x, "y": y, "hit": None}
        return {"x": x, "y": y, "hit": hit["collision"], "z": round(hit["position"][2], 3)}

    def _delta(self, p, x, y, yaw, lift=0.0):
        """World-space transform taking the object from its current pose (p) to (x, y, yaw) on the floor."""
        to_origin = Gf.Matrix4d().SetTranslate(Gf.Vec3d(-p["x"], -p["y"], 0.0))
        rotate = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw - p["yaw"]))
        to_target = Gf.Matrix4d().SetTranslate(Gf.Vec3d(x, y, self.floor_z - p["zmin"] + lift))
        # USD row-vector convention: point * to_origin * rotate * to_target
        return to_origin * rotate * to_target

    def _shape(self, obj_id):
        """The object's visible mesh as boxes [umin, vmin, wmin, umax, vmax, wmax] in its own
        frame (u, v along its yaw axes from its centre, w up from its base). Cached.

        Triangles are sampled densely, the points are binned into VOXEL columns, and
        every unbroken vertical run of a column becomes one box fitted to its points.
        So an L-shaped forklift keeps its low forks and the open space under its roof.
        """
        o = self.objects[obj_id]
        if "shape" in o:
            return o["shape"]
        prim = self.stage.GetPrimAtPath(o["path"])
        p = self._pose(prim, self._bbox_cache())
        yaw = math.radians(p["yaw"])
        c, s = math.cos(yaw), math.sin(yaw)
        pts = []
        for q in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
            if not q.IsA(UsdGeom.Mesh):
                continue
            img = UsdGeom.Imageable(q)
            if img.ComputeVisibility() == UsdGeom.Tokens.invisible or \
                    img.ComputePurpose() not in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render):
                continue
            mesh = UsdGeom.Mesh(q)
            points, counts, idx = (mesh.GetPointsAttr().Get(), mesh.GetFaceVertexCountsAttr().Get(),
                                   mesh.GetFaceVertexIndicesAttr().Get())
            if not points or not counts:
                continue
            m = np.array(UsdGeom.Xformable(q).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
            w = np.array(points, dtype=np.float64) @ m[:3, :3] + m[3, :3]
            counts, idx = np.array(counts), np.array(idx)
            pts.append(sample_triangles(w, counts, idx, VOXEL / 2))
        if not pts:
            o["shape"] = np.zeros((0, 6))
            return o["shape"]
        pts = np.concatenate(pts)
        dx, dy = pts[:, 0] - p["x"], pts[:, 1] - p["y"]
        local = np.stack([c * dx + s * dy, -s * dx + c * dy, pts[:, 2] - p["zmin"]], axis=1)
        o["shape"] = column_boxes(local, VOXEL)
        print(f"[ObjectMover] shape of {obj_id}: {len(o['shape'])} boxes from {len(local)} points")
        return o["shape"]

    def _check(self, obj_id, x, y, yaw, clearance, ignore=(), stop_at_first=False):
        """Names of what the object would hit at (x, y, yaw); empty = there is room.

        Every box of _shape is placed at the target pose (base lifted CHECK_LIFT off
        the floor, grown by `clearance` sideways) and PhysX overlap-tests it against
        all colliders in the scene. Hits on the object itself, on objects in `ignore`
        (moving away in the same command) and on the ground plane are skipped.
        """
        o = self.objects[obj_id]
        skip = [o["path"]] + list(ignore)
        hits = set()

        def report(hit):
            path = hit.collision
            if any(path == k or path.startswith(k + "/") for k in skip) or path.startswith("/World/ground"):
                return True
            hits.add(self._name(path))
            return not stop_at_first

        query = get_physx_scene_query_interface()
        r = math.radians(yaw)
        c, s = math.cos(r), math.sin(r)
        rot = carb.Float4(0.0, 0.0, math.sin(r / 2), math.cos(r / 2))
        for b in self._shape(obj_id):
            uc, vc, wc = (b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2
            half = carb.Float3((b[3] - b[0]) / 2 + clearance, (b[4] - b[1]) / 2 + clearance,
                               max((b[5] - b[2]) / 2, 0.005))
            centre = carb.Float3(x + c * uc - s * vc, y + s * uc + c * vc, self.floor_z + CHECK_LIFT + wc)
            query.overlap_box(half, centre, rot, report, False)
            if hits and stop_at_first:
                break
        return sorted(hits)

    def _name(self, path):
        """Readable name for a collider path: the robot, or the asset under the scene root."""
        if path.startswith(ROBOT_PATH):
            return "robot (Go2)"
        if path.startswith(self.root + "/"):
            return path[len(self.root) + 1:].split("/")[0]
        return path

    def _place(self, prim, cache, x, y, yaw):
        p = self._pose(prim, cache)
        world = p["world"] * self._delta(p, x, y, yaw)
        xf = UsdGeom.Xformable(prim)
        parent = xf.ComputeParentToWorldTransform(Usd.TimeCode.Default())
        xf.MakeMatrixXform().Set(world * parent.GetInverse())
