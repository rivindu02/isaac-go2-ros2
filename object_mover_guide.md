# Moving objects in the sim (dynamic environment)

These scripts move a few objects in the running Isaac Sim scene, so the Go2 sees a changed environment. Use them to test scene-graph updates, change detection and relocalization.

- An object **jumps** to its new place. It does not slide, and nothing pushes it.
- Only the objects you choose move. Everything else stays where it is.
- A moved object is set down on the floor and keeps its collision. The robot cannot walk through it.
- The moves are not saved to the USD file. Restarting the sim brings back the original layout.

---

## 1. Quick start

Run these on the entc host from `~/isaac_ws/src/isaac-go2-ros2`, with the sim container running.

```bash
./sim_objects.sh list                         # what can move, and where it is now
./sim_objects.sh move --random 3              # move 3 random objects to free places
./sim_objects.sh move --move forklift=0,-5,0 --dry-run   # check a point first
./sim_objects.sh move --move forklift=0,-5,0             # then move it
./sim_objects.sh move --move forklift=home     # one object back to its original place
./sim_objects.sh move --reset                 # every object back to the original layout
```

`./sim_objects.sh` runs the Python tools inside the `isaac-go2-ros2` container with ROS 2 already set up. To see every option, run:

```bash
./sim_objects.sh list --help
./sim_objects.sh move --help
```

---

## 2. Requirements

- The sim is running (`docker ps` shows `isaac-go2-ros2`) and has finished booting. Booting takes about 90 s.
- `object_mover.enable: True` is set in `cfg/sim.yaml`. This is the default.
- When the scan is done, the sim log shows a line like this:

  ```
  [ObjectMover] 368 movable objects, 1338 static obstacles under /World/Warehouse, placement bounds [...]
  ```

  To check: `docker logs isaac-go2-ros2 2>&1 | grep "ObjectMover\] [0-9]"`

The mover works with the `full-warehouse` scene and the other warehouse variants. The office and hospital scenes are untested.

---

## 3. Listing objects

```bash
./sim_objects.sh list                     # every movable object
./sim_objects.sh list --label forklift    # filter by label or id (text match, any case)
./sim_objects.sh list --label Box_19
./sim_objects.sh list --json              # raw inventory, including obstacles and bounds
```

Example output:

```
id                               label             x        y     yaw    size x*y
S_TrafficCone2                   cone          -4.08     9.33     0.0     0.3*0.3
forklift                         forklift       3.18    10.80    90.0     1.2*3.5
368 movable objects, 1338 static obstacles, bounds x [-26.5, 5.5] y [-23.4, 30.6]
```

| Column | Meaning |
|---|---|
| `id` | Name to use in `--move`. It is the USD prim name. |
| `label` | Object kind: box, cardbox, palette, bottle, barrel, cone, crate, cart, bucket, forklift |
| `x`, `y` | Centre of the object, in sim world metres |
| `yaw` | Rotation in degrees |
| `size x*y` | Footprint in metres, measured along the object's own axes |
| `bounds` | Area objects can be placed in: the outline of the warehouse walls |

**Which objects can move.** In `full-warehouse`, 368 objects can move: boxes, pallets, bottles, barrels, traffic cones, crates, a push cart, a bucket and one forklift. These stay fixed:

- racks, walls, pillars, lights and the fuse box;
- any object whose base is higher than 0.3 m, for example boxes on rack shelves or the upper layers of a stack.

The `allow` and `deny` patterns in `cfg/sim.yaml` control this (see section 8).

**Frames.** Coordinates are the **sim world frame**. This is the same frame as `/unitree_go2/odom` and `/unitree_go2/pose`. It is not the gravity-aligned scene-graph map frame. To convert, use `T_align` from `data/processed/<scene>/gravity_alignment.json`, as `waypoint_follower.py` does.

---

## 4. Moving objects

### 4.1 Random

```bash
./sim_objects.sh move                                   # same as --random 3
./sim_objects.sh move --random 5
./sim_objects.sh move --random 2 --seed 7               # same seed = same objects and places
./sim_objects.sh move --random 1 --label forklift
./sim_objects.sh move --random 3 --label cone
```

For each chosen object, the script tries its named spots first (section 6), in random order. If none of them has room, it picks a random place with room inside the walls, with a random yaw. "Room" is decided by the same check as in section 5, so random moves never put an object into something.

`--random` never picks an object that has another object resting on it. Moving that object would leave the one on top floating.

### 4.2 Choosing the object and the place

```bash
./sim_objects.sh move --move forklift=0,-5           # x,y; keeps the current yaw
./sim_objects.sh move --move forklift=0,-5,45        # x,y,yaw (degrees)
./sim_objects.sh move --move S_TrafficCone2=aisle_a  # a named spot from the spot file
./sim_objects.sh move --move Box_19299               # the script picks a place with room
```

You can use `--move` more than once in one command. You can also combine it with `--random`:

```bash
./sim_objects.sh move --move forklift=0,-5,0 --move S_TrafficCone2=1,-2 --random 2
```

The new places are checked against each other, so two objects in one command never land on top of each other.

### 4.3 Back to the original place

```bash
./sim_objects.sh move --move forklift=home     # one object
./sim_objects.sh move --reset                  # every object
```

Both put back the exact original (authored) transform and collision.

In the original scene some objects touch others. The forklift's forks are *inside* the pallet stack it carries on the rack. So `=home` still runs the room check, but only to report:

```
OK: forklift back to its original place (3.18, 10.80, yaw 90), touching as in the original scene: Box_40748, Box_40750, ...
```

`=home` is refused only when an object **you moved** now stands in that place:

```
skip: forklift: its original place is taken by moved object(s) ['S_TrafficCone3'] (move them first, use --reset, or add --force)
```

Send both back in one command: `--move S_TrafficCone3=home --move forklift=home`.

Typing the original coordinates (`forklift=3.18,10.8,90`) is checked like any other point, so it is **rejected**: the forks would be inside those boxes. Use `=home` to put an object back.

---

## 5. Checking that a point is acceptable

**Run `--dry-run` before you move anything to a point you picked yourself.** It runs every check, prints the result, and moves nothing.

```bash
./sim_objects.sh move --move forklift=6,3,90 --dry-run
```

```
REJECTED: forklift at (6.00, 3.00, yaw 90)
    - outside the warehouse walls (x [-26.0, 5.0] y [-22.9, 30.1])
    - hits SM_PillarPartA_9M4_1446
    - hits SM_WallA_6M14
    - hits SM_WallA_6M15
skip: forklift: nearest point with room: --move forklift=3.28,3.37,90  (or add --force)
```

```bash
./sim_objects.sh move --move forklift=3.28,3.37,90 --dry-run
```

```
OK: forklift at (3.28, 3.37, yaw 90) has room
```

**How "room" is decided.** The sim tests the object's real shape with PhysX:

1. **Shape.** The object's visible mesh is turned into small upright boxes, about 15 cm columns. They follow its real outline: a forklift keeps its low forks, its tall mast and the open space under its roof.
2. **Test.** Those boxes are placed at the target position and yaw, standing on the floor. PhysX overlap-tests them against **every collider** in the scene: walls, pillars, racks, boxes on shelves, other objects and the robot.
3. **Result.** Everything hit is listed by name. Nothing hit means there is room.

The test is 3-D. An object fits under a shelf if it really fits, and it is rejected if any part would be inside something, for example the forklift's forks inside a wall.

A point is accepted when all of these pass:

| Check | Default | Option |
|---|---|---|
| Nothing hit by the object's shape (3-D, PhysX), with this sideways gap | 0.1 m | `--clearance` |
| Inside the warehouse walls, at least this far in | 0.5 m | `--margin` |
| Gap to the Go2 (from `/unitree_go2/pose`) | 1.0 m | `--robot-clearance` |
| Not on the new place of another object in the same command | 0.1 m | `--clearance` |

- **REJECTED:** the script lists every reason, with the name of everything the object would hit. It also suggests the nearest point with room at the same yaw, found with the same check. Rejected targets are **not moved**.
- **`--force`:** places the object anyway. The output then says `FORCED` and lists the reasons.

Tips:

- **Yaw matters.** The forklift is 1.2 × 3.5 m. At yaw 0 its long side runs along y, and at yaw 90 along x. A narrow aisle may fit only one of these.
- **Leaving the place out** (`--move ID`) is the easiest way to get a place with room.
- **Named spots** get the same check. A spot that had room once can be taken later by another moved object.
- **Each check takes about 2 s**, mostly ROS round trips. The first check of an object also builds its shape, about 0.1 s for the forklift.

---

## 6. Named spots (`cfg/object_spots.yaml`)

You can give each object its own list of places.

1. Create or update the file:

   ```bash
   ./sim_objects.sh list --save
   ```

   Every movable object gets an entry with its `home` pose and an empty `spots` map. Running it again adds new objects and **keeps** the spots you already wrote.

2. Edit the file and add spots as `name: [x, y, yaw_deg]`:

   ```yaml
   forklift:
     home: [3.18, 10.8, 90.0]
     label: forklift
     spots:
       aisle_block: [0.0, -5.0, 0.0]
       near_dock: [3.07, 2.38, 90.0]
   ```

3. Check each spot with `--dry-run`, then use it:

   ```bash
   ./sim_objects.sh move --move forklift=aisle_block --dry-run
   ./sim_objects.sh move --move forklift=aisle_block    # this spot
   ./sim_objects.sh move --move forklift                # a free spot from the list, else a random free place
   ./sim_objects.sh move --random 1 --label forklift    # same
   ```

Notes:

- `--save` rewrites the file. Comments you add below the header are lost.
- Use a different file with `--spots path.yaml`.
- `cfg/object_spots.yaml` has two example spots for `S_TrafficCone2` (`aisle_a`, `aisle_b`). Delete them if you don't need them.

---

## 7. Logs: ground truth for change detection

Every real move or reset writes a log file:

```
logs/object_moves/<YYYYmmdd_HHMMSS>_<request_id>.json
```

| Key | Content |
|---|---|
| `request` | What was sent to the sim |
| `planned` | Each move: `id`, `x`, `y`, `yaw`, `source` (`given`, a spot name, or `random`) |
| `result` | `moved` and `failed` lists from the sim |
| `before` | Every object's pose before the move |
| `after` | Every object's pose after the move |

Use `before` and `after` to score whether the scene graph found the right changed objects. Only the objects in `planned` change; all others keep the same pose.

A typical experiment:

```bash
./sim_objects.sh move --reset
# 1. build the map / scene graph of the original layout
./sim_objects.sh move --random 3 --seed 1      # repeatable change
# 2. run the robot, detect the changes, compare with the newest log in logs/object_moves/
./sim_objects.sh move --reset
```

---

## 8. Settings (`cfg/sim.yaml`)

```yaml
object_mover:
  enable: True
  allow: "Forklift|CardBox|Carton|Box|Pallet|Palette|Barel|Barrel|Bucket|Cone|Sign|Crate|Bottle|Bin|Extinguisher|Trolley|Cart"
  deny: "Fuse|Rack|Shelf|Shelves|Wall|Floor|Ceiling|Light|Lamp|Pillar|Column|Beam|Door|Window|Roof|Stair"
  floor_z: 0.0
  max_base_z: 0.3
  bounds: null
  wall: "Wall"
  max_obstacle_area: 40.0
  resync_physics: True
```

| Key | Meaning |
|---|---|
| `enable` | Start the mover with the sim |
| `allow` | Regex for prim names that can move (any case). The top-most matching prim moves as one piece. |
| `deny` | Never move these, even if `allow` matches |
| `floor_z` | Height the moved object's base is set down at |
| `max_base_z` | Objects whose base starts higher than this cannot move (on racks, stacked) |
| `bounds` | `[xmin, ymin, xmax, ymax]` placement area. `null` uses the outline of the `wall` prims. |
| `wall` | Regex for the building's wall prims, used for `bounds` |
| `max_obstacle_area` | Larger static prims (floor, ceiling) are left out of the inventory's obstacle list (used to find the walls; the room check itself uses PhysX) |
| `resync_physics` | Rebuild the collision of a moved object (see section 9). Keep `True`. |

Changes to `cfg/sim.yaml` need a sim restart:

```bash
docker restart isaac-go2-ros2     # about 90 s until the mover scan line appears
```

---

## 9. How it works

```
 host / container shell                                 Isaac Sim process (isaac_go2_ros2.py)
 ----------------------                                 -------------------------------------
 list_sim_objects.py  <-- /sim/objects (latched JSON) --  ros2/object_mover.py (ObjectMover)
 move_sim_objects.py  --> /sim/move_objects (JSON)    -->   queue -> applied in main loop
                      <-- /sim/move_objects/result    --    (USD edits on Kit main thread)
```

| File | Role |
|---|---|
| `ros2/object_mover.py` | Runs inside the sim. It scans the scene at startup, publishes the inventory, and applies moves. |
| `list_sim_objects.py` | Lists objects and writes the spot file |
| `move_sim_objects.py` | Picks objects and targets, runs the checks, sends the request, writes the log |
| `object_placement.py` | Pure helpers (no ROS, no Isaac): client-side checks (walls, robot, new places), candidate poses, and the mesh-to-boxes maths the sim uses for its room check |
| `sim_objects.sh` | Host wrapper that runs the tools in the container and gives the output files back to your user |
| `tests/test_object_placement.py` | Unit tests for `object_placement.py` |

How a move is applied:

0. The client asks the sim for a room check at the target (section 5) and only moves the object if there is room.
1. The prim's transform is replaced. The object turns about its own centre to the new yaw, its centre goes to (x, y), and its base is set on `floor_z`.
2. **Physics resync.** After a transform edit, PhysX moves the shape that ray-casts see. But the robot's contacts with some moved colliders (the forklift's convex hull) stayed at the old place, so the Go2 walked straight through the forklift. So the moved prims are switched off for one sim step and switched on again. PhysX then rebuilds their collision at the new place. The result is sent once this is done, about two sim steps after the request.

Topics, all `std_msgs/String` with a JSON payload:

| Topic | Direction | Content |
|---|---|---|
| `/sim/objects` | sim → | `bounds`, `objects` (id, label, path, x, y, yaw, half, zmin, zmax, rigid, collider), `obstacles` + `obstacle_names`. Latched, and published again after every move. |
| `/sim/move_objects` | → sim | Move: `{"request_id", "moves": [{"id", "x", "y", "yaw"}], "home": [id, ...]}`. Reset: `{"request_id", "reset": true}`. Optional `"probe": [[x, y, z], ...]`. Room check, which moves nothing: `{"request_id", "check": [{"id", "x", "y", "yaw"}, ...], "clearance", "ignore": [id, ...], "first": bool}` |
| `/sim/move_objects/result` | sim → | `{"request_id", "moved", "failed", "probe", "check": [{"id", "x", "y", "yaw", "ok", "hits"}]}` |

Any ROS 2 node on `ROS_DOMAIN_ID=100` can use these topics directly. For example, a test script can trigger a change in the middle of a run.

---

## 10. Debugging

**Which collider is at a point?** This sends a PhysX ray straight down from height z (default 3 m):

```bash
./sim_objects.sh move --probe=4.5,0,1.0
#   probe (4.50, 0.00): /World/Warehouse/forklift/S_ForkliftBody at z 1.00
#   probe (7.00, 3.00): /World/ground/GroundPlane/CollisionPlane at z 0.00   <- empty floor
```

Write `--probe=X,Y` (with `=`) when X is negative. Otherwise the minus sign is read as an option. A ray that starts inside a shape reports a hit at its start height.

**Does the robot really collide with a moved object?** Put the object in front of the robot and walk the robot into it:

```bash
./sim_objects.sh move --move forklift=4.5,0,90 --force
docker exec isaac-go2-ros2 bash -lc 'source /opt/ros/humble/setup.bash; \
  timeout 45 ros2 topic pub -r 10 /unitree_go2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}}"'
# the robot should stop in front of the forklift (x about 2.4), not pass through it
```

**Skip the physics resync** (debug only): `--no-resync`. The object moves on screen, but the robot can still bump into its old place.

| Problem | Cause / fix |
|---|---|
| `no inventory on /sim/objects` | The sim is still booting, `object_mover.enable` is off, or the scene is not supported. Check the `[ObjectMover]` log line. |
| `warning: no /unitree_go2/pose, robot clearance not checked` | The robot pose was not received within 3 s, usually right after boot. Run again. |
| `no result from sim within 15 s` | The sim is stalled or very slow. Check `docker logs isaac-go2-ros2`. |
| An object you expect is missing from `list` | Its name doesn't match `allow`, or its base is above `max_base_z`. Change `cfg/sim.yaml` and restart. |
| `skip: ... no place with room found` | No random place had room (unlikely). Run again, or give a spot. |
| `the running sim cannot check for room yet` | The sim runs older code. Restart it: `docker restart isaac-go2-ros2`. |
| The robot walks through a moved object | `resync_physics` is off, or `--no-resync` was used. |
| `list` shows a different layout than expected | Moves last only until the sim restarts. Run `--reset`, or restart the sim. |

Unit tests (no sim needed):

```bash
docker exec isaac-go2-ros2 bash -lc "cd /home/ubuntu/isaac-go2-ros2 && /usr/bin/python3 -m pytest -q tests/test_object_placement.py"
```
