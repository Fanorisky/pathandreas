# PathAndreas

External **world-awareness** and **pathfinding** process for SA-MP / open.mp.
It never talks to the game SDK. Pawn (or anything else) asks geometric
questions over WebSocket; this process answers them.

Continuation of world-query-service (Locus). The navmesh build that
originally failed here was repaired (see git history for the two root
causes: tile/voxel grid misalignment that prevented all inter-tile
links, and inconsistent .col face windings that erased roads from the
raster). A full-map pedestrian bake now covers every tile, but its
largest connected component is only ~20.8% of the mesh: San Andreas is
genuinely not walkable end to end, and the 58.6% an earlier bake
reported was partly stitched through the sea floor, which the seabed
filter removed. Inter-city walking is therefore routed on the node
graphs (below) rather than on the mesh alone.

```
.cadb / .col  →  triangle mesh  →  BVH or Bullet raycast
                              ↘  Recast (offline) → Detour (runtime)
                              ↘  WebSocket JSON oracle
```

The service is **stateless with respect to the game**. No NPC, no player, no
AI. It is an oracle: *is there a wall here? what is ground Z? what is a
walkable path from A to B?*

## Layout

```
pathandreas/
├── CMakeLists.txt
├── Makefile                  # no-CMake build (built-in BVH)
├── src/
│   ├── collision_loader/     # Fase 1 — CADB + COL1/2/3/4 + test city
│   ├── collision_world/      # Fase 2 — SAH BVH (default) or Bullet
│   ├── navmesh_builder/      # Fase 3 — tiled Recast → WQS1 .navmesh
│   ├── pathfinder/           # Fase 4 — Detour findPath / moveAlongSurface
│   ├── query_server/         # Fase 5 — WebSocket + HTTP JSON
│   └── main.cpp
├── tools/navmesh_builder_cli.cpp
├── tests/test_all.cpp
└── third_party/              # Recast+Detour, nlohmann/json
```

## Build

Requires g++ with C++17 and pthread. Recast/Detour and nlohmann/json are
vendored.

```bash
make -j$(nproc)
make test
```

CMake (optional Bullet):

```bash
cmake -S . -B build -DWQS_USE_BULLET=ON
cmake --build build -j
```

`WQS_USE_BULLET=ON` FetchContents bullet3 and builds `btBvhTriangleMeshShape`.
The default backend is a static SAH BVH with the same `RayCastLine` /
`FindGroundZ` API — lighter to deploy on a 2 vCPU / 4 GB VPS.

## Offline navmesh

Bake once per map. Do **not** bake at server start.

```bash
./build/navmesh_builder --test-city --out data/test_city.navmesh
./build/navmesh_builder --cadb /path/to/ColAndreas.cadb --out data/sa.navmesh --tile-size 128
```

Tile size is in GTA world units. Default **128** (not 500): at `cs=0.3` a 500²
tile is ~1667 voxels on a side and will blow RAM during Recast. 128 wu ≈ 427
vx/tile, ~47×47 tiles for the full SA map, well within Detour’s 14-bit tile
index.

Agent defaults (GTA SA ped):

| param | value | why |
|---|---|---|
| cs / ch | 0.3 / 0.2 | voxel size |
| walkableSlopeAngle | 45° | |
| agentHeight | 2.0 | ped capsule |
| agentClimb | 0.9 | SA stair riser |
| agentRadius | 0.6 | |

Coordinates: **GTA Z-up** on the wire. Recast is Y-up internally; conversion
is isolated in `gtaToRecast` / `recastToGta`.

Multi-level geometry (overpass, interiors) is handled by Recast’s stacked
spans — no per-floor split required for typical SA. Water should be omitted
from the walkable mesh (don’t place those models, or strip them before bake).

## Run

```bash
# Synthetic city (no GTA files) — collision + navmesh + server
./build/navmesh_builder --test-city --out /tmp/city.navmesh
./build/pathandreas --mesh-test-city --navmesh /tmp/city.navmesh --port 8090

# Production-shaped
./build/pathandreas --cadb scriptfiles/colandreas/ColAndreas.cadb \
    --navmesh data/sa.navmesh --paths data/paths \
    --navmesh-vehicle data/gta_vehicle.navmesh \
    --bind 0.0.0.0 --port 8090 --threads 4
```

`--paths DIR` wants the game's own `NODES0.DAT` .. `NODES63.DAT` (from the
GTA SA install's `data/paths`). It supersedes `--roads data/GPS.dat`, which
stays supported for installs without those files but carries neither the
pedestrian graph nor lane data. There is no authentication or rate limiting
on the listener - bind it to localhost, or keep it behind something that has
both, since any client can trigger a multi-minute `world_commit`.

## Protocol

WebSocket (text frames) or `POST /query`. Field `id` is echoed so the caller
can correlate async replies. One socket per open.mp process is enough;
requests on that socket run on a thread pool.

**raycast**
```json
{"type":"raycast","id":"req-1","from":[x,y,z],"to":[x,y,z]}
{"type":"raycast_result","id":"req-1","hit":true,"point":[x,y,z],"normal":[x,y,z]}
```

**ground Z**
```json
{"type":"find_ground_z","id":"req-2","x":0,"y":0}
{"type":"find_ground_z_result","id":"req-2","found":true,"z":3.1}
```

**path**
```json
{"type":"find_path","id":"req-3","from":[x,y,z],"to":[x,y,z]}
{"type":"find_path_result","id":"req-3","success":true,"partial":false,"waypoints":[[x,y,z],...]}
```

`partial: true` means the goal is unreachable on the current navmesh and the
waypoints lead only part of the way (Detour partial result). Treat it as
"no route", not as a route.

**hybrid path** (walking; all three backends combined)
```json
{"type":"find_hybrid_path","id":"req-5","from":[x,y,z],"to":[x,y,z]}
{"type":"find_hybrid_path_result","id":"req-5","success":true,
 "graph":"ped+vehicle","waypoints":[[x,y,z],...],
 "repaired_segments":246,"straight_segments":2}
```
The three backends fail in opposite places, so this combines them rather than
picking one. The **pedestrian node graph** is dense and complete inside a city
but splits per city; the **road graph** is one connected component but its
nodes are carriageway centre lines; the **navmesh** reaches anywhere there is
geometry but only ~20.8% of it is one walkable component.

1. Sidewalks wherever the pedestrian graph reaches. At Grove Street its
   nearest node is 1.4 units away where the nearest road node is 10.7 units
   out in the road.
2. The road graph only for the stretch between the component the start is in
   and the one the goal is in. Handover happens where a sidewalk node is
   actually within 30 units of the corridor, so the route does not "hand over"
   in open countryside.
3. The navmesh pulls the route tight through that corridor. This is the part
   that makes the nodes a *reference* rather than a track: from each point the
   route reaches for the furthest corridor node the navmesh can get to by a
   path no longer than the corridor between them allows, so a run of nodes
   across open ground collapses into one direct mesh leg. Where the whole trip
   is one connected mesh region it becomes the navmesh's own near-optimal path
   with the nodes skipped entirely; where the mesh fragments, the route falls
   back to the next node and marks that hop unverified. The goal is allowed to
   sit a few units off the mesh - a kerb, a slope - so a near-straight walk is
   not dragged back onto a detouring corridor for the sake of the last step.

Tracing the corridor node by node used to hug the road/sidewalk centre and
zig-zag through every node. Pulling it tight is both shorter and far
straighter, and where the mesh is connected it matches routing on the mesh
alone:

| route | node-by-node | pulled | turning |
|---|---|---|---|
| Grove -> Pershing Square | 1,888u | 1,140u | 35.8 -> 5.7 rad |
| a reported LV trip (150u apart) | 357u | 157u | 9.3 -> 1.7 rad |
| Grove -> San Fierro | 8,129u | 5,868u | - |
| San Fierro -> Las Venturas | 11,102u | 5,013u | - |

`graph` reports which networks carried it: `"ped"`, `"vehicle"` or
`"ped+vehicle"`. `straight_segments` is the number of hops the navmesh could
not confirm - the only number that matters for a controller, since that is
where recovery mode is still needed - and `longest_unconfirmed` bounds how far
one such hop runs. Across the map that is 2-7 hops of a few hundred; inside a
city it is usually zero.

The pull costs a handful of navmesh queries per connected run (it doubles its
reach rather than stepping node by node): 150-230 ms across the whole map,
a few ms inside a city. `"repair": false` skips it and returns the sparse node
backbone at ~`minSpacing` (25 unit) spacing.

Waypoint 0 and the last are your exact endpoint positions. Semantics
otherwise: a graph-following route to the node nearest the goal, not exact
reachability - the final approach into an off-graph goal is the consumer's
business. Pair with move_along_surface per tick; when a tick stops making
progress, switch to direct movement + find_ground_z until the next waypoint.
That was verified end to end before confirmation existed (a simulated
pedestrian walked Grove Street -> San Fierro, 7,005 units, 2,813 ticks, 0.7%
recovery ticks, arriving 0.2 units from the goal); with confirmation the same
route leaves 2 of 248 hops unchecked, so recovery is now a safety net for a
named handful of stretches rather than for anywhere the mesh happens to break.

**vehicle path** (road network)
```json
{"type":"find_vehicle_path","id":"req-5b","from":[x,y,z],"to":[x,y,z]}
{"type":"find_vehicle_path_result","id":"req-5b","success":true,
 "waypoints":[[x,y,z],...],
 "lanes":[2,2,1,...], "has_lane_data":true,
 "offroad_start":{"distance":45.5,"drivable":true},
 "offroad_goal":{"distance":7.2,"drivable":true}}
```
With `--paths`, `lanes[i]` is the number of lanes available driving from
waypoint `i` to `i+1` - what a consumer needs to offset an NPC into its own
lane instead of straddling the centre line. It is 0 on off-road and
mesh-routed legs, which have no lane data. Lane counts are also how San
Andreas records one-way streets, so the default profile refuses to enter a
segment with no lane in the direction of travel; three optional fields
override the profile per query:

| field | default | effect |
|---|---|---|
| `one_way` | `true` | `false` lets the route drive against the lanes |
| `allow_emergency` | `true` | `false` restricts to nodes not flagged emergency-only |
| `highway_cost` | `0.8` | edge cost multiplier on highway nodes; `1.0` disables the freeway preference |

### Vehicle dimensions

Send a `vehicle` object and the answer is checked against that vehicle instead
of a generic car. Every field is optional; `turn_radius` 0 (the default) means
"do not check corners" - the service will not invent one from the length, since
the caller knows its own steering lock.

```json
{"type":"find_vehicle_path","id":"req-5c","from":[x,y,z],"to":[x,y,z],
 "vehicle":{"width":2.5,"length":9.0,"height":3.6,"turn_radius":12.0}}
```
```json
"offroad_start":{"distance":69.8,"drivable":false,"reason":"width","routed":"mesh"},
"vehicle_check":{
  "width":2.5,"length":9.0,"height":3.6,"turn_radius":12.0,
  "measured_waypoints":122,"min_clearance":0.98,
  "low_clearance":[{"index":15,"height":0.98}],
  "min_turn_radius":5.31,"tight_turns":[{"index":88,"radius":6.4}],
  "mesh_agent_radius":1.5,"mesh_agent_height":2.5,"exceeds_mesh_agent":true}
```

**width** turns the off-road ground check from a line into a corridor: the
ground is sampled across the track as well as down the centre, and a leg fails
when the cross-slope is too steep - which is also what a gap too narrow to fit
through looks like, since the outer samples land on whatever forms the gap.
This is not cosmetic. Over 400 random off-road legs, the old width-blind check
passed 214; a 2.0-wide car passes 189 of the same legs, a 2.6-wide truck 186.
So ~6% of legs it used to call drivable are not, for a car. `reason` now says
which check failed: `no_ground`, `step` or `width`.

**height** scans overhead clearance at every waypoint. `min_clearance` is a
lower bound - when nothing is found within the vehicle's height, that height is
reported, so the number always means "at least this much room everywhere" and
larger is always better. A reported clearance equal to the vehicle's height
means it just touches. Across all 27,083 car nodes, 5 have something within
2.5 units overhead and 419 within 6.0, so this matters for tall vehicles and
almost never for cars.

**turn_radius** flags corners the vehicle cannot take. Read `min_turn_radius`
and `tight_turns` as curvature of the polyline the service returned, **not** of
the road: a consumer that splines the route can take a wider line than these
numbers suggest. They mean "slow down or cut wide here", not "this route is
impossible". Both lists are capped at 32 entries.

**mesh_agent_radius / height** appear when a car navmesh is loaded. Detour
bakes the agent radius into the mesh by eroding it, so a mesh-routed leg is
only valid for a vehicle that fits the bake; `exceeds_mesh_agent` says when the
vehicle described is bigger than what the loaded mesh was built for. The
service reports the profile it was told, not one it measured - a `.navmesh`
file does not record its agent parameters. A genuinely correct answer for
trucks would need a second bake at a larger radius.

The checks cost a raycast per waypoint (~7 us), so they only run when a
`vehicle` object is present; without it the response is byte-for-byte what it
was before. `find_offroad_path` accepts the same object.

Waypoints are traffic-node positions (road centrelines) bracketed by the
caller's exact endpoint positions. The node graph covers roads and dirt roads
(most "off-road" spots are under 100 units from a node; beaches and mountain
slopes are the exceptions), but the legs from the endpoints to their nearest
nodes are straight lines the graph knows nothing about - the offroad_start /
offroad_goal objects report each leg's length and a coarse drivability check
(ground sampled every ~5 units; no ground or height steps over ~31 degrees
fail it). `drivable: false` means a car cannot simply drive that line. With
`--navmesh-vehicle` (a second navmesh baked with car-agent parameters -
`--radius 1.5 --agent-height 2.5 --agent-climb 0.5 --slope 30 --cs 0.4`)
such legs are routed on that mesh instead and the waypoints are spliced
into the route: `routed: "mesh"`. `routed: "straight"` keeps the plain
line - either it was drivable, or no car-mesh route exists either.

**pure off-road path** (requires `--navmesh-vehicle`)
```json
{"type":"find_offroad_path","id":"req-7","from":[x,y,z],"to":[x,y,z]}
{"type":"find_offroad_path_result","id":"req-7","success":true,"partial":false,"waypoints":[[x,y,z],...]}
```
Car-mesh routing for trips that never touch a road (beach to beach,
across open desert).

**nearest node**
```json
{"type":"nearest_node","id":"req-6","pos":[x,y,z],"graph":"vehicle"}
{"type":"nearest_node_result","id":"req-6","found":true,"graph":"vehicle",
 "node":4653,"pos":[x,y,z],"distance":10.7,"flags":8194,
 "highway":true,"emergency":false,"parking":false,"boat":false,"node_type":1}
```
`graph` is `"vehicle"` (default) or `"ped"`. The same profile overrides as
`find_vehicle_path` apply, so a car will not snap to the boat network and a
query can refuse emergency-only nodes. The decoded class booleans are only
returned for the vehicle graph - pedestrian nodes reuse those bits for
something else, so only their raw `flags` is reported.

**boat path** (requires `--paths`)
```json
{"type":"find_boat_path","id":"req-6b","from":[x,y,z],"to":[x,y,z]}
{"type":"find_boat_path_result","id":"req-6b","success":true,"waypoints":[[x,y,z],...]}
```
San Andreas keeps its boat network in the same files as the roads, as nodes
of type 2: 1,507 nodes forming one connected component over the water. LS
coast -> San Fierro bay is 152 waypoints / ~4,500 units.

**move along surface**
```json
{"type":"move_along_surface","id":"req-4","from":[x,y,z],"delta":[dx,dy,dz]}
{"type":"move_along_surface_result","id":"req-4","position":[x,y,z]}
```

**world editing** (requires `--cadb`; `--col` / `--test-city` have no placement
data to edit)
```json
{"type":"world_remove_object","id":"req-8","model":1410,"pos":[x,y,z],"radius":15}
{"type":"world_remove_object_result","id":"req-8","matched":3,"pending_removes":1}

{"type":"world_add_object","id":"req-9","model":1498,"pos":[x,y,z],"rot":[0,0,45]}
{"type":"world_add_object_result","id":"req-9","pending_adds":1}

{"type":"world_edits","id":"req-10"}
{"type":"world_edits_result","id":"req-10","removes":1,"adds":1,"committing":false}

{"type":"world_commit","id":"req-11"}
{"type":"world_commit_result","id":"req-11","started":true}

{"type":"world_reset","id":"req-12"}
```
Mirrors the RemoveBuilding / CreateObject calls a server script makes, so the
pathfinder stops routing around geometry the players cannot see. `matched` is
how many stock placements the removal hit (0 means the model id or radius is
wrong). `rot` is SA-MP euler degrees and is optional. Added objects take their
collision from the same CADB, so only stock model ids work.

Edits are recorded instantly but **take effect only on `world_commit`**, which
re-assembles the world mesh and re-bakes the collision world and both
navmeshes: seconds on the synthetic city, minutes on the full map. The commit
runs in the background - queries keep answering from the old world for its
whole duration and the new one appears in a single atomic swap - so poll
`world_edits` for `committing: false` rather than expecting `world_commit` to
block. A commit re-bakes with the pedestrian parameters from the command line
and the documented car profile, since a `.navmesh` file does not record the
agent profile it was baked with.

The usual pattern is to replay the server's map edits once at boot, commit,
and only commit again when the map actually changes.

Also: `{"type":"status","id":"..."}` and `{"type":"ping","id":"..."}`.
`GET /health` returns a status JSON.

`dtNavMesh` is shared read-only. Each worker thread owns a `dtNavMeshQuery`.

## CADB notes

Magic `cadf` (ColAndreas Wizard) and the older documented `Cskp` layout are
both accepted. Parser is original (format description, not a source port).
See LICENSE for GPL implications if you later vendor ColAndreas code.

`.col` files are model-local. World assembly needs IPL placements — that is
what CADB already contains.

Bakes drop triangles fully below z -1.5: the GTA sea floor is collision
geometry, flat enough to rasterize as perfectly walkable, and without the
filter NPCs route - and walk - underwater on both meshes.

## Node graphs

San Andreas ships its own path networks, and they are a better routing
backend than anything derived from geometry. Two loaders are supported.

**`--paths DIR` (preferred): the game's `NODES0.DAT` .. `NODES63.DAT`.**
One file per 750x750 unit square, holding two separate graphs plus the
per-segment data the game's traffic AI uses. Measured over the stock files:

| | nodes | directed edges |
|---|---|---|
| vehicles | 30,587 | 62,936 |
| pedestrians | 37,650 | 80,686 |

62,932 of the vehicle edges carry lane counts, and 12,287 of them have no
lane in the direction of travel - that is how the format records one-way
streets, since the link table itself is undirected. GPS.dat reports only 961.

Under the car profile the vehicle graph is **one connected component of
27,083 nodes**. The pedestrian graph is deliberately not: it has 179
components, the largest being Los Santos (8,880), San Fierro (8,332) and
Las Venturas (7,567), because vanilla San Andreas has no sidewalk between
cities. 3,605 pedestrian nodes sit above z 500 - interior paths, which the
navmesh bake excludes. So `find_hybrid_path` walks the pedestrian graph
inside a city and falls back to the road graph between them.

Flag bit 8 is documented as "emergency vehicles only" and does mark 7,669
nodes forming continuous chains of their own. Excluding them is still not the
default: over 300 random trips at least 500 units apart, dropping those nodes
left 47% of routes longer, 23% more than a fifth longer, and 11 with no route
at all. `allow_emergency: false` gives a strictly civilian network of 20,766
nodes, 99.5% of it one component, for consumers that want it.

Boat paths are in the same files as nodes of type 2 (1,507 nodes, one
component) and are reachable through `find_boat_path`.

The binary layout comes from the public GTAMods description ("Paths (GTA
SA)"); this reader is original. Two blocks the description leaves open were
measured against the stock files instead: the constant block after the link
array is 768 bytes (`FF FF 00 00` x192, byte-identical in all 64 files) and
the trailing block is exactly `linkCount + 384` bytes. The loader checks the
whole file size against those strides up front, so a wrong assumption fails
loudly rather than shifting the lane data by a few bytes.

**`--roads data/GPS.dat` (fallback): the samp-gps text export.**
~27.6k vehicle nodes, one connected component, no pedestrian graph, no flags,
no lanes - it looks like the largest drivable component of the vehicle graph
with the boat nodes removed. Useful when the game's path files are not
available; the file format only, the loader here is original.

Routing costs: a cross-city vehicle route is ~1.5 ms on the SA graph
(LS -> San Fierro, 7,207 units), an intra-city pedestrian route ~0.5 ms.

## Route audit

Everything above was verified by hand at some point: scratch tools, run once,
never committed. `tools/route_audit.cpp` is that verification made repeatable.

```bash
make audit                     # or:
./build/route_audit --cadb data/ColAndreas.cadb --navmesh data/gta.navmesh \
    --paths paths/Paths [--walk-sim] [--update-baseline]
```

It links the routing code directly rather than talking to a running server, so
it sees lane counts, off-road leg reasons and waypoint arrays instead of JSON.
110 cases: 20 hand-picked in `tests/route_cases.txt` - the places this project
has already had trouble with, named rather than left to chance - plus 30 seeded
random cases per kind, generated from node positions so the same data files
always give the same corpus.

Each case reduces to a verdict of **categorical flags**, never measurements:
`route=ok src=ped+vehicle unverified=low`. A verdict survives a cost-function
tweak; "this route is 7,207 units long" would not, and a check that breaks
whenever anything is tuned gets deleted rather than fixed. Verdicts are
compared against a committed baseline, so a case that is known to fail **stays
known** - it is recorded with its failure, and the audit only complains when a
verdict changes. Both regressions and fixes are reported; only a regression
sets a non-zero exit code.

A failure means the service answered wrongly or unusably: no route, a route
through ungrounded space, endpoints that do not match the request, travel
against the lanes, a boat route off the water, an unconfirmed hop over 60
units, or a walk that does not actually complete. Off-road leg reasons and low
clearance are *recorded but not failures* - "the last 40 units to your goal are
not drivable in a straight line" and "this tunnel has 0.98 units of headroom"
are correct answers about the world, and treating them as defects would bury
the real ones. Current state: 100 pass, 10 known failures.

`--walk-sim` additionally walks every walking route tick by tick the way a
consumer must - navmesh movement, and direct movement until the **next**
waypoint whenever a tick stalls - and reports whether the NPC arrives and how
much of the trip needed recovery. It keeps its own baseline, since simulation
adds flags the plain run cannot compare against. All 8 hand-picked walks and 28
of 28 routable random walks arrive; 7 of those 28 need recovery for more than
5% of their ticks, which is the honest remaining weakness.

Note on ungrounded routes: this cannot be an absolute height test. San Andreas
has road tunnels down at z -46 that are below sea level and perfectly solid.
What separates a tunnel from open water is that a tunnel has ground directly
under the waypoint.

**This cannot run in CI.** It needs `ColAndreas.cadb`, the baked navmeshes and
the game's `NODES*.DAT`, none of which may be redistributed. `make test` covers
what can be checked synthetically; this is a local gate to run before
committing anything that touches routing. The baselines store only case ids and
verdicts - no coordinates, nothing derived from the game files.

## What this is not

NPC movement, animations, decision-making, streaming, and Pawn natives live
on the game server. This process only answers spatial questions.
