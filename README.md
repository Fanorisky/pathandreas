# PathAndreas

External **world-awareness** and **pathfinding** process for SA-MP / open.mp.
It never talks to the game SDK. Pawn (or anything else) asks geometric
questions over WebSocket; this process answers them.

Continuation of world-query-service (Locus). The navmesh build that
originally failed here was repaired (see git history for the two root
causes: tile/voxel grid misalignment that prevented all inter-tile
links, and inconsistent .col face windings that erased roads from the
raster). After the fixes a full-map bake has one connected component
covering 58.6% of all polygons; San Fierro and Las Venturas are still
isolated (bridge sidewalks vs agent radius) and are the next work
items.

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
world-query-service/
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
./build/world-query-service --mesh-test-city --navmesh /tmp/city.navmesh --port 8090

# Production-shaped
./build/world-query-service --cadb scriptfiles/colandreas/ColAndreas.cadb \
    --navmesh data/sa.navmesh --roads data/GPS.dat \
    --bind 0.0.0.0 --port 8090 --threads 4
```

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

**hybrid path** (walking, requires `--roads`; macro road route + grounded waypoints)
```json
{"type":"find_hybrid_path","id":"req-5","from":[x,y,z],"to":[x,y,z]}
{"type":"find_hybrid_path_result","id":"req-5","success":true,"waypoints":[[x,y,z],...]}
```
Long-distance pedestrian routing: the traffic node graph picks the corridor
(it is connected where the navmesh fragments between cities) and every
waypoint is ground-snapped, at ~25-unit spacing. Waypoint 0 and the last are
your exact endpoint positions. Semantics: guaranteed road-following route to
the node nearest the goal - not exact reachability; the final approach into
an off-road goal is the consumer's business. Pair with move_along_surface
per tick; when a tick stops making progress (navmesh gap), switch to direct
movement + find_ground_z until the next waypoint - verified end to end:
a simulated pedestrian walked Grove Street -> San Fierro (7,005 units,
2,813 movement ticks, 0.7% recovery ticks) arriving 0.2 units from the goal.

**vehicle path** (road network, requires `--roads`)
```json
{"type":"find_vehicle_path","id":"req-5b","from":[x,y,z],"to":[x,y,z]}
{"type":"find_vehicle_path_result","id":"req-5b","success":true,"waypoints":[[x,y,z],...]}
```
Waypoints are traffic-node positions (road centrelines) and the first/last are
the nodes nearest to from/to - endpoints may sit tens of units from the exact
positions. Combine with `find_path` (navmesh) for the on-foot legs.

**nearest road node** (requires `--roads`)
```json
{"type":"nearest_node","id":"req-6","pos":[x,y,z]}
{"type":"nearest_node_result","id":"req-6","found":true,"node":4653,"pos":[x,y,z]}
```

**move along surface**
```json
{"type":"move_along_surface","id":"req-4","from":[x,y,z],"delta":[dx,dy,dz]}
{"type":"move_along_surface_result","id":"req-4","position":[x,y,z]}
```

Also: `{"type":"status","id":"..."}` and `{"type":"ping","id":"..."}`.
`GET /health` returns a status JSON.

`dtNavMesh` is shared read-only. Each worker thread owns a `dtNavMeshQuery`.

## CADB notes

Magic `cadf` (ColAndreas Wizard) and the older documented `Cskp` layout are
both accepted. Parser is original (format description, not a source port).
See LICENSE for GPL implications if you later vendor ColAndreas code.

`.col` files are model-local. World assembly needs IPL placements — that is
what CADB already contains.

## Road network (GPS.dat)

`--roads data/GPS.dat` loads the GTA SA traffic node graph - the road network
the game's vehicle AI drives on (~27.6k nodes, one connected component
covering every drivable road including the inter-city bridges). The file is
the GPS.dat format distributed with the samp-gps plugin releases (the file
format only; the loader here is original). The graph is the routing backend
for vehicles; the navmesh remains the backend for pedestrians. A full
LS -> San Fierro vehicle route (~7000 world units, 551 nodes) computes in
~40 ms.

## What this is not

NPC movement, animations, decision-making, streaming, and Pawn natives live
on the game server. This process only answers spatial questions.
