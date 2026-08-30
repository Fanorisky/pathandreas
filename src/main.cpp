#include "collision_loader/collision_loader.h"
#include "collision_world/collision_world.h"
#include "navmesh_builder/navmesh_builder.h"
#include "pathfinder/pathfinder.h"
#include "road_network/road_network.h"
#include "world_manager/world_manager.h"
#include "world_manager/world_committer.h"
#include "query_server/query_server.h"
#include "common/log.h"

#include <cstring>
#include <memory>
#include <string>
#include <iostream>

using namespace wqs;

static void usage() {
    std::fprintf(stderr,
        "PathAndreas\n"
        "Usage:\n"
        "  pathandreas [options]\n"
        "Options:\n"
        "  --cadb PATH          Load ColAndreas .cadb and build collision world\n"
        "  --col PATH           Load raw GTA SA .col (model-local)\n"
        "  --mesh-test-city     Use the built-in synthetic city (no GTA data)\n"
        "  --navmesh PATH       Load a previously built .navmesh (WQS1)\n"
        "  --navmesh-vehicle P  Load a car-agent navmesh for offroad legs\n"
        "  --roads PATH         Load the vehicle road network (GPS.dat format)\n"
        "  --build-navmesh PATH Build navmesh from the loaded collision mesh and save\n"
        "  --tile-size N        Navmesh tile size in world units (default 128)\n"
        "  --threads N          Query pool AND navmesh bake threads (default hardware)\n"
        "  --bind ADDR          Bind address (default 0.0.0.0)\n"
        "  --port N             Listen port (default 8090)\n"
        "  --threads N          Query thread pool size (default hardware)\n"
        "  --help\n");
}

int main(int argc, char** argv) {
    std::string cadb, col, navmeshIn, navmeshOut, roadsFile, navmeshVehicleIn;
    bool testCity = false;
    ServerConfig scfg;
    NavBuildConfig ncfg;

    for (int i = 1; i < argc; ++i) {
        auto next = [&](std::string& dst) {
            if (i + 1 >= argc) return false;
            dst = argv[++i];
            return true;
        };
        if (!std::strcmp(argv[i], "--help") || !std::strcmp(argv[i], "-h")) {
            usage();
            return 0;
        } else if (!std::strcmp(argv[i], "--cadb")) {
            if (!next(cadb)) { usage(); return 2; }
        } else if (!std::strcmp(argv[i], "--col")) {
            if (!next(col)) { usage(); return 2; }
        } else if (!std::strcmp(argv[i], "--mesh-test-city")) {
            testCity = true;
        } else if (!std::strcmp(argv[i], "--navmesh")) {
            if (!next(navmeshIn)) { usage(); return 2; }
        } else if (!std::strcmp(argv[i], "--navmesh-vehicle")) {
            if (!next(navmeshVehicleIn)) { usage(); return 2; }
        } else if (!std::strcmp(argv[i], "--roads")) {
            if (!next(roadsFile)) { usage(); return 2; }
        } else if (!std::strcmp(argv[i], "--build-navmesh")) {
            if (!next(navmeshOut)) { usage(); return 2; }
        } else if (!std::strcmp(argv[i], "--tile-size")) {
            std::string v;
            if (!next(v)) { usage(); return 2; }
            ncfg.tileWorldSize = std::strtof(v.c_str(), nullptr);
        } else if (!std::strcmp(argv[i], "--bind")) {
            if (!next(scfg.bind)) { usage(); return 2; }
        } else if (!std::strcmp(argv[i], "--port")) {
            std::string v;
            if (!next(v)) { usage(); return 2; }
            scfg.port = static_cast<uint16_t>(std::stoi(v));
        } else if (!std::strcmp(argv[i], "--threads")) {
            std::string v;
            if (!next(v)) { usage(); return 2; }
            const unsigned n = static_cast<unsigned>(std::stoi(v));
            // One flag, two pools: the query thread pool at runtime, and the
            // navmesh bake pool when --build-navmesh is used.
            scfg.threads = n;
            ncfg.threads = n;
        } else {
            WQS_ERROR("unknown arg %s", argv[i]);
            usage();
            return 2;
        }
    }

    // With --cadb the world is loaded through WorldManager, which retains the
    // placement database so RemoveBuilding/CreateObject edits can be applied
    // later (world_* queries). --col and --test-city have no placement data to
    // edit, so editing stays disabled for them.
    WorldManager manager;
    CollisionMesh mesh;
    bool editingEnabled = false;
    if (testCity) {
        mesh = MakeTestCityMesh();
    } else if (!cadb.empty()) {
        std::string err;
        if (!manager.loadCadb(cadb, err)) {
            WQS_ERROR("cadb load: %s", err.c_str());
            return 1;
        }
        mesh = manager.assembleEdited();
        editingEnabled = true;
    } else if (!col.empty()) {
        mesh = LoadFromCol(col);
    }

    Backends backends;
    backends.world = std::make_unique<CollisionWorld>();
    if (mesh.triangleCount() > 0) {
        std::string err;
        if (!backends.world->build(mesh, err)) {
            WQS_ERROR("collision build: %s", err.c_str());
            return 1;
        }
    } else if (navmeshIn.empty()) {
        WQS_WARN("No collision mesh loaded. Use --mesh-test-city, --cadb, or --col.");
    }

    backends.pathfinder = std::make_unique<Pathfinder>();
    Pathfinder& pathfinder = *backends.pathfinder;
    if (!navmeshOut.empty()) {
        if (mesh.triangleCount() == 0) {
            WQS_ERROR("--build-navmesh requires a collision mesh");
            return 1;
        }
        std::string err;
        if (!BuildNavMeshFile(mesh, navmeshOut, ncfg, err)) {
            WQS_ERROR("navmesh build: %s", err.c_str());
            return 1;
        }
        navmeshIn = navmeshOut;
    }
    if (!navmeshIn.empty()) {
        std::string err;
        if (!pathfinder.loadFile(navmeshIn, err)) {
            WQS_ERROR("navmesh load: %s", err.c_str());
            return 1;
        }
    }

    RoadNetwork roads;
    if (!roadsFile.empty()) {
        std::string err;
        if (!roads.loadFile(roadsFile, err)) {
            WQS_ERROR("road network: %s", err.c_str());
            return 1;
        }
    }
    backends.roads = &roads;

    // Optional car-agent navmesh (larger radius, low climb, shallow slopes):
    // the routing backend for vehicle off-road legs and pure off-road trips.
    if (!navmeshVehicleIn.empty()) {
        backends.vehiclePathfinder = std::make_unique<Pathfinder>();
        std::string err;
        if (!backends.vehiclePathfinder->loadFile(navmeshVehicleIn, err)) {
            WQS_ERROR("vehicle navmesh load: %s", err.c_str());
            return 1;
        }
    }

    // A world commit re-bakes from scratch, and a .navmesh file does not record
    // the agent profile it was baked with - so commits use the pedestrian
    // parameters given on this command line, and the documented car profile for
    // the vehicle mesh. Bake the input meshes with matching flags to keep a
    // commit from silently changing agent behaviour.
    NavBuildConfig vehicleCfg = ncfg;
    vehicleCfg.cs = 0.4f;
    vehicleCfg.agentRadius = 1.5f;
    vehicleCfg.agentHeight = 2.5f;
    vehicleCfg.agentClimb = 0.5f;
    vehicleCfg.walkableSlopeAngle = 30.f;

    std::unique_ptr<WorldCommitter> committer;
    if (editingEnabled) {
        committer = std::make_unique<WorldCommitter>(
            manager, backends, ncfg, vehicleCfg, backends.vehiclePathfinder != nullptr);
        WQS_INFO("World editing enabled (%zu placements); use world_* queries",
                 manager.placementCount());
    }

    QueryServer server(&backends, scfg, committer.get());
    return server.run();
}
