#include "collision_loader/collision_loader.h"
#include "collision_world/collision_world.h"
#include "navmesh_builder/navmesh_builder.h"
#include "pathfinder/pathfinder.h"
#include "road_network/road_network.h"
#include "query_server/query_server.h"
#include "common/log.h"

#include <cstring>
#include <string>
#include <iostream>

using namespace wqs;

static void usage() {
    std::fprintf(stderr,
        "Locus world-query-service\n"
        "Usage:\n"
        "  world-query-service [options]\n"
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

    CollisionMesh mesh;
    if (testCity) {
        mesh = MakeTestCityMesh();
    } else if (!cadb.empty()) {
        mesh = LoadFromCadb(cadb);
    } else if (!col.empty()) {
        mesh = LoadFromCol(col);
    }

    CollisionWorld world;
    if (mesh.triangleCount() > 0) {
        std::string err;
        if (!world.build(mesh, err)) {
            WQS_ERROR("collision build: %s", err.c_str());
            return 1;
        }
    } else if (navmeshIn.empty()) {
        WQS_WARN("No collision mesh loaded. Use --mesh-test-city, --cadb, or --col.");
    }

    Pathfinder pathfinder;
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

    // Optional car-agent navmesh (larger radius, low climb, shallow slopes):
    // the routing backend for vehicle off-road legs and pure off-road trips.
    Pathfinder vehiclePathfinder;
    if (!navmeshVehicleIn.empty()) {
        std::string err;
        if (!vehiclePathfinder.loadFile(navmeshVehicleIn, err)) {
            WQS_ERROR("vehicle navmesh load: %s", err.c_str());
            return 1;
        }
    }

    QueryServer server(&world, &pathfinder, scfg, &roads,
                       navmeshVehicleIn.empty() ? nullptr : &vehiclePathfinder);
    return server.run();
}
