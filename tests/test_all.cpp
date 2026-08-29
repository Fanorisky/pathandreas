#include "collision_loader/collision_loader.h"
#include "collision_world/collision_world.h"
#include "navmesh_builder/navmesh_builder.h"
#include "pathfinder/pathfinder.h"
#include "query_server/json_protocol.h"
#include "common/log.h"

#include <cstdio>
#include <cmath>
#include <string>
#include <cstdlib>

using namespace wqs;

static int gFails = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); ++gFails; } \
    else { std::fprintf(stderr, "ok   %s\n", msg); } \
} while (0)

int main() {
    // --- Phase 1: CADB roundtrip + test city ---
    CadbDatabase city = MakeTestCity();
    CHECK(!city.models.empty(), "test city has models");
    CHECK(!city.placements.empty(), "test city has placements");

    const char* cadbPath = "/tmp/wqs_test_city.cadb";
    std::string err;
    CHECK(WriteCadbFile(cadbPath, city, err), "write cadb");
    CadbDatabase loaded;
    CHECK(LoadCadbFile(cadbPath, loaded, err), "read cadb");
    CHECK(loaded.models.size() == city.models.size(), "model count roundtrip");
    CHECK(loaded.placements.size() == city.placements.size(), "placement count roundtrip");

    CollisionMesh mesh = AssembleWorldMesh(loaded, {});
    CHECK(mesh.triangleCount() > 100, "assembled mesh has triangles");

    // --- Phase 2: raycast + ground Z ---
    CollisionWorld world;
    CHECK(world.build(mesh, err), "build collision world");

    RayHitResult hit;
    // Open ground away from the plaza platform (which sits at z≈0.8).
    CHECK(world.RayCastLine({50, 0, 10}, {50, 0, -10}, hit) && hit.hit, "downward ray hits ground");
    CHECK(std::fabs(hit.point.z) < 0.6f, "ground hit near z=0");

    float gz = 999.f;
    CHECK(world.FindGroundZ(50.f, 0.f, gz), "FindGroundZ");
    CHECK(std::fabs(gz) < 0.6f, "FindGroundZ near 0");

    // Horizontal ray into a building around (-18, -18) (gx=-1, gy=-1 block).
    RayHitResult wall;
    bool wallHit = world.RayCastLine({-10, -18, 1.5f}, {-30, -18, 1.5f}, wall);
    CHECK(wallHit && wall.hit, "ray into building hits");

    // Open sky: ray straight up from above everything should miss.
    RayHitResult miss;
    CHECK(!world.RayCastLine({1, 1, 50}, {1, 1, 80}, miss), "upward ray misses");

    // --- Phase 3+4: navmesh + path ---
    NavBuildConfig ncfg;
    ncfg.tileWorldSize = 64.f;
    ncfg.cs = 0.4f;
    ncfg.ch = 0.2f;
    const char* navPath = "/tmp/wqs_test_city.navmesh";
    CHECK(BuildNavMeshFile(mesh, navPath, ncfg, err), "build navmesh file");

    Pathfinder pf;
    CHECK(pf.loadFile(navPath, err), "load navmesh");

    PathResult path = pf.FindPath({-10, -12, 1}, {10, -12, 1});
    CHECK(path.success, "FindPath succeeds on open ground");
    CHECK(path.waypoints.size() >= 2, "path has waypoints");

    Vec3 stepped = pf.MoveAlongSurface({0, -10, 1}, {0, 2, 0});
    CHECK(std::isfinite(stepped.x) && std::isfinite(stepped.y), "MoveAlongSurface finite");

    // --- Phase 5: JSON protocol ---
    std::string r = HandleQueryJson(
        R"({"type":"raycast","id":"req-1","from":[1,1,10],"to":[1,1,-10]})",
        &world, &pf);
    CHECK(r.find("raycast_result") != std::string::npos, "json raycast type");
    CHECK(r.find("\"id\":\"req-1\"") != std::string::npos, "json id echoed");
    CHECK(r.find("\"hit\":true") != std::string::npos, "json hit true");

    r = HandleQueryJson(
        R"({"type":"find_ground_z","id":"req-2","x":1,"y":1})",
        &world, &pf);
    CHECK(r.find("find_ground_z_result") != std::string::npos, "json ground z");

    r = HandleQueryJson(
        R"({"type":"find_path","id":"req-3","from":[-10,-12,1],"to":[10,-12,1]})",
        &world, &pf);
    CHECK(r.find("find_path_result") != std::string::npos, "json find_path");
    CHECK(r.find("\"id\":\"req-3\"") != std::string::npos, "json path id");

    r = HandleQueryJson(R"({"type":"nope","id":7})", &world, &pf);
    CHECK(r.find("error") != std::string::npos, "unknown type errors");

    if (gFails) {
        std::fprintf(stderr, "\n%d FAILED\n", gFails);
        return 1;
    }
    std::fprintf(stderr, "\nall tests passed\n");
    return 0;
}
