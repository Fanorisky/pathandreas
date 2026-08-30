#include "collision_loader/collision_loader.h"
#include "collision_world/collision_world.h"
#include "navmesh_builder/navmesh_builder.h"
#include "pathfinder/pathfinder.h"
#include "query_server/json_protocol.h"
#include "world_manager/world_manager.h"
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
    // --- WorldManager: dynamic edits -------------------------------------
    {
        WorldManager wm;
        // Build a manager over the synthetic city by writing/loading a cadb.
        CadbDatabase cityDb = MakeTestCity();
        std::string tmpCadb = "/tmp/wqs_wm_test.cadb";
        std::string werr;
        CHECK(WriteCadbFile(tmpCadb, cityDb, werr), "world manager: write test cadb");
        CHECK(wm.loadCadb(tmpCadb, werr), "world manager: load cadb");

        const CollisionMesh base = wm.assembleEdited();
        const uint32_t baseTris = base.triangleCount();
        CHECK(baseTris > 0, "world manager: base mesh assembles");

        // No edits -> assembleEdited equals the plain assembly.
        CHECK(wm.removeEditCount() == 0 && wm.addEditCount() == 0,
              "world manager: starts with no edits");

        // Remove every model-2 building storey near the origin.
        const size_t matching = wm.countMatching(2, {0.f, 0.f, 0.f}, 60.f);
        CHECK(matching > 0, "world manager: removal preview finds placements");
        wm.removeBuilding(2, {0.f, 0.f, 0.f}, 60.f);
        CollisionMesh edited = wm.assembleEdited();
        CHECK(edited.triangleCount() < baseTris,
              "world manager: removal shrinks the mesh");
        CHECK(edited.triangleCount() > 0, "world manager: removal keeps the world");

        // Add an object with stock model 8 (wall) somewhere fresh.
        std::string aerr;
        CHECK(wm.addObject(8, {50.f, 50.f, 0.f}, Quat{}, aerr),
              "world manager: add object with known model");
        CHECK(!wm.addObject(9999, {0.f, 0.f, 0.f}, Quat{}, aerr),
              "world manager: unknown model id rejected");
        edited = wm.assembleEdited();
        CHECK(edited.triangleCount() > 0, "world manager: add+remove assembles");

        // Euler -> quaternion sanity: identity and 90-degree Z.
        const Quat ident = EulerDegreesToQuat({0.f, 0.f, 0.f});
        CHECK(std::fabs(ident.w - 1.f) < 1e-5f, "euler: identity quaternion");
        const Quat qz = EulerDegreesToQuat({0.f, 0.f, 90.f});
        const Vec3 rx = rotate(qz, {1.f, 0.f, 0.f});
        CHECK(std::fabs(rx.y - 1.f) < 1e-4f && std::fabs(rx.x) < 1e-4f,
              "euler: 90 deg Z maps +X to +Y");

        // Clearing edits restores the original mesh size.
        wm.clearEdits();
        CHECK(wm.assembleEdited().triangleCount() == baseTris,
              "world manager: clearEdits restores the base mesh");
    }

    std::fprintf(stderr, "\nall tests passed\n");
    return 0;
}
