#include "collision_loader/collision_loader.h"
#include "collision_world/collision_world.h"
#include "navmesh_builder/navmesh_builder.h"
#include "pathfinder/pathfinder.h"
#include "query_server/json_protocol.h"
#include "road_network/road_network.h"
#include "route_planner/route_planner.h"
#include "road_network/sa_paths.h"
#include "world_manager/world_manager.h"
#include "common/log.h"

#include <cstdio>
#include <cmath>
#include <string>
#include <cstdlib>
#include <cstdint>
#include <vector>
#include <sys/stat.h>

using namespace wqs;

static int gFails = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); ++gFails; } \
    else { std::fprintf(stderr, "ok   %s\n", msg); } \
} while (0)


// --- synthetic NODES*.DAT builder -------------------------------------------
// Small enough to reason about by hand: three vehicle nodes in a row where the
// middle-to-right segment is one-way, a boat node, an emergency-only node, and
// a two-node pedestrian path. Everything the loader has to get right - the
// eighths-of-a-unit positions, the vehicle/pedestrian split, the flag bits and
// the lane counts that encode one-way streets - is observable from it.
namespace syn {

void put16(std::vector<uint8_t>& b, uint16_t v) {
    b.push_back(static_cast<uint8_t>(v & 0xFF));
    b.push_back(static_cast<uint8_t>(v >> 8));
}
void put32(std::vector<uint8_t>& b, uint32_t v) {
    for (int i = 0; i < 4; ++i) b.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
}
void putPos(std::vector<uint8_t>& b, float v) {
    put16(b, static_cast<uint16_t>(static_cast<int16_t>(v * 8.f)));
}

struct Node { float x, y, z; uint16_t linkId; uint8_t width, type; uint32_t flags; };
struct Navi { float x, y; uint16_t tArea, tNode; int dx, dy; uint8_t left, right; };

void writeNode(std::vector<uint8_t>& b, const Node& n) {
    put32(b, 0);            // memory address, ignored
    put32(b, 0);            // always zero
    putPos(b, n.x); putPos(b, n.y); putPos(b, n.z);
    put16(b, 0x7FFE);       // heuristic, constant in the stock files
    put16(b, n.linkId);
    put16(b, 0);            // area id
    put16(b, 0);            // node id, overwritten below by the caller order
    b.push_back(n.width);
    b.push_back(n.type);
    put32(b, n.flags);
}

void writeNavi(std::vector<uint8_t>& b, const Navi& v) {
    putPos(b, v.x); putPos(b, v.y);
    put16(b, v.tArea); put16(b, v.tNode);
    b.push_back(static_cast<uint8_t>(static_cast<int8_t>(v.dx)));
    b.push_back(static_cast<uint8_t>(static_cast<int8_t>(v.dy)));
    put32(b, (static_cast<uint32_t>(v.left) << 8) | (static_cast<uint32_t>(v.right) << 11));
}

bool writeFile(const std::string& path, const std::vector<Node>& nodes, uint32_t vehCount,
               const std::vector<Navi>& navis,
               const std::vector<std::pair<uint16_t, uint16_t>>& links,
               const std::vector<uint16_t>& naviLinks,
               const std::vector<uint8_t>& linkLens) {
    std::vector<uint8_t> b;
    put32(b, static_cast<uint32_t>(nodes.size()));
    put32(b, vehCount);
    put32(b, static_cast<uint32_t>(nodes.size()) - vehCount);
    put32(b, static_cast<uint32_t>(navis.size()));
    put32(b, static_cast<uint32_t>(links.size()));
    for (size_t i = 0; i < nodes.size(); ++i) {
        const size_t at = b.size();
        writeNode(b, nodes[i]);
        // node id must equal the local index; patch it in place.
        b[at + 20] = static_cast<uint8_t>(i & 0xFF);
        b[at + 21] = static_cast<uint8_t>(i >> 8);
    }
    for (const Navi& v : navis) writeNavi(b, v);
    for (const auto& l : links) { put16(b, l.first); put16(b, l.second); }
    for (int i = 0; i < 192; ++i) { b.push_back(0xFF); b.push_back(0xFF); b.push_back(0); b.push_back(0); }
    for (const uint16_t v : naviLinks) put16(b, v);
    for (const uint8_t v : linkLens) b.push_back(v);
    b.insert(b.end(), links.size() + 384, 0); // trailer
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const bool ok = std::fwrite(b.data(), 1, b.size(), f) == b.size();
    std::fclose(f);
    return ok;
}

} // namespace syn

// A horizontal quad at height z spanning [x0,x1] x [y0,y1], wound so the
// normal points up.
static void addSlab(CollisionMesh& m, float x0, float x1, float y0, float y1, float z) {
    const uint32_t base = m.vertexCount();
    m.addVertex({x0, y0, z});
    m.addVertex({x1, y0, z});
    m.addVertex({x1, y1, z});
    m.addVertex({x0, y1, z});
    const uint32_t idx[6] = {base, base + 1, base + 2, base, base + 2, base + 3};
    for (const uint32_t i : idx) m.indices.push_back(i);
}


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


    // --- SA path files: NODES*.DAT loader --------------------------------
    {
        const std::string dir = "/tmp/wqs_sa_paths";
        ::mkdir(dir.c_str(), 0777);

        using syn::Node; using syn::Navi;
        // flags carry the link count in bits 0-3.
        const std::vector<Node> nodes = {
            {   0.f,   0.f, 0.f, 0, 0, 1, 1u | (1u << 13)},  // 0 highway, ->1
            {  80.f,   0.f, 0.f, 1, 0, 1, 2u},               // 1 ->0, ->2
            { 160.f,   0.f, 0.f, 3, 0, 1, 1u},               // 2 ->1
            {   0.f, -80.f, 0.f, 0, 0, 2, 0u | (1u << 7)},   // 3 boat, isolated
            {  80.f, -80.f, 0.f, 0, 0, 1, 0u | (1u << 8)},   // 4 emergency, isolated
            {   0.f,  80.f, 0.f, 4, 8, 0, 1u},               // 5 ped ->6
            {  80.f,  80.f, 0.f, 5, 8, 0, 1u},               // 6 ped ->5
        };
        // Segment 0-1 is two-way (1 lane each way); segment 1-2 has lanes only
        // in the navi direction, which points from node 2 back to node 1.
        const std::vector<Navi> navis = {
            { 40.f, 0.f, 0, 0, -100, 0, 1, 1},
            {120.f, 0.f, 0, 1, -100, 0, 0, 1},
        };
        const std::vector<std::pair<uint16_t, uint16_t>> links = {
            {0, 1}, {0, 0}, {0, 2}, {0, 1}, {0, 6}, {0, 5},
        };
        const std::vector<uint16_t> naviLinks = {0, 0, 1, 1, 0, 0};
        const std::vector<uint8_t> linkLens = {80, 80, 80, 80, 80, 80};
        CHECK(syn::writeFile(dir + "/NODES0.DAT", nodes, 5, navis, links, naviLinks, linkLens),
              "sa paths: write synthetic area 0");
        // Areas 1..62 exist but are empty; area 63 is absent on purpose, so
        // the loader's tolerance for a missing file is exercised too.
        bool emptyAreasOk = true;
        for (int i = 1; i < 63; ++i)
            emptyAreasOk &= syn::writeFile(dir + "/NODES" + std::to_string(i) + ".DAT",
                                           {}, 0, {}, {}, {}, {});
        CHECK(emptyAreasOk, "sa paths: write empty areas 1..62");
        std::remove((dir + "/NODES63.DAT").c_str());

        RoadNetwork veh, ped;
        SaPathsStats st;
        std::string perr;
        CHECK(LoadSaPaths(dir, veh, ped, st, perr), "sa paths: load");
        CHECK(st.areasLoaded == 63, "sa paths: absent area file tolerated");
        CHECK(st.unresolved == 0, "sa paths: every link entry resolved");
        CHECK(veh.nodeCount() == 5 && ped.nodeCount() == 2,
              "sa paths: vehicle and pedestrian nodes land in separate graphs");
        CHECK(std::fabs(veh.nodePos(1).x - 80.f) < 1e-4f &&
              std::fabs(veh.nodePos(3).y + 80.f) < 1e-4f,
              "sa paths: positions decode from eighths of a unit");
        CHECK((veh.nodeInfo(0).flags & SaFlags::kHighway) != 0,
              "sa paths: highway flag decoded");
        CHECK((veh.nodeInfo(3).flags & SaFlags::kBoat) != 0 && veh.nodeInfo(3).type == 2,
              "sa paths: boat node decoded");
        CHECK((veh.nodeInfo(4).flags & SaFlags::kEmergency) != 0,
              "sa paths: emergency flag decoded");
        CHECK(ped.nodeInfo(0).type == 0,
              "sa paths: pedestrian nodes carry no vehicle type");
        CHECK(veh.hasLaneData() && !ped.hasLaneData(),
              "sa paths: lane data on vehicles only");

        // One-way: the lanes, not the link table, decide the legal direction.
        const RouteProfile car = RouteProfile::Car();
        CHECK(!veh.findPath({0.f, 0.f, 0.f}, {160.f, 0.f, 0.f}, car).success,
              "sa paths: one-way segment refuses travel against its lanes");
        const RoadNetwork::RouteResult back =
            veh.findPath({160.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, car);
        CHECK(back.success && back.waypoints.size() == 3,
              "sa paths: one-way segment passes with its lanes");
        CHECK(back.lanes.size() == 2 && back.lanes[0] == 1 && back.lanes[1] == 1,
              "sa paths: lane count reported per leg");
        RouteProfile ignoreOneWay = car;
        ignoreOneWay.respectOneWay = false;
        CHECK(veh.findPath({0.f, 0.f, 0.f}, {160.f, 0.f, 0.f}, ignoreOneWay).success,
              "sa paths: one-way can be waived per query");

        // Node classes gate which network a query snaps to.
        CHECK(veh.nearestNode({0.f, -80.f, 0.f}, car) != 3,
              "sa paths: a car never snaps to the boat network");
        CHECK(veh.nearestNode({0.f, -80.f, 0.f}, RouteProfile::Boat()) == 3,
              "sa paths: a boat snaps to the boat network");
        CHECK(veh.nearestNode({80.f, -80.f, 0.f}, car) == 4,
              "sa paths: emergency nodes are usable by default");
        RouteProfile civilian = car;
        civilian.allowEmergency = false;
        CHECK(veh.nearestNode({80.f, -80.f, 0.f}, civilian) != 4,
              "sa paths: emergency nodes excluded on request");

        const RoadNetwork::RouteResult walk =
            ped.findPath({0.f, 80.f, 0.f}, {80.f, 80.f, 0.f}, RouteProfile::Ped());
        CHECK(walk.success && walk.waypoints.size() == 2,
              "sa paths: pedestrian graph routes on its own");
        // Vehicle graph under the car profile: {0,1,2} plus the lone emergency
        // node, with the boat node filtered out entirely.
        CHECK(veh.componentSizes(car).size() == 2,
              "sa paths: component sizes computed per profile");

        // A short refusal path: a truncated file must fail, not read garbage.
        std::FILE* trunc = std::fopen((dir + "/NODES5.DAT").c_str(), "wb");
        if (trunc) { std::fputs("nope", trunc); std::fclose(trunc); }
        RoadNetwork v2, p2;
        SaPathsStats st2;
        std::string e2;
        CHECK(LoadSaPaths(dir, v2, p2, st2, e2) && st2.areasLoaded == 62,
              "sa paths: a corrupt area is skipped, the rest still load");
    }


    // --- vehicle dimensions: fit checks -----------------------------------
    {
        using RoutePlanner::VehicleSpec;

        // Corner sharpness is pure geometry, so it can be checked exactly.
        const std::vector<Vec3> straight = {{0,0,0}, {10,0,0}, {20,0,0}};
        CHECK(RoutePlanner::CheckTurns(straight, {}).minRadius == 0.f,
              "vehicle: a straight run reports no corner");
        const std::vector<Vec3> corner = {{0,0,0}, {10,0,0}, {10,10,0}};
        // Circle through those three points has radius 10/sqrt(2).
        const RoutePlanner::TurnReport t0 = RoutePlanner::CheckTurns(corner, {});
        CHECK(std::fabs(t0.minRadius - 7.0711f) < 1e-3f,
              "vehicle: right-angle corner radius is exact");
        CHECK(t0.tight.empty(), "vehicle: no turn flagged without a turn radius");
        VehicleSpec bus;
        bus.turnRadius = 10.f;
        const RoutePlanner::TurnReport t1 = RoutePlanner::CheckTurns(corner, bus);
        CHECK(t1.tight.size() == 1 && t1.tight[0].index == 1,
              "vehicle: corner tighter than the vehicle is flagged");
        VehicleSpec bike;
        bike.turnRadius = 5.f;
        CHECK(RoutePlanner::CheckTurns(corner, bike).tight.empty(),
              "vehicle: corner the vehicle can take is not flagged");

        // A flat floor with a raised kerb along one side, and a low roof over
        // part of it: enough to separate the width and height checks.
        CollisionMesh fit;
        addSlab(fit, -10.f, 60.f, -10.f, 10.f, 0.f);   // floor
        addSlab(fit, -10.f, 60.f, 2.f, 10.f, 3.f);     // raised shelf, y >= 2
        addSlab(fit, 20.f, 30.f, -10.f, 1.9f, 1.2f);   // low roof over the lane
        CollisionWorld fitWorld;
        std::string ferr;
        CHECK(fitWorld.build(fit, ferr), "vehicle: fit test world builds");

        // Along y = 0 the floor is flat, so a narrow vehicle is fine; a wide
        // one straddles onto the 3-unit shelf and must be refused.
        VehicleSpec narrow; narrow.width = 1.0f;
        VehicleSpec wide;   wide.width = 5.0f;  // straddles onto the shelf at y = 2
        const RoutePlanner::OffroadLeg legNarrow =
            RoutePlanner::CheckOffroadLeg(&fitWorld, {0.f, 0.f, 0.f}, {50.f, 0.f, 0.f}, narrow);
        const RoutePlanner::OffroadLeg legWide =
            RoutePlanner::CheckOffroadLeg(&fitWorld, {0.f, 0.f, 0.f}, {50.f, 0.f, 0.f}, wide);
        CHECK(legNarrow.drivable, "vehicle: narrow vehicle fits the flat lane");
        CHECK(!legWide.drivable && std::string(legWide.reason) == "width",
              "vehicle: wide vehicle refused, reported as a width failure");

        // Overhead: the roof sits 1.2 above the floor between x 20 and 30.
        const std::vector<Vec3> lane = {{5,0,0}, {15,0,0}, {25,0,0}, {35,0,0}};
        VehicleSpec low;  low.height = 1.0f;
        VehicleSpec tall; tall.height = 2.5f;
        const RoutePlanner::ClearanceReport cLow =
            RoutePlanner::CheckClearance(&fitWorld, lane, low);
        const RoutePlanner::ClearanceReport cTall =
            RoutePlanner::CheckClearance(&fitWorld, lane, tall);
        CHECK(cLow.measured == 4 && cLow.hits.empty() &&
              std::fabs(cLow.minHeight - low.height) < 1e-4f,
              "vehicle: a low vehicle clears the roof, reported as a lower bound");
        CHECK(cTall.hits.size() == 1 && cTall.hits[0].index == 2 &&
              std::fabs(cTall.minHeight - 1.2f) < 0.05f,
              "vehicle: a tall vehicle is told which waypoint is too low");

        // The protocol only pays for the checks when a vehicle is described.
        const std::string noVeh = HandleQueryJson(
            R"({"type":"find_offroad_path","id":"v1","from":[0,0,0],"to":[1,0,0]})",
            &fitWorld, &pf);
        CHECK(noVeh.find("vehicle_check") == std::string::npos,
              "vehicle: no fit report unless a vehicle is given");
    }

    if (gFails) {
        std::fprintf(stderr, "\n%d FAILED\n", gFails);
        return 1;
    }
    std::fprintf(stderr, "\nall tests passed\n");
    return 0;
}
