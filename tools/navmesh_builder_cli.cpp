#include "collision_loader/collision_loader.h"
#include "navmesh_builder/navmesh_builder.h"
#include "road_network/road_network.h"
#include "road_network/sa_paths.h"
#include "common/log.h"

#include <cstring>
#include <string>
#include <cstdio>
#include <cstdlib>

using namespace wqs;

static void usage() {
    std::fprintf(stderr,
        "navmesh_builder — offline Recast tiled navmesh bake\n"
        "Usage:\n"
        "  navmesh_builder --out FILE.navmesh [--cadb FILE | --col FILE | --test-city]\n"
        "                  [--tile-size 128] [--cs 0.3] [--ch 0.2] [--threads N]\n"
        "                  [--radius 0.6] [--slope 45] [--agent-height 2.0] [--agent-climb 0.9]\n"
        "                  [--step-links] [--step-rise 2.0]\n"
        "                  [--paths DIR] [--sidewalk-radius 7.0]  mark the ped corridor\n"
        "                  [--region X1,Y1,X2,Y2]  bake only this AABB (GTA coords)\n");
}

int main(int argc, char** argv) {
    std::string cadb, col, out, region;
    bool testCity = false;
    NavBuildConfig cfg;
    std::string pathsDir;
    std::vector<Vec3> sidewalkNodes;
    LoaderOptions loadOpt;

    for (int i = 1; i < argc; ++i) {
        auto next = [&](std::string& d) {
            if (i + 1 >= argc) return false;
            d = argv[++i];
            return true;
        };
        if (!std::strcmp(argv[i], "--help")) { usage(); return 0; }
        else if (!std::strcmp(argv[i], "--cadb")) { if (!next(cadb)) return 2; }
        else if (!std::strcmp(argv[i], "--col")) { if (!next(col)) return 2; }
        else if (!std::strcmp(argv[i], "--test-city")) { testCity = true; }
        else if (!std::strcmp(argv[i], "--out")) { if (!next(out)) return 2; }
        else if (!std::strcmp(argv[i], "--region")) {
            std::string v; if (!next(v)) return 2;
            // "X1,Y1,X2,Y2" in GTA coords; z is unbounded.
            float x1, y1, x2, y2;
            if (std::sscanf(v.c_str(), "%f,%f,%f,%f", &x1, &y1, &x2, &y2) != 4) {
                WQS_ERROR("--region expects X1,Y1,X2,Y2");
                return 2;
            }
            loadOpt.clipRegion = true;
            loadOpt.region.min = Vec3{std::min(x1, x2), std::min(y1, y2), -1000.f};
            loadOpt.region.max = Vec3{std::max(x1, x2), std::max(y1, y2), 1000.f};
        } else if (!std::strcmp(argv[i], "--tile-size")) {
            std::string v; if (!next(v)) return 2;
            cfg.tileWorldSize = std::strtof(v.c_str(), nullptr);
        } else if (!std::strcmp(argv[i], "--cs")) {
            std::string v; if (!next(v)) return 2;
            cfg.cs = std::strtof(v.c_str(), nullptr);
        } else if (!std::strcmp(argv[i], "--ch")) {
            std::string v; if (!next(v)) return 2;
            cfg.ch = std::strtof(v.c_str(), nullptr);
        } else if (!std::strcmp(argv[i], "--threads")) {
            std::string v; if (!next(v)) return 2;
            cfg.threads = static_cast<unsigned>(std::stoi(v));
        } else if (!std::strcmp(argv[i], "--radius")) {
            std::string v; if (!next(v)) return 2;
            cfg.agentRadius = std::strtof(v.c_str(), nullptr);
        } else if (!std::strcmp(argv[i], "--agent-height")) {
            std::string v; if (!next(v)) return 2;
            cfg.agentHeight = std::strtof(v.c_str(), nullptr);
        } else if (!std::strcmp(argv[i], "--agent-climb")) {
            std::string v; if (!next(v)) return 2;
            cfg.agentClimb = std::strtof(v.c_str(), nullptr);
        } else if (!std::strcmp(argv[i], "--slope")) {
            std::string v; if (!next(v)) return 2;
            cfg.walkableSlopeAngle = std::strtof(v.c_str(), nullptr);
        } else if (!std::strcmp(argv[i], "--paths")) {
            if (!next(pathsDir)) return 2;
        } else if (!std::strcmp(argv[i], "--sidewalk-radius")) {
            std::string v; if (!next(v)) return 2;
            cfg.sidewalkRadius = std::strtof(v.c_str(), nullptr);
        } else if (!std::strcmp(argv[i], "--step-links")) {
            cfg.stepLinks = true;
        } else if (!std::strcmp(argv[i], "--step-rise")) {
            std::string v; if (!next(v)) return 2;
            cfg.stepLinkMaxRise = std::strtof(v.c_str(), nullptr);
        } else {
            usage();
            return 2;
        }
    }
    if (out.empty()) { usage(); return 2; }

    // The pedestrian corridor is marked from the game's own ped path nodes, so
    // the bake needs them. Default the radius when --paths is given without one:
    // 7 sits just above the measured continuity floor (see NavBuildConfig).
    if (!pathsDir.empty()) {
        RoadNetwork veh, ped;
        SaPathsStats st;
        std::string perr;
        if (!LoadSaPaths(pathsDir, veh, ped, st, perr)) {
            WQS_ERROR("SA paths: %s", perr.c_str());
            return 1;
        }
        sidewalkNodes.reserve(static_cast<size_t>(ped.nodeCount()));
        for (long i = 0; i < ped.nodeCount(); ++i) sidewalkNodes.push_back(ped.nodePos(i));
        cfg.sidewalkNodes = &sidewalkNodes;
        if (cfg.sidewalkRadius <= 0.f) cfg.sidewalkRadius = 7.0f;
        WQS_INFO("Pedestrian corridor from %zu ped nodes, radius %.1f",
                 sidewalkNodes.size(), cfg.sidewalkRadius);
    } else if (cfg.sidewalkRadius > 0.f) {
        WQS_ERROR("--sidewalk-radius needs --paths to know where the nodes are");
        return 2;
    }

    CollisionMesh mesh;
    if (testCity) mesh = MakeTestCityMesh();
    else if (!cadb.empty()) mesh = LoadFromCadb(cadb, loadOpt);
    else if (!col.empty()) mesh = LoadFromCol(col, loadOpt);
    else { usage(); return 2; }

    if (mesh.triangleCount() == 0) {
        WQS_ERROR("empty mesh");
        return 1;
    }
    std::string err;
    if (!BuildNavMeshFile(mesh, out, cfg, err)) {
        WQS_ERROR("%s", err.c_str());
        return 1;
    }
    WQS_INFO("Wrote %s", out.c_str());
    return 0;
}
