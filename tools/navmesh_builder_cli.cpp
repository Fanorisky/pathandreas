#include "collision_loader/collision_loader.h"
#include "navmesh_builder/navmesh_builder.h"
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
        "                  [--radius 0.6] [--slope 45]\n"
        "                  [--region X1,Y1,X2,Y2]  bake only this AABB (GTA coords)\n");
}

int main(int argc, char** argv) {
    std::string cadb, col, out, region;
    bool testCity = false;
    NavBuildConfig cfg;
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
        } else if (!std::strcmp(argv[i], "--slope")) {
            std::string v; if (!next(v)) return 2;
            cfg.walkableSlopeAngle = std::strtof(v.c_str(), nullptr);
        } else {
            usage();
            return 2;
        }
    }
    if (out.empty()) { usage(); return 2; }

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
