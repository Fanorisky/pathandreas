#pragma once

#include "common/vec3.h"
#include <string>

class dtNavMesh;

namespace wqs {

struct NavBuildConfig {
    // Recast voxel size. GTA SA metres.
    float cs = 0.3f;
    float ch = 0.2f;
    float walkableSlopeAngle = 45.0f;
    float agentHeight = 2.0f;
    float agentClimb = 0.9f;   // GTA SA stair riser
    float agentRadius = 0.6f;
    int maxEdgeLen = 12;
    float maxSimplificationError = 1.3f;
    int minRegionArea = 8;
    int mergeRegionArea = 20;
    int maxVertsPerPoly = 6;
    float detailSampleDist = 6.0f;
    float detailSampleMaxError = 1.0f;
    // Tile size in world units (GTA XY). 128 keeps the voxel grid reasonable
    // at cs=0.3 (≈427 vx). 500 wu would be ~1667 vx/tile and is memory-heavy.
    float tileWorldSize = 128.0f;
    // 0 = auto (hardware_concurrency, min 2). Tiles are independent, so they
    // build in parallel on this many worker threads.
    unsigned threads = 0;

    // Bridge navmesh fragments that a small vertical step separates, with
    // Detour off-mesh connections. GTA staircases and platforms often have
    // risers taller than agentClimb (measured 1.44u on the LV block against a
    // 0.9u climb), so Recast walls each step off and floors bake disconnected.
    // A step link reconnects them the way a character actually climbs. Only
    // gaps between DIFFERENT connected components are bridged, so this targets
    // genuine fragmentation, not every curb.
    bool stepLinks = false;
    float stepLinkMaxRise = 2.0f;   // largest vertical gap treated as a step
    float stepLinkReach = 1.6f;     // how far horizontally to look across a gap
    float stepLinkRadius = 0.6f;    // Detour connection radius (agent-sized)
};

struct NavBuildStats {
    int tilesAttempted = 0;
    int tilesBuilt = 0;
    int tilesEmpty = 0;
    int totalPolys = 0;
    int offMeshLinks = 0;
};

dtNavMesh* BuildTiledNavMesh(const CollisionMesh& mesh, const NavBuildConfig& cfg,
                             NavBuildStats& stats, std::string& err);

bool BuildNavMeshFile(const CollisionMesh& mesh, const std::string& outPath,
                      const NavBuildConfig& cfg, std::string& err);

} // namespace wqs
