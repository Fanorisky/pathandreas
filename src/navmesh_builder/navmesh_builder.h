#pragma once

#include "common/nav_area.h"
#include "common/vec3.h"
#include <string>
#include <vector>

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

    // Mark walkable ground within this radius of a pedestrian path node as
    // NavArea::kSidewalk, so a query can make it cheaper than everything else
    // and routes follow the network the game's own authors laid out instead of
    // cutting the geometrically shortest line across roads and plazas.
    //
    // The radius has a floor and a ceiling that were measured, not guessed.
    // Pedestrian nodes sit a median 6.4 units apart (p90 13.2), and a disc per
    // node only forms a CONTINUOUS corridor when the radius is at least half
    // the spacing - so below ~6.6 the corridor is a dotted line. Going wider
    // marks more and more of the map until the distinction stops meaning
    // anything: inside Los Santos, radius 5 marks 12.7% of walkable polygons,
    // 7 marks ~22%, 10 marks 34%. 7 sits just above the continuity floor while
    // still leaving roads, plazas and interiors outside the corridor.
    float sidewalkRadius = 0.f;             // 0 disables the marking entirely
    float sidewalkHeight = 3.0f;            // vertical extent of each mark
    // Node positions in GTA coords, not owned. Supplied by the caller because
    // the builder has no business knowing how path files are parsed.
    const std::vector<Vec3>* sidewalkNodes = nullptr;
};

struct NavBuildStats {
    int tilesAttempted = 0;
    int tilesBuilt = 0;
    int tilesEmpty = 0;
    int totalPolys = 0;
    int offMeshLinks = 0;
    int sidewalkPolys = 0;   // polygons carrying NavArea::kSidewalk
};

dtNavMesh* BuildTiledNavMesh(const CollisionMesh& mesh, const NavBuildConfig& cfg,
                             NavBuildStats& stats, std::string& err);

bool BuildNavMeshFile(const CollisionMesh& mesh, const std::string& outPath,
                      const NavBuildConfig& cfg, std::string& err);

} // namespace wqs
