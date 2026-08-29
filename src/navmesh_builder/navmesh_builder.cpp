#include "navmesh_builder/navmesh_builder.h"
#include "navmesh_builder/navmesh_file.h"
#include "common/log.h"
#include "common/thread_pool.h"

#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"

#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>

namespace wqs {
namespace {

struct RecastVerts {
    std::vector<float> verts; // recast Y-up
    std::vector<int> tris;
};

RecastVerts toRecast(const CollisionMesh& mesh) {
    RecastVerts r;
    r.verts.resize(mesh.vertices.size());
    for (size_t i = 0; i < mesh.vertexCount(); ++i) {
        r.verts[i * 3 + 0] = mesh.vertices[i * 3 + 0];  // x
        r.verts[i * 3 + 1] = mesh.vertices[i * 3 + 2];  // z-up -> y-up
        r.verts[i * 3 + 2] = -mesh.vertices[i * 3 + 1]; // y -> -z (preserve winding)
    }
    r.tris.resize(mesh.indices.size());
    for (size_t i = 0; i < mesh.indices.size(); ++i)
        r.tris[i] = static_cast<int>(mesh.indices[i]);
    return r;
}

void calcBounds(const RecastVerts& r, float bmin[3], float bmax[3]) {
    rcCalcBounds(r.verts.data(), static_cast<int>(r.verts.size() / 3), bmin, bmax);
}

struct TileResult {
    unsigned char* data = nullptr;
    int dataSize = 0;
};

TileResult buildTile(rcContext* ctx, const RecastVerts& rv, const NavBuildConfig& in,
                     const float orig[3], int tx, int ty, float tileWu) {
    TileResult result;
    rcConfig cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.cs = in.cs;
    cfg.ch = in.ch;
    cfg.walkableSlopeAngle = in.walkableSlopeAngle;
    cfg.walkableHeight = static_cast<int>(std::ceil(in.agentHeight / cfg.ch));
    cfg.walkableClimb = static_cast<int>(std::ceil(in.agentClimb / cfg.ch));
    cfg.walkableRadius = static_cast<int>(std::ceil(in.agentRadius / cfg.cs));
    cfg.maxEdgeLen = in.maxEdgeLen;
    cfg.maxSimplificationError = in.maxSimplificationError;
    cfg.minRegionArea = in.minRegionArea;
    cfg.mergeRegionArea = in.mergeRegionArea;
    cfg.maxVertsPerPoly = in.maxVertsPerPoly;
    cfg.detailSampleDist = in.detailSampleDist < 0.9f ? 0.f : in.detailSampleDist;
    cfg.detailSampleMaxError = in.detailSampleMaxError;
    cfg.tileSize = static_cast<int>(std::ceil(tileWu / cfg.cs));
    cfg.borderSize = cfg.walkableRadius + 3;
    cfg.width = cfg.tileSize + cfg.borderSize * 2;
    cfg.height = cfg.tileSize + cfg.borderSize * 2;

    cfg.bmin[0] = orig[0] + tx * tileWu;
    cfg.bmin[1] = orig[1];
    cfg.bmin[2] = orig[2] + ty * tileWu;
    cfg.bmax[0] = orig[0] + (tx + 1) * tileWu;
    cfg.bmax[1] = orig[1] + 0; // filled below
    cfg.bmax[2] = orig[2] + (ty + 1) * tileWu;

    float meshBmin[3], meshBmax[3];
    calcBounds(rv, meshBmin, meshBmax);
    cfg.bmin[1] = meshBmin[1];
    cfg.bmax[1] = meshBmax[1];

    // Expand by border.
    cfg.bmin[0] -= cfg.borderSize * cfg.cs;
    cfg.bmin[2] -= cfg.borderSize * cfg.cs;
    cfg.bmax[0] += cfg.borderSize * cfg.cs;
    cfg.bmax[2] += cfg.borderSize * cfg.cs;

    rcHeightfield* solid = rcAllocHeightfield();
    if (!solid || !rcCreateHeightfield(ctx, *solid, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch)) {
        rcFreeHeightField(solid);
        return result;
    }

    const int nverts = static_cast<int>(rv.verts.size() / 3);
    const int ntrisAll = static_cast<int>(rv.tris.size() / 3);

    // Keep triangles that overlap the expanded tile AABB (cheap reject).
    std::vector<int> tileTris;
    std::vector<unsigned char> dummyAreas;
    tileTris.reserve(static_cast<size_t>(ntrisAll / 8 + 16));
    for (int i = 0; i < ntrisAll; ++i) {
        const int ia = rv.tris[i * 3 + 0] * 3;
        const int ib = rv.tris[i * 3 + 1] * 3;
        const int ic = rv.tris[i * 3 + 2] * 3;
        float tmin[3] = {rv.verts[ia], rv.verts[ia + 1], rv.verts[ia + 2]};
        float tmax[3] = {tmin[0], tmin[1], tmin[2]};
        for (int k = 0; k < 3; ++k) {
            tmin[k] = std::min(tmin[k], std::min(rv.verts[ib + k], rv.verts[ic + k]));
            tmax[k] = std::max(tmax[k], std::max(rv.verts[ib + k], rv.verts[ic + k]));
        }
        if (tmax[0] < cfg.bmin[0] || tmin[0] > cfg.bmax[0]) continue;
        if (tmax[1] < cfg.bmin[1] || tmin[1] > cfg.bmax[1]) continue;
        if (tmax[2] < cfg.bmin[2] || tmin[2] > cfg.bmax[2]) continue;
        // CADB contains stray geometry the game never places on screen: whole
        // duplicate surface layers floating at z 800-1200 (~6.5k walkable polys
        // per full-map bake), and misplaced slabs far outside the +-3000 map
        // bounds. Nothing legitimate is above Mount Chiliad's ~500 or beyond
        // the map. Rasterized, they become phantom walkable islands that show
        // up as connected components and can even snap a query. Drop them.
        // The bounds check is per-vertex, not per-triangle-bbox, so enormous
        // triangles that merely stretch out of the map are caught too.
        // (rv is Recast Y-up: rv.y = GTA z, rv.z = -GTA y.)
        {
            bool stray = false;
            int belowSea = 0;
            for (int k = 0; k < 3 && !stray; ++k) {
                const float* v[3] = {&rv.verts[ia], &rv.verts[ib], &rv.verts[ic]};
                if (v[k][1] > 600.0f) stray = true;                       // above the world
                if (std::fabs(v[k][0]) > 3500.0f) stray = true;           // outside map x
                if (std::fabs(v[k][2]) > 3500.0f) stray = true;           // outside map y
                if (v[k][1] < -1.5f) ++belowSea;                          // rv.y = GTA z
            }
            // The sea floor is collision geometry too, and it is flat enough
            // to rasterize as perfectly walkable - without this filter the
            // navmesh spreads across the ocean and NPCs happily route (and
            // walk) underwater. Triangles fully below sea level (GTA water
            // plane is z=0; -1.5 keeps sloped shoreline) are the sea bed.
            if (stray || belowSea == 3) continue;
        }
        tileTris.push_back(rv.tris[i * 3 + 0]);
        tileTris.push_back(rv.tris[i * 3 + 1]);
        tileTris.push_back(rv.tris[i * 3 + 2]);
    }
    if (tileTris.empty()) {
        rcFreeHeightField(solid);
        return result;
    }
    const int ntris = static_cast<int>(tileTris.size() / 3);
    std::vector<unsigned char> triAreas(static_cast<size_t>(ntris));
    // GTA .col face windings are inconsistent (the game itself collides on both
    // sides), so roughly half of all flat surfaces come with downward-facing
    // normals, and rcMarkWalkableTriangles rejects them as unwalkable. That
    // silently deletes entire roads and pavements from the navmesh while the
    // correctly-wound duplicate surfaces buried under them also fail the
    // agent-height clearance filter. Mark walkability by ABSOLUTE slope
    // instead: the orientation of a face does not matter, only its steepness.
    {
        const float cosSlope = std::cos(cfg.walkableSlopeAngle * 3.14159265f / 180.0f);
        for (int i = 0; i < ntris; ++i) {
            const float* va = &rv.verts[static_cast<size_t>(tileTris[i*3+0]) * 3];
            const float* vb = &rv.verts[static_cast<size_t>(tileTris[i*3+1]) * 3];
            const float* vc = &rv.verts[static_cast<size_t>(tileTris[i*3+2]) * 3];
            const float ux = vb[0]-va[0], uy = vb[1]-va[1], uz = vb[2]-va[2];
            const float vx = vc[0]-va[0], vy = vc[1]-va[1], vz = vc[2]-va[2];
            const float nx = uy*vz - uz*vy, ny = uz*vx - ux*vz, nz = ux*vy - uy*vx;
            const float len = std::sqrt(nx*nx + ny*ny + nz*nz);
            triAreas[i] = (len > 1e-9f && std::fabs(ny) / len >= cosSlope)
                          ? RC_WALKABLE_AREA : RC_NULL_AREA;
        }
    }
    if (!rcRasterizeTriangles(ctx, rv.verts.data(), nverts, tileTris.data(),
                              triAreas.data(), ntris, *solid, cfg.walkableClimb)) {
        rcFreeHeightField(solid);
        return result;
    }

    rcFilterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, *solid);
    rcFilterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, *solid);
    rcFilterWalkableLowHeightSpans(ctx, cfg.walkableHeight, *solid);

    rcCompactHeightfield* chf = rcAllocCompactHeightfield();
    if (!chf || !rcBuildCompactHeightfield(ctx, cfg.walkableHeight, cfg.walkableClimb, *solid, *chf)) {
        rcFreeCompactHeightfield(chf);
        rcFreeHeightField(solid);
        return result;
    }
    rcFreeHeightField(solid);

    if (!rcErodeWalkableArea(ctx, cfg.walkableRadius, *chf)) {
        rcFreeCompactHeightfield(chf);
        return result;
    }
    if (!rcBuildDistanceField(ctx, *chf)) {
        rcFreeCompactHeightfield(chf);
        return result;
    }
    if (!rcBuildRegions(ctx, *chf, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea)) {
        rcFreeCompactHeightfield(chf);
        return result;
    }

    rcContourSet* cset = rcAllocContourSet();
    if (!cset || !rcBuildContours(ctx, *chf, cfg.maxSimplificationError, cfg.maxEdgeLen, *cset)) {
        rcFreeContourSet(cset);
        rcFreeCompactHeightfield(chf);
        return result;
    }

    rcPolyMesh* pmesh = rcAllocPolyMesh();
    if (!pmesh || !rcBuildPolyMesh(ctx, *cset, cfg.maxVertsPerPoly, *pmesh)) {
        rcFreePolyMesh(pmesh);
        rcFreeContourSet(cset);
        rcFreeCompactHeightfield(chf);
        return result;
    }

    rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
    if (!dmesh || !rcBuildPolyMeshDetail(ctx, *pmesh, *chf, cfg.detailSampleDist, cfg.detailSampleMaxError, *dmesh)) {
        rcFreePolyMeshDetail(dmesh);
        rcFreePolyMesh(pmesh);
        rcFreeContourSet(cset);
        rcFreeCompactHeightfield(chf);
        return result;
    }
    rcFreeCompactHeightfield(chf);
    rcFreeContourSet(cset);

    if (pmesh->nverts == 0 || pmesh->npolys == 0) {
        rcFreePolyMeshDetail(dmesh);
        rcFreePolyMesh(pmesh);
        return result;
    }

    for (int i = 0; i < pmesh->npolys; ++i) {
        pmesh->flags[i] = 1;
        if (pmesh->areas[i] == RC_WALKABLE_AREA) pmesh->areas[i] = 63;
    }

    dtNavMeshCreateParams params;
    std::memset(&params, 0, sizeof(params));
    params.verts = pmesh->verts;
    params.vertCount = pmesh->nverts;
    params.polys = pmesh->polys;
    params.polyAreas = pmesh->areas;
    params.polyFlags = pmesh->flags;
    params.polyCount = pmesh->npolys;
    params.nvp = pmesh->nvp;
    params.detailMeshes = dmesh->meshes;
    params.detailVerts = dmesh->verts;
    params.detailVertsCount = dmesh->nverts;
    params.detailTris = dmesh->tris;
    params.detailTriCount = dmesh->ntris;
    params.walkableHeight = in.agentHeight;
    params.walkableRadius = in.agentRadius;
    params.walkableClimb = in.agentClimb;
    params.cs = cfg.cs;
    params.ch = cfg.ch;
    params.buildBvTree = true;
    params.tileX = tx;
    params.tileY = ty;
    params.tileLayer = 0;
    rcVcopy(params.bmin, pmesh->bmin);
    rcVcopy(params.bmax, pmesh->bmax);

    unsigned char* data = nullptr;
    int dataSize = 0;
    if (!dtCreateNavMeshData(&params, &data, &dataSize)) {
        rcFreePolyMeshDetail(dmesh);
        rcFreePolyMesh(pmesh);
        return result;
    }
    rcFreePolyMeshDetail(dmesh);
    rcFreePolyMesh(pmesh);
    result.data = data;
    result.dataSize = dataSize;
    return result;
}

} // namespace

dtNavMesh* BuildTiledNavMesh(const CollisionMesh& mesh, const NavBuildConfig& cfg,
                             NavBuildStats& stats, std::string& err) {
    stats = {};
    if (mesh.triangleCount() == 0) {
        err = "empty mesh";
        return nullptr;
    }
    RecastVerts rv = toRecast(mesh);
    float bmin[3], bmax[3];
    calcBounds(rv, bmin, bmax);

    // Tiles must span an integer number of voxels. If tileWorldSize is not a
    // multiple of cs, each tile's polymesh overshoots the logical tile boundary
    // by the rounding remainder (ceil(160/0.3)=534 voxels = 160.2 world units),
    // so adjacent tiles' border portal planes sit that remainder apart and
    // Detour (0.01 match tolerance in findConnectingPolys) rejects every
    // inter-tile link. Snap the tile size up to the next whole voxel multiple
    // so tile N's +x portal plane lands exactly on tile N+1's -x portal plane.
    const float tw = std::ceil(cfg.tileWorldSize / cfg.cs) * cfg.cs;
    const int twCount = std::max(1, static_cast<int>(std::ceil((bmax[0] - bmin[0]) / tw)));
    const int thCount = std::max(1, static_cast<int>(std::ceil((bmax[2] - bmin[2]) / tw)));

    int tileBits = 0;
    int tmp = twCount * thCount;
    while ((1 << tileBits) < tmp) ++tileBits;
    if (tileBits < 1) tileBits = 1;
    if (tileBits > 14) tileBits = 14;
    const int polyBits = 22 - tileBits;

    dtNavMeshParams params;
    std::memset(&params, 0, sizeof(params));
    rcVcopy(params.orig, bmin);
    params.tileWidth = tw;
    params.tileHeight = tw;
    params.maxTiles = 1 << tileBits;
    params.maxPolys = 1 << polyBits;

    dtNavMesh* nav = dtAllocNavMesh();
    if (!nav || dtStatusFailed(nav->init(&params))) {
        err = "dtNavMesh init failed";
        dtFreeNavMesh(nav);
        return nullptr;
    }

    rcContext ctx(false);
    const int totalTiles = twCount * thCount;
    WQS_INFO("Navmesh tiles %dx%d = %d (wu=%.1f, cs=%.2f) maxTiles=%d",
             twCount, thCount, totalTiles, tw, cfg.cs, params.maxTiles);

    // Tiles are independent of each other, so they can bake in parallel. Recast
    // documents no thread-safety for rcContext (and its logging path is shared
    // state), so each worker makes its own quiet context; dtNavMesh::addTile is
    // likewise not thread-safe, so tiles are collected first and added serially
    // on this thread afterwards.
    const unsigned workerCount = cfg.threads != 0
        ? cfg.threads
        : std::max(2u, std::thread::hardware_concurrency());
    const unsigned actualWorkers = std::min<unsigned>(workerCount, static_cast<unsigned>(totalTiles));
    WQS_INFO("Baking with %u thread(s)...", actualWorkers);

    struct BuiltTile {
        int tx = 0, ty = 0;
        TileResult tr;
    };
    std::vector<BuiltTile> built;
    built.reserve(static_cast<size_t>(totalTiles));
    std::mutex builtMu;
    std::atomic<int> done{0};
    std::atomic<int> empty{0};
    std::atomic<int> attempted{0};

    // Progress is reported on a percentage schedule. One line per tile would flood
    // the log on the full map (~2200 tiles at wu=128); one per milestone keeps it
    // readable while still showing steady movement, and each line carries the
    // current rate so a stalled bake is visible as a collapsing tiles/s figure.
    const auto t0 = std::chrono::steady_clock::now();
    auto report = [&](int doneNow, bool finalLine) {
        const int pct = totalTiles > 0 ? (doneNow * 100) / totalTiles : 100;
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const double rate = secs > 0.0 ? doneNow / secs : 0.0;
        if (finalLine) {
            WQS_INFO("Bake done: %d/%d tiles in %.1fs (%.1f tiles/s, %d empty)",
                     doneNow, totalTiles, secs, rate, empty.load());
        } else {
            WQS_INFO("Baking... %d/%d tiles (%d%%, %.1f tiles/s, %d empty)",
                     doneNow, totalTiles, pct, rate, empty.load());
        }
    };
    // Emit one line per 25% milestone, in order, exactly once. Under concurrency
    // several workers can cross the same milestone back-to-back, so the check and
    // the advance must be one atomic operation - a plain read-then-increment would
    // let two threads both see "25 not yet printed" and print it twice. When one
    // tile jumps the cursor over several milestones at once (tiny maps, the last
    // tile), only the highest is printed - four identical 100% lines say nothing
    // the last one doesn't.
    std::atomic<int> nextMilestone{25};
    auto tick = [&](int d) {
        int m = nextMilestone.load(std::memory_order_relaxed);
        while (m <= 100 && d * 100 >= m * totalTiles) {
            // Advance past every milestone this tile covers.
            int covered = m;
            while (covered <= 100 && d * 100 >= covered * totalTiles) covered += 25;
            if (nextMilestone.compare_exchange_weak(m, covered,
                                                    std::memory_order_relaxed)) {
                report(d, false);
                m = covered;
            }
            // On CAS failure `m` was reloaded with the current cursor; loop again.
        }
    };

    if (actualWorkers <= 1) {
        for (int ty = 0; ty < thCount; ++ty) {
            for (int tx = 0; tx < twCount; ++tx) {
                ++attempted;
                TileResult tr = buildTile(&ctx, rv, cfg, bmin, tx, ty, tw);
                const int d = ++done;
                if (!tr.data) {
                    ++empty;
                } else {
                    built.push_back({tx, ty, tr});
                }
                tick(d);
            }
        }
    } else {
        ThreadPool pool(actualWorkers);
        std::vector<std::future<void>> pending;
        pending.reserve(static_cast<size_t>(totalTiles));

        for (int ty = 0; ty < thCount; ++ty) {
            for (int tx = 0; tx < twCount; ++tx) {
                pending.push_back(pool.submit([&, tx, ty] {
                    rcContext tileCtx(false); // per-worker: see note above
                    TileResult tr = buildTile(&tileCtx, rv, cfg, bmin, tx, ty, tw);
                    ++attempted;
                    const int d = ++done;
                    if (!tr.data) {
                        ++empty;
                    } else {
                        std::lock_guard<std::mutex> lk(builtMu);
                        built.push_back({tx, ty, tr});
                    }
                    tick(d);
                }));
            }
        }
        for (auto& f : pending) f.wait();
    }

    report(done.load(), true);
    stats.tilesAttempted = attempted.load();
    stats.tilesEmpty = empty.load();

    // addTile assigns slots in call order, and SaveNavMesh walks slots - so the file
    // layout follows whatever order tiles finished baking in. Workers complete out of
    // order, which would make two bakes of the same input produce different bytes.
    // Sort back to row-major so the parallel bake is byte-identical to the serial one.
    std::sort(built.begin(), built.end(), [](const BuiltTile& a, const BuiltTile& b) {
        if (a.ty != b.ty) return a.ty < b.ty;
        return a.tx < b.tx;
    });

    for (const auto& bt : built) {
        dtStatus st = nav->addTile(bt.tr.data, bt.tr.dataSize, DT_TILE_FREE_DATA, 0, nullptr);
        if (dtStatusFailed(st)) {
            dtFree(bt.tr.data);
            // The status code names which addTile precondition tripped - without it
            // a single silent failure gives no clue whether it is occupancy (two
            // tiles claiming x,y), poly-count overflow (tile denser than maxPolys),
            // or plain allocator exhaustion.
            const char* why = "unknown";
            if (st & DT_ALREADY_OCCUPIED) why = "already occupied";
            else if (st & DT_OUT_OF_MEMORY) why = "out of memory (maxTiles?)";
            else if (st & DT_INVALID_PARAM) why = "invalid param (polyCount > maxPolys?)";
            else if (st & DT_WRONG_MAGIC) why = "wrong magic";
            else if (st & DT_WRONG_VERSION) why = "wrong version";
            WQS_WARN("addTile failed %d,%d: %s (status 0x%x)", bt.tx, bt.ty, why, st);
            continue;
        }
        ++stats.tilesBuilt;
    }

    WQS_INFO("Navmesh built: %d/%d tiles (empty %d)",
             stats.tilesBuilt, stats.tilesAttempted, stats.tilesEmpty);
    if (stats.tilesBuilt == 0) {
        err = "no walkable tiles produced";
        dtFreeNavMesh(nav);
        return nullptr;
    }
    return nav;
}

bool BuildNavMeshFile(const CollisionMesh& mesh, const std::string& outPath,
                      const NavBuildConfig& cfg, std::string& err) {
    NavBuildStats stats;
    dtNavMesh* nav = BuildTiledNavMesh(mesh, cfg, stats, err);
    if (!nav) return false;
    const bool ok = SaveNavMesh(outPath, nav, err);
    dtFreeNavMesh(nav);
    return ok;
}

} // namespace wqs
