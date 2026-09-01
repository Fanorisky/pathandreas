#include "pathfinder/pathfinder.h"
#include "navmesh_builder/navmesh_file.h"
#include "common/log.h"
#include "collision_world/collision_world.h"

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"

#include <cstring>
#include <vector>
#include <mutex>
#include <unordered_map>
#include <thread>

namespace wqs {
namespace {

constexpr int kMaxPolys = 16384;
constexpr int kMaxStraight = 16384;
constexpr int kMaxVisited = 32;
constexpr int kMaxNodes = 65535;

void toRc(const Vec3& g, float* r) { gtaToRecast(g, r); }
Vec3 toGta(const float* r) { return recastToGta(r); }

} // namespace

struct Pathfinder::QuerySlot {
    dtNavMeshQuery* q = nullptr;
    dtQueryFilter filter;
    const dtNavMesh* mesh = nullptr;

    ~QuerySlot() {
        if (q) dtFreeNavMeshQuery(q);
    }
};

Pathfinder::Pathfinder() = default;

Pathfinder::~Pathfinder() {
    if (owned_ && mesh_) dtFreeNavMesh(mesh_);
}

bool Pathfinder::loadFile(const std::string& path, std::string& err) {
    dtNavMesh* m = LoadNavMesh(path, err);
    if (!m) return false;
    return attach(m, true, err);
}

bool Pathfinder::attach(dtNavMesh* mesh, bool takeOwnership, std::string& err) {
    if (!mesh) {
        err = "null navmesh";
        return false;
    }
    if (owned_ && mesh_) dtFreeNavMesh(mesh_);
    mesh_ = mesh;
    owned_ = takeOwnership;
    return true;
}

bool Pathfinder::ready() const { return mesh_ != nullptr; }

Pathfinder::QuerySlot* Pathfinder::threadQuery() const {
    // One dtNavMeshQuery per thread. dtNavMesh is read-only and shared.
    thread_local QuerySlot slot;
    if (slot.mesh != mesh_ || !slot.q) {
        if (slot.q) {
            dtFreeNavMeshQuery(slot.q);
            slot.q = nullptr;
        }
        slot.q = dtAllocNavMeshQuery();
        if (!slot.q || dtStatusFailed(slot.q->init(mesh_, kMaxNodes))) {
            WQS_ERROR("dtNavMeshQuery::init failed");
            return nullptr;
        }
        slot.filter.setIncludeFlags(0xffff);
        slot.filter.setExcludeFlags(0);
        slot.mesh = mesh_;
    }
    return &slot;
}

PathResult Pathfinder::FindPath(const Vec3& startPos, const Vec3& endPos,
                                const CollisionWorld* world, float offroadCost) const {
    PathResult out;
    if (!mesh_) return out;
    QuerySlot* slot = threadQuery();
    if (!slot) return out;

    // Price the corridor for this query. The filter lives in the thread's slot
    // and is reused, so both areas are set every time rather than once at
    // init - otherwise a query that asked for a preference would leak it into
    // the next caller's query on the same thread.
    slot->filter.setAreaCost(NavArea::kSidewalk, 1.0f);
    slot->filter.setAreaCost(NavArea::kWalkable, offroadCost > 0.f ? offroadCost : 1.0f);

    // Snap start/end Z down to the collision ground before searching the navmesh.
    // Callers often stand a fraction above the surface (feet offset, spawn height),
    // and some areas sit a few units above walkable navmesh; findNearestPoly only
    // snaps within a fixed box, so feeding it the true ground Z gives it the best
    // chance and keeps the first waypoint on the surface instead of floating.
    Vec3 start = startPos, end = endPos;
    if (world) {
        float gz = 0.f;
        if (world->FindGroundZFrom(start.x, start.y, start.z + 2.f, gz) && gz <= start.z + 0.1f)
            start.z = gz;
        if (world->FindGroundZFrom(end.x, end.y, end.z + 2.f, gz) && gz <= end.z + 0.1f)
            end.z = gz;
    }

    float s[3], e[3];
    toRc(start, s);
    toRc(end, e);

    dtPolyRef startRef = 0, endRef = 0;
    float sn[3], en[3];

    // Tight box first - NPCs on the road snap precisely. If either end is still
    // unreachable (fragmented geometry like pillar-choked sidewalks), retry with a
    // wider box rather than failing outright.
    auto snapWith = [&](const float ext[3]) -> bool {
        startRef = 0; endRef = 0;
        slot->q->findNearestPoly(s, ext, &slot->filter, &startRef, sn);
        slot->q->findNearestPoly(e, ext, &slot->filter, &endRef, en);
        return startRef != 0 && endRef != 0;
    };
    const float extTight[3] = {pathExtents_.x, pathExtents_.z, pathExtents_.y};
    if (!snapWith(extTight)) {
        const float extWide[3] = {pathExtentsWide_.x, pathExtentsWide_.z, pathExtentsWide_.y};
        if (!snapWith(extWide)) return out;
    }

    dtPolyRef polys[kMaxPolys];
    int npolys = 0;
    dtStatus st = slot->q->findPath(startRef, endRef, sn, en, &slot->filter,
                                    polys, &npolys, kMaxPolys);
    if (dtStatusFailed(st) || npolys == 0) return out;
    // DT_PARTIAL_RESULT means the goal polygon was never reached (disconnected
    // navmesh or node budget exhausted): Detour returns the best path toward
    // it. Surface that to the caller - a consumer that walks the waypoints
    // blindly would stop mid-route with no idea the route is incomplete.
    out.partial = (st & DT_PARTIAL_RESULT) != 0;

    float straight[kMaxStraight * 3];
    unsigned char flags[kMaxStraight];
    dtPolyRef refs[kMaxStraight];
    int nstraight = 0;
    st = slot->q->findStraightPath(sn, en, polys, npolys,
                                   straight, flags, refs, &nstraight, kMaxStraight,
                                   DT_STRAIGHTPATH_ALL_CROSSINGS);
    if (dtStatusFailed(st) || nstraight == 0) return out;

    // Convert to GTA coords and keep the first point exactly on the snapped start
    // (the poly projection can sit slightly off; the ground-snap above is better).
    out.waypoints.reserve(static_cast<size_t>(nstraight) + 1);
    out.offMesh.reserve(static_cast<size_t>(nstraight) + 1);
    if (nstraight > 0) {
        Vec3 p0 = toGta(&straight[0]);
        if ((p0 - start).lengthSq() < 900.f) p0 = start;
        out.waypoints.push_back(p0);
        out.offMesh.push_back((flags[0] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? 1 : 0);
    }
    for (int i = 1; i < nstraight; ++i) {
        out.waypoints.push_back(toGta(&straight[i * 3]));
        // Detour sets the flag on the waypoint where the connection STARTS, so
        // it describes the step leaving that waypoint.
        out.offMesh.push_back((flags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? 1 : 0);
    }

    // Raycast each straight segment. Detour's straight path follows polygon edges,
    // which are walkable by construction, so walls should not be crossed - but
    // fragmented geometry can still route a segment through an obstacle. When a hit
    // is found, insert the hit point as a waypoint to force the NPC to stop there.
    if (world) {
        std::vector<Vec3> clean;
        std::vector<uint8_t> cleanFlags;
        clean.reserve(out.waypoints.size() + 2);
        cleanFlags.reserve(out.waypoints.size() + 2);
        for (size_t i = 0; i < out.waypoints.size(); ++i) {
            const Vec3& a = out.waypoints[i];
            const uint8_t f = i < out.offMesh.size() ? out.offMesh[i] : 0;
            clean.push_back(a);
            cleanFlags.push_back(f);
            if (i + 1 < out.waypoints.size()) {
                // An off-mesh step legitimately passes through geometry - the
                // riser it climbs over - so raycasting it would always "hit"
                // and insert a bogus stop point in the middle of the climb.
                if (f) continue;
                const Vec3& b = out.waypoints[i + 1];
                RayHitResult hit;
                if (world->RayCastLine(a, b, hit) && hit.hit && hit.fraction > 0.05f && hit.fraction < 0.95f) {
                    clean.push_back(hit.point);
                    cleanFlags.push_back(0);
                }
            }
        }
        out.waypoints.swap(clean);
        out.offMesh.swap(cleanFlags);
    }

    out.success = true;
    return out;
}

unsigned char Pathfinder::AreaAt(const Vec3& pos) const {
    if (!mesh_) return 0;
    QuerySlot* slot = threadQuery();
    if (!slot) return 0;
    // Neutral costs: this asks what is there, it does not route.
    slot->filter.setAreaCost(NavArea::kSidewalk, 1.0f);
    slot->filter.setAreaCost(NavArea::kWalkable, 1.0f);
    const float p[3] = {pos.x, pos.z, -pos.y};
    const float ext[3] = {surfaceExtents_.x, surfaceExtents_.z, surfaceExtents_.y};
    dtPolyRef ref = 0;
    float nearest[3];
    if (dtStatusFailed(slot->q->findNearestPoly(p, ext, &slot->filter, &ref, nearest)) || !ref)
        return 0;
    const dtMeshTile* tile = nullptr;
    const dtPoly* poly = nullptr;
    if (dtStatusFailed(mesh_->getTileAndPolyByRef(ref, &tile, &poly)) || !poly) return 0;
    return poly->getArea();
}

Vec3 Pathfinder::MoveAlongSurface(const Vec3& currentPos, const Vec3& desiredMove) const {
    if (!mesh_) return currentPos + desiredMove;
    QuerySlot* slot = threadQuery();
    if (!slot) return currentPos + desiredMove;

    float s[3], e[3];
    toRc(currentPos, s);
    toRc(currentPos + desiredMove, e);
    // Tight box: the actor is already on the navmesh, only needs a small snap.
    const float ext[3] = {surfaceExtents_.x, surfaceExtents_.z, surfaceExtents_.y};

    dtPolyRef startRef = 0;
    float sn[3];
    slot->q->findNearestPoly(s, ext, &slot->filter, &startRef, sn);
    if (!startRef) return currentPos;

    float result[3];
    dtPolyRef visited[kMaxVisited];
    int nvisited = 0;
    dtStatus st = slot->q->moveAlongSurface(startRef, sn, e, &slot->filter,
                                            result, visited, &nvisited, kMaxVisited);
    if (dtStatusFailed(st)) return currentPos;

    // Snap to surface height.
    dtPolyRef resultRef = nvisited ? visited[nvisited - 1] : startRef;
    float h = result[1];
    slot->q->getPolyHeight(resultRef, result, &h);
    result[1] = h;
    return toGta(result);
}

} // namespace wqs
