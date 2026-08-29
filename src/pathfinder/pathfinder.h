#pragma once

#include "common/vec3.h"
#include <memory>
#include <string>

class dtNavMesh;

namespace wqs {

class CollisionWorld;

class Pathfinder {
public:
    Pathfinder();
    ~Pathfinder();

    Pathfinder(const Pathfinder&) = delete;
    Pathfinder& operator=(const Pathfinder&) = delete;

    bool loadFile(const std::string& path, std::string& err);
    bool attach(dtNavMesh* mesh, bool takeOwnership, std::string& err);
    bool ready() const;

    // GTA SA coordinates (Z-up). Internally converted to Recast Y-up.
    PathResult FindPath(const Vec3& startPos, const Vec3& endPos, const CollisionWorld* world = nullptr) const;
    Vec3 MoveAlongSurface(const Vec3& currentPos, const Vec3& desiredMove) const;

    // Search box half-extents in GTA space (x,y,z). z is vertical.
    void setExtents(const Vec3& halfExtents) { pathExtents_ = halfExtents; }

private:
    struct QuerySlot;
    QuerySlot* threadQuery() const;

    dtNavMesh* mesh_ = nullptr;
    bool owned_ = false;
    // Half-extents for find_path snapping. NPC positions and destinations usually sit
    // near, but not exactly on, walkable navmesh (sidewalk edge, doorway, parking lot),
    // so the start/end must snap to the nearest polygon within a generous box. 16/16/32
    // covers those without jumping across an unrelated street - the first waypoint may
    // still sit a few metres off, which is fine for route planning.
    Vec3 pathExtents_{16.f, 16.f, 32.f};
    // Fallback box for find_path when the tight one finds nothing - fragmented geometry
    // (pillar-choked sidewalks, etc.) can leave a valid ground surface without any
    // walkable polygon within 16 units, but with one a bit further out. The wide box
    // only kicks in after the tight one fails, so on-road NPCs keep the precise snap.
    Vec3 pathExtentsWide_{48.f, 48.f, 96.f};
    // Half-extents for move_along_surface. This is called every movement tick with the
    // actor already on the navmesh, so only a tight box is needed to keep the position
    // glued to the surface. A large box here would snap the actor sideways across
    // obstacles mid-step, producing visible lurching.
    Vec3 surfaceExtents_{2.f, 2.f, 4.f};
};

} // namespace wqs
