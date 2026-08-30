#pragma once

#include "collision_world/collision_world.h"
#include "common/vec3.h"
#include "pathfinder/pathfinder.h"
#include "road_network/road_network.h"

#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

namespace wqs {

// Backends shared by every query thread. A world commit swaps them under `mu`;
// each request holds a shared lock for its duration, so an in-flight query
// always finishes on the world it started with, and the swap waits for the
// last of them before replacing (and destroying) the old objects.
struct Backends {
    std::shared_mutex mu;
    std::unique_ptr<CollisionWorld> world;
    std::unique_ptr<Pathfinder> pathfinder;         // pedestrian navmesh
    std::unique_ptr<Pathfinder> vehiclePathfinder;  // car-agent navmesh (optional)
    RoadNetwork* roads = nullptr;                   // not owned; edits never affect it
};

// Protocol-facing surface for world editing (RemoveBuilding / CreateObject
// awareness). Implemented by the server; the JSON layer only sees this.
struct WorldEditor {
    virtual ~WorldEditor() = default;

    // Mirrors RemoveBuildingForPlayer. Returns how many stock placements the
    // removal matched, or -1 with err set.
    virtual long removeBuilding(uint16_t modelId, const Vec3& pos, float radius,
                                std::string& err) = 0;
    // Mirrors CreateObject with a stock model id; fails when the model has no
    // collision geometry in the database.
    virtual bool addObject(uint16_t modelId, const Vec3& pos, const Quat& rot,
                           std::string& err) = 0;
    virtual void reset() = 0;
    // Starts an asynchronous re-assemble + re-bake + swap of everything in
    // `backends`. Returns false (with err) if a commit is already running.
    // Completion is observed via committing(); on the full map this takes
    // several minutes, which is why it is not synchronous - an HTTP client
    // would time out and a pool thread would be blocked for the duration.
    virtual bool beginCommit(std::string& err) = 0;
    virtual bool committing() const = 0;
    virtual long removeCount() const = 0;
    virtual long addCount() const = 0;
};

} // namespace wqs
