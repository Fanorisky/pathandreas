#pragma once

#include "navmesh_builder/navmesh_builder.h"
#include "query_server/world_editor.h"
#include "world_manager/world_manager.h"

#include <atomic>
#include <string>
#include <thread>

namespace wqs {

// Applies recorded world edits to the live service: re-assembles the world
// mesh, rebuilds the collision world and navmesh(es) off to the side, then
// swaps them into `backends` under an exclusive lock.
//
// The bake is the expensive part (minutes on the full map) and runs with no
// lock held, so queries keep answering from the old world the whole time;
// only the pointer swap is exclusive. That makes a commit safe to trigger on
// a live server, at the cost of the new world appearing a few minutes after
// the request.
class WorldCommitter : public WorldEditor {
public:
    WorldCommitter(WorldManager& manager, Backends& backends,
                   const NavBuildConfig& pedCfg, const NavBuildConfig& vehicleCfg,
                   bool bakeVehicle);
    ~WorldCommitter() override;

    long removeBuilding(uint16_t modelId, const Vec3& pos, float radius,
                        std::string& err) override;
    bool addObject(uint16_t modelId, const Vec3& pos, const Quat& rot,
                   std::string& err) override;
    void reset() override;
    bool beginCommit(std::string& err) override;
    bool committing() const override { return committing_.load(); }
    long removeCount() const override { return static_cast<long>(manager_.removeEditCount()); }
    long addCount() const override { return static_cast<long>(manager_.addEditCount()); }

    // Blocks until any running commit finishes (tests, shutdown).
    void wait();

private:
    void runCommit();

    WorldManager& manager_;
    Backends& backends_;
    NavBuildConfig pedCfg_;
    NavBuildConfig vehicleCfg_;
    bool bakeVehicle_;
    std::atomic<bool> committing_{false};
    std::thread worker_;
};

} // namespace wqs
