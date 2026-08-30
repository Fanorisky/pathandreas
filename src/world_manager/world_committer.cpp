#include "world_manager/world_committer.h"
#include "common/log.h"

#include <chrono>
#include <memory>
#include <utility>

namespace wqs {

WorldCommitter::WorldCommitter(WorldManager& manager, Backends& backends,
                               const NavBuildConfig& pedCfg,
                               const NavBuildConfig& vehicleCfg, bool bakeVehicle)
    : manager_(manager), backends_(backends), pedCfg_(pedCfg),
      vehicleCfg_(vehicleCfg), bakeVehicle_(bakeVehicle) {}

WorldCommitter::~WorldCommitter() { wait(); }

void WorldCommitter::wait() {
    if (worker_.joinable()) worker_.join();
}

long WorldCommitter::removeBuilding(uint16_t modelId, const Vec3& pos, float radius,
                                    std::string& err) {
    if (radius <= 0.f) {
        err = "radius must be positive";
        return -1;
    }
    const long matched = static_cast<long>(manager_.countMatching(modelId, pos, radius));
    manager_.removeBuilding(modelId, pos, radius);
    return matched;
}

bool WorldCommitter::addObject(uint16_t modelId, const Vec3& pos, const Quat& rot,
                               std::string& err) {
    return manager_.addObject(modelId, pos, rot, err);
}

void WorldCommitter::reset() { manager_.clearEdits(); }

bool WorldCommitter::beginCommit(std::string& err) {
    bool expected = false;
    if (!committing_.compare_exchange_strong(expected, true)) {
        err = "a commit is already running";
        return false;
    }
    // Join the previous worker before replacing it: the thread object is only
    // reused once its commit has finished (committing_ was false).
    if (worker_.joinable()) worker_.join();
    worker_ = std::thread([this] { runCommit(); });
    return true;
}

void WorldCommitter::runCommit() {
    const auto t0 = std::chrono::steady_clock::now();
    WQS_INFO("World commit: assembling mesh with %ld removals and %ld additions",
             removeCount(), addCount());

    // Everything up to the swap happens on locally-owned objects, so queries
    // keep running against the current world for the whole (long) rebuild.
    CollisionMesh mesh = manager_.assembleEdited();
    if (mesh.triangleCount() == 0) {
        WQS_ERROR("World commit aborted: edited world has no geometry");
        committing_.store(false);
        return;
    }

    auto world = std::make_unique<CollisionWorld>();
    std::string err;
    if (!world->build(mesh, err)) {
        WQS_ERROR("World commit aborted: collision build: %s", err.c_str());
        committing_.store(false);
        return;
    }

    std::unique_ptr<Pathfinder> ped, vehicle;
    NavBuildStats stats;
    if (dtNavMesh* nav = BuildTiledNavMesh(mesh, pedCfg_, stats, err)) {
        ped = std::make_unique<Pathfinder>();
        if (!ped->attach(nav, true, err)) {
            WQS_ERROR("World commit: pedestrian navmesh attach: %s", err.c_str());
            ped.reset();
        }
    } else {
        WQS_WARN("World commit: pedestrian navmesh bake failed (%s); keeping the old one",
                 err.c_str());
    }
    if (bakeVehicle_) {
        if (dtNavMesh* nav = BuildTiledNavMesh(mesh, vehicleCfg_, stats, err)) {
            vehicle = std::make_unique<Pathfinder>();
            if (!vehicle->attach(nav, true, err)) {
                WQS_ERROR("World commit: vehicle navmesh attach: %s", err.c_str());
                vehicle.reset();
            }
        } else {
            WQS_WARN("World commit: vehicle navmesh bake failed (%s); keeping the old one",
                     err.c_str());
        }
    }

    // Swap. Old objects are destroyed after the lock is released, once no
    // query can still hold a pointer to them. A backend whose rebuild failed
    // keeps its previous instance rather than going dark.
    std::unique_ptr<CollisionWorld> oldWorld;
    std::unique_ptr<Pathfinder> oldPed, oldVehicle;
    {
        std::unique_lock<std::shared_mutex> lk(backends_.mu);
        oldWorld = std::exchange(backends_.world, std::move(world));
        if (ped) oldPed = std::exchange(backends_.pathfinder, std::move(ped));
        if (vehicle) oldVehicle = std::exchange(backends_.vehiclePathfinder, std::move(vehicle));
    }
    oldWorld.reset();
    oldPed.reset();
    oldVehicle.reset();

    const double secs = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    WQS_INFO("World commit done in %.1fs (%u triangles)", secs, mesh.triangleCount());
    committing_.store(false);
}

} // namespace wqs
