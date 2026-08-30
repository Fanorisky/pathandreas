#pragma once

#include "collision_loader/collision_loader.h"
#include "common/vec3.h"
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace wqs {

// Runtime world edits, mirroring the SA-MP/open.mp map-edit calls a script
// makes. Edits are recorded as they arrive and applied together when the
// world is re-assembled (see assembleEdited); nothing is modified in place.
//
// The CADB doubles as the model library for additions: a CreateObject call
// with a stock model id finds that model's collision geometry in the same
// database, so added objects need no extra data.
class WorldManager {
public:
    // Mirrors RemoveBuildingForPlayer: every placement of `modelId` whose
    // position lies within `radius` of `pos` is excluded from the world.
    struct RemoveEdit {
        uint16_t modelId = 0;
        Vec3 pos{};
        float radius = 0.f;
    };
    // Mirrors CreateObject with a stock model id.
    struct AddEdit {
        uint16_t modelId = 0;
        Vec3 pos{};
        Quat rotation{};
    };

    bool loadCadb(const std::string& path, std::string& err);
    const CadbDatabase& database() const { return db_; }
    void setLoaderOptions(const LoaderOptions& opt) { opt_ = opt; }

    // --- edit recording -------------------------------------------------
    // Number of stock placements the removal would exclude (for previews).
    size_t countMatching(uint16_t modelId, const Vec3& pos, float radius) const;
    size_t placementCount() const { return db_.placements.size(); }
    void removeBuilding(uint16_t modelId, const Vec3& pos, float radius);
    // Fails when the model id is not in the database (the object would have
    // no collision geometry).
    bool addObject(uint16_t modelId, const Vec3& pos, const Quat& rot, std::string& err);
    void clearEdits();
    size_t removeEditCount() const { return removes_.size(); }
    size_t addEditCount() const { return adds_.size(); }

    // --- re-assembly ----------------------------------------------------
    // Assemble the world mesh with all recorded edits applied: stock
    // placements matched by a removal are skipped, added placements are
    // appended (their models come from the same database).
    CollisionMesh assembleEdited() const;

private:
    // Caller must hold editsMu_.
    CadbDatabase editedDatabase() const;

    CadbDatabase db_;              // immutable after loadCadb
    LoaderOptions opt_;
    // Edits are mutated by protocol requests and snapshotted by commits that
    // can run concurrently on another thread; assembleEdited copies the
    // edited database under the lock and does the (long) assembly outside it.
    mutable std::mutex editsMu_;
    std::vector<RemoveEdit> removes_;
    std::vector<AddEdit> adds_;
};

// SA-MP CreateObject takes Euler angles in degrees. The GTA convention for
// object rotation is a Z, then X, then Y extrinsic application (equivalently
// Rz * Rx * Ry as a composite matrix) - the order the game itself converts
// with. If in-game testing ever shows a mismatch for rotated objects, this
// is the single place to change.
Quat EulerDegreesToQuat(const Vec3& eulerDegrees);

} // namespace wqs
