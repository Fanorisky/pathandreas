#include "world_manager/world_manager.h"
#include "common/log.h"

#include <algorithm>
#include <cmath>

namespace wqs {

Quat EulerDegreesToQuat(const Vec3& e) {
    // Composite Rz * Rx * Ry, angles in radians, quaternion per axis then
    // multiplied out. See the header note before changing the order.
    const float pi = 3.14159265f;
    const float hx = e.x * pi / 180.f * 0.5f;
    const float hy = e.y * pi / 180.f * 0.5f;
    const float hz = e.z * pi / 180.f * 0.5f;
    const Quat qx{std::sin(hx), 0, 0, std::cos(hx)};
    const Quat qy{0, std::sin(hy), 0, std::cos(hy)};
    const Quat qz{0, 0, std::sin(hz), std::cos(hz)};
    // Hamilton products: q = qz * qx * qy.
    auto mul = [](const Quat& a, const Quat& b) {
        return Quat{
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        };
    };
    return mul(qz, mul(qx, qy));
}

bool WorldManager::loadCadb(const std::string& path, std::string& err) {
    db_ = CadbDatabase{};
    return LoadCadbFile(path, db_, err);
}

namespace {
// 3D distance, matching the "within radius of this point" wording of the
// script API. Flat (2D) matching is a one-line change here if in-game
// behaviour ever disagrees.
bool matchesAnyRemove(const CadbPlacement& p,
                      const std::vector<WorldManager::RemoveEdit>& removes,
                      const WorldManager::RemoveEdit* extra = nullptr) {
    if (extra && extra->modelId == p.modelId &&
        (p.position - extra->pos).length() <= extra->radius)
        return true;
    for (const auto& r : removes) {
        if (r.modelId != p.modelId) continue;
        if ((p.position - r.pos).length() <= r.radius) return true;
    }
    return false;
}
} // namespace

size_t WorldManager::countMatching(uint16_t modelId, const Vec3& pos, float radius) const {
    const RemoveEdit probe{modelId, pos, radius};
    std::lock_guard<std::mutex> lk(editsMu_);
    size_t count = 0;
    for (const auto& p : db_.placements)
        if (matchesAnyRemove(p, removes_, &probe)) ++count;
    return count;
}

void WorldManager::removeBuilding(uint16_t modelId, const Vec3& pos, float radius) {
    std::lock_guard<std::mutex> lk(editsMu_);
    removes_.push_back({modelId, pos, radius});
}

bool WorldManager::addObject(uint16_t modelId, const Vec3& pos, const Quat& rot,
                             std::string& err) {
    const auto it = std::find_if(db_.models.begin(), db_.models.end(),
                                 [modelId](const CadbModel& m) { return m.modelId == modelId; });
    if (it == db_.models.end()) {
        err = "model id " + std::to_string(modelId) +
              " not in the CADB database - no collision geometry for it";
        return false;
    }
    std::lock_guard<std::mutex> lk(editsMu_);
    adds_.push_back({modelId, pos, rot});
    return true;
}

void WorldManager::clearEdits() {
    std::lock_guard<std::mutex> lk(editsMu_);
    removes_.clear();
    adds_.clear();
}

CadbDatabase WorldManager::editedDatabase() const {
    CadbDatabase edited;
    edited.version = db_.version;
    edited.models = db_.models;
    edited.placements.reserve(db_.placements.size() + adds_.size());
    for (const auto& p : db_.placements)
        if (!matchesAnyRemove(p, removes_)) edited.placements.push_back(p);
    for (const auto& a : adds_) {
        CadbPlacement p;
        p.modelId = a.modelId;
        p.position = a.pos;
        p.rotation = a.rotation;
        edited.placements.push_back(p);
    }
    return edited;
}

CollisionMesh WorldManager::assembleEdited() const {
    // Snapshot the edits (and skip the placement copy entirely when there is
    // nothing to edit - the common case on an unmodified server), then
    // assemble outside the lock: assembly is the long part and concurrent
    // edit recording must not wait for it.
    CadbDatabase edited;
    {
        std::lock_guard<std::mutex> lk(editsMu_);
        if (removes_.empty() && adds_.empty()) return AssembleWorldMesh(db_, opt_);
        edited = editedDatabase();
    }
    return AssembleWorldMesh(edited, opt_);
}

} // namespace wqs
