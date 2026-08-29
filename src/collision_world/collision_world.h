#pragma once

#include "common/vec3.h"
#include <memory>
#include <string>

namespace wqs {

class CollisionWorld {
public:
    CollisionWorld();
    ~CollisionWorld();

    CollisionWorld(const CollisionWorld&) = delete;
    CollisionWorld& operator=(const CollisionWorld&) = delete;

    // Build a static BVH (or Bullet btBvhTriangleMeshShape when WQS_USE_BULLET).
    bool build(const CollisionMesh& mesh, std::string& err);

    bool empty() const;

    // Closest hit along the segment [from, to]. Normal is unit, facing the ray.
    bool RayCastLine(const Vec3& from, const Vec3& to, RayHitResult& out) const;

    // Vertical downward ray. Starts at mesh-max-Z + 50 (or 1000) unless fromZ is set.
    bool FindGroundZ(float x, float y, float& outZ) const;
    bool FindGroundZFrom(float x, float y, float fromZ, float& outZ) const;

    const Aabb& bounds() const;
    uint32_t triangleCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace wqs
