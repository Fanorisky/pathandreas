#pragma once

#include "common/vec3.h"
#include <vector>
#include <cstdint>

namespace wqs {

class TriangleBvh {
public:
    void build(const CollisionMesh& mesh);
    bool raycast(const Vec3& origin, const Vec3& dir, float tmax, RayHitResult& hit) const;
    const Aabb& bounds() const { return bounds_; }
    uint32_t triangleCount() const { return static_cast<uint32_t>(tris_.size()); }
    bool empty() const { return tris_.empty(); }

private:
    struct Node {
        Aabb bounds;
        int left = -1;   // child index, or first tri if leaf
        int right = -1;  // child index, or tri count if leaf
        bool leaf = false;
    };
    struct Tri {
        Vec3 a, b, c;
        Vec3 n;
    };

    std::vector<Node> nodes_;
    std::vector<Tri> tris_;
    std::vector<uint32_t> order_;
    Aabb bounds_;

    int buildRange(int begin, int end, int depth);
    bool intersectTri(const Tri& t, const Vec3& orig, const Vec3& dir, float tmax,
                      float& tHit, Vec3& n) const;
};

} // namespace wqs
