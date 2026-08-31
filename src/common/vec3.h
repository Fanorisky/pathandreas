#pragma once

#include <cmath>
#include <cstdint>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>

namespace wqs {

struct Vec3 {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    float lengthSq() const { return x * x + y * y + z * z; }
    float length() const { return std::sqrt(lengthSq()); }

    Vec3 normalized() const {
        float l = length();
        return l > 1e-12f ? (*this / l) : Vec3{};
    }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }
inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Vec3 minv(const Vec3& a, const Vec3& b) {
    return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}
inline Vec3 maxv(const Vec3& a, const Vec3& b) {
    return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}

struct Quat {
    float x = 0.f, y = 0.f, z = 0.f, w = 1.f;
};

// Rotate vector by quaternion (q * v * q^-1). Matches ColAndreas/Bullet (x,y,z,w).
inline Vec3 rotate(const Quat& q, const Vec3& v) {
    const Vec3 u{q.x, q.y, q.z};
    const float s = q.w;
    return u * (2.f * dot(u, v)) + v * (s * s - dot(u, u)) + cross(u, v) * (2.f * s);
}

inline Vec3 transformPoint(const Vec3& pos, const Quat& rot, const Vec3& local) {
    return pos + rotate(rot, local);
}

struct Aabb {
    Vec3 min{ std::numeric_limits<float>::max(),
              std::numeric_limits<float>::max(),
              std::numeric_limits<float>::max() };
    Vec3 max{ std::numeric_limits<float>::lowest(),
              std::numeric_limits<float>::lowest(),
              std::numeric_limits<float>::lowest() };

    void expand(const Vec3& p) {
        min = minv(min, p);
        max = maxv(max, p);
    }
    void expand(const Aabb& o) {
        min = minv(min, o.min);
        max = maxv(max, o.max);
    }
    Vec3 center() const { return (min + max) * 0.5f; }
    Vec3 extent() const { return (max - min) * 0.5f; }
    Vec3 size() const { return max - min; }
    bool valid() const { return min.x <= max.x; }

    bool overlaps(const Aabb& o) const {
        return min.x <= o.max.x && max.x >= o.min.x &&
               min.y <= o.max.y && max.y >= o.min.y &&
               min.z <= o.max.z && max.z >= o.min.z;
    }
};

// GTA SA is Z-up. Recast/Detour is Y-up.
// Proper rotation (90° about X): (x, y, z) -> (x, z, -y) preserves winding
// so ground facing +Z stays walkable (facing +Y in Recast).
inline void gtaToRecast(const Vec3& g, float* r) {
    r[0] = g.x;
    r[1] = g.z;
    r[2] = -g.y;
}
inline Vec3 recastToGta(const float* r) {
    return {r[0], -r[2], r[1]};
}

struct RayHitResult {
    bool hit = false;
    Vec3 point{};
    Vec3 normal{};
    float fraction = 1.f; // 0 at origin, 1 at end
};

struct PathResult {
    std::vector<Vec3> waypoints;
    bool success = false;
    // True when Detour could not reach the goal polygon and returned the best
    // partial path instead (DT_PARTIAL_RESULT). success=true + partial=true
    // means "walk this far, but the destination is unreachable from here".
    bool partial = false;
    // Parallel to waypoints: 1 when the step from waypoint i to i+1 crosses an
    // off-mesh connection - a baked step or climb link, not walkable surface.
    // move_along_surface cannot traverse one: it slides along polygons and an
    // off-mesh link is not a polygon, so a controller that only slides will
    // stall there. Move directly (and play a climb animation) for that step.
    std::vector<uint8_t> offMesh;
};

struct CollisionMesh {
    std::vector<float> vertices;    // x,y,z packed
    std::vector<uint32_t> indices;  // 3 per triangle
    std::vector<uint8_t> materials; // optional, 1 per triangle (0 = default)

    uint32_t vertexCount() const { return static_cast<uint32_t>(vertices.size() / 3); }
    uint32_t triangleCount() const { return static_cast<uint32_t>(indices.size() / 3); }

    Vec3 vertex(uint32_t i) const {
        return {vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]};
    }
    void addVertex(const Vec3& v) {
        vertices.push_back(v.x);
        vertices.push_back(v.y);
        vertices.push_back(v.z);
    }
    void addTriangle(uint32_t a, uint32_t b, uint32_t c, uint8_t mat = 0) {
        indices.push_back(a);
        indices.push_back(b);
        indices.push_back(c);
        materials.push_back(mat);
    }
    Aabb bounds() const {
        Aabb b;
        for (uint32_t i = 0; i < vertexCount(); ++i) b.expand(vertex(i));
        return b;
    }
    void clear() {
        vertices.clear();
        indices.clear();
        materials.clear();
    }
    void append(const CollisionMesh& o) {
        const uint32_t base = vertexCount();
        vertices.insert(vertices.end(), o.vertices.begin(), o.vertices.end());
        for (uint32_t idx : o.indices) indices.push_back(base + idx);
        materials.insert(materials.end(), o.materials.begin(), o.materials.end());
    }
};

// Surface material ids used when tessellating / tagging.
enum class SurfaceKind : uint8_t {
    Default = 0,
    Water   = 1,
};

} // namespace wqs
