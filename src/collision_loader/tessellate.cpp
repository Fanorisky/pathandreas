#include "collision_loader/collision_loader.h"
#include <cmath>

namespace wqs {
namespace {

Quat identity() { return {0.f, 0.f, 0.f, 1.f}; }

} // namespace

void AppendTriangle(CollisionMesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c,
                    const Vec3& pos, const Quat& rot, uint8_t mat) {
    const uint32_t base = mesh.vertexCount();
    mesh.addVertex(transformPoint(pos, rot, a));
    mesh.addVertex(transformPoint(pos, rot, b));
    mesh.addVertex(transformPoint(pos, rot, c));
    mesh.addTriangle(base, base + 1, base + 2, mat);
}

void AppendBox(CollisionMesh& mesh, const Vec3& center, const Vec3& he,
               const Vec3& pos, const Quat& rot, uint8_t mat) {
    // 8 corners in local (model) space, then transform.
    const Vec3 c = center;
    Vec3 p[8] = {
        {c.x - he.x, c.y - he.y, c.z - he.z},
        {c.x + he.x, c.y - he.y, c.z - he.z},
        {c.x + he.x, c.y + he.y, c.z - he.z},
        {c.x - he.x, c.y + he.y, c.z - he.z},
        {c.x - he.x, c.y - he.y, c.z + he.z},
        {c.x + he.x, c.y - he.y, c.z + he.z},
        {c.x + he.x, c.y + he.y, c.z + he.z},
        {c.x - he.x, c.y + he.y, c.z + he.z},
    };
    const uint32_t base = mesh.vertexCount();
    for (int i = 0; i < 8; ++i) mesh.addVertex(transformPoint(pos, rot, p[i]));
    const uint32_t faces[12][3] = {
        {0, 1, 2}, {0, 2, 3}, // -Z
        {4, 6, 5}, {4, 7, 6}, // +Z
        {0, 4, 5}, {0, 5, 1}, // -Y
        {3, 2, 6}, {3, 6, 7}, // +Y
        {0, 3, 7}, {0, 7, 4}, // -X
        {1, 5, 6}, {1, 6, 2}, // +X
    };
    for (auto& f : faces) mesh.addTriangle(base + f[0], base + f[1], base + f[2], mat);
    (void)identity;
}

void AppendSphere(CollisionMesh& mesh, const Vec3& center, float radius,
                  const Vec3& pos, const Quat& rot, int subdiv, uint8_t mat) {
    // Unit icosahedron, then subdivide.
    const float t = (1.f + std::sqrt(5.f)) * 0.5f;
    std::vector<Vec3> verts = {
        {-1,  t, 0}, { 1,  t, 0}, {-1, -t, 0}, { 1, -t, 0},
        { 0, -1,  t}, { 0,  1,  t}, { 0, -1, -t}, { 0,  1, -t},
        { t,  0, -1}, { t,  0,  1}, {-t,  0, -1}, {-t,  0,  1},
    };
    for (auto& v : verts) v = v.normalized();
    std::vector<uint32_t> idx = {
        0,11,5,  0,5,1,  0,1,7,  0,7,10,  0,10,11,
        1,5,9,   5,11,4, 11,10,2, 10,7,6,  7,1,8,
        3,9,4,   3,4,2,  3,2,6,  3,6,8,   3,8,9,
        4,9,5,   2,4,11, 6,2,10, 8,6,7,   9,8,1
    };

    auto midpoint = [&](uint32_t a, uint32_t b) {
        Vec3 m = (verts[a] + verts[b]).normalized();
        uint32_t i = static_cast<uint32_t>(verts.size());
        verts.push_back(m);
        return i;
    };

    subdiv = std::max(0, std::min(subdiv, 3));
    for (int s = 0; s < subdiv; ++s) {
        std::vector<uint32_t> next;
        next.reserve(idx.size() * 4);
        for (size_t i = 0; i < idx.size(); i += 3) {
            uint32_t a = idx[i], b = idx[i + 1], c = idx[i + 2];
            uint32_t ab = midpoint(a, b);
            uint32_t bc = midpoint(b, c);
            uint32_t ca = midpoint(c, a);
            next.insert(next.end(), {a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca});
        }
        idx.swap(next);
    }

    const uint32_t base = mesh.vertexCount();
    for (const auto& v : verts) {
        mesh.addVertex(transformPoint(pos, rot, center + v * radius));
    }
    for (size_t i = 0; i < idx.size(); i += 3) {
        mesh.addTriangle(base + idx[i], base + idx[i + 1], base + idx[i + 2], mat);
    }
}

} // namespace wqs
