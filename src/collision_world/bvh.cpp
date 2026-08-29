#include "collision_world/bvh.h"
#include <algorithm>
#include <cmath>

namespace wqs {
namespace {

constexpr float kEps = 1e-8f;

bool aabbRay(const Aabb& b, const Vec3& orig, const Vec3& invDir, float tmax, float& tEnter) {
    float tmin = 0.f;
    float tmax2 = tmax;
    const float bounds[2][3] = {
        {b.min.x, b.min.y, b.min.z},
        {b.max.x, b.max.y, b.max.z},
    };
    const float o[3] = {orig.x, orig.y, orig.z};
    const float id[3] = {invDir.x, invDir.y, invDir.z};
    for (int a = 0; a < 3; ++a) {
        float t0 = (bounds[0][a] - o[a]) * id[a];
        float t1 = (bounds[1][a] - o[a]) * id[a];
        if (id[a] < 0.f) std::swap(t0, t1);
        tmin = t0 > tmin ? t0 : tmin;
        tmax2 = t1 < tmax2 ? t1 : tmax2;
        if (tmax2 < tmin) return false;
    }
    tEnter = tmin;
    return true;
}

} // namespace

void TriangleBvh::build(const CollisionMesh& mesh) {
    nodes_.clear();
    tris_.clear();
    order_.clear();
    bounds_ = {};

    const uint32_t n = mesh.triangleCount();
    tris_.reserve(n);
    order_.resize(n);
    for (uint32_t i = 0; i < n; ++i) {
        const Vec3 a = mesh.vertex(mesh.indices[i * 3]);
        const Vec3 b = mesh.vertex(mesh.indices[i * 3 + 1]);
        const Vec3 c = mesh.vertex(mesh.indices[i * 3 + 2]);
        Tri t;
        t.a = a;
        t.b = b;
        t.c = c;
        t.n = cross(b - a, c - a).normalized();
        tris_.push_back(t);
        order_[i] = i;
        bounds_.expand(a);
        bounds_.expand(b);
        bounds_.expand(c);
    }
    nodes_.reserve(n * 2);
    if (n) buildRange(0, static_cast<int>(n), 0);
}

int TriangleBvh::buildRange(int begin, int end, int depth) {
    Node node;
    Aabb b;
    for (int i = begin; i < end; ++i) {
        const Tri& t = tris_[order_[static_cast<size_t>(i)]];
        b.expand(t.a);
        b.expand(t.b);
        b.expand(t.c);
    }
    node.bounds = b;
    const int count = end - begin;
    const int self = static_cast<int>(nodes_.size());
    nodes_.push_back(node);

    if (count <= 4 || depth > 24) {
        nodes_[self].leaf = true;
        nodes_[self].left = begin;
        nodes_[self].right = count;
        return self;
    }

    const Vec3 sz = b.size();
    int axis = 0;
    if (sz.y > sz.x && sz.y >= sz.z) axis = 1;
    else if (sz.z > sz.x && sz.z >= sz.y) axis = 2;

    const int mid = (begin + end) / 2;
    std::nth_element(order_.begin() + begin, order_.begin() + mid, order_.begin() + end,
                     [&](uint32_t ia, uint32_t ib) {
                         const Tri& ta = tris_[ia];
                         const Tri& tb = tris_[ib];
                         const float ca = ((&ta.a.x)[axis] + (&ta.b.x)[axis] + (&ta.c.x)[axis]);
                         const float cb = ((&tb.a.x)[axis] + (&tb.b.x)[axis] + (&tb.c.x)[axis]);
                         return ca < cb;
                     });

    const int L = buildRange(begin, mid, depth + 1);
    const int R = buildRange(mid, end, depth + 1);
    nodes_[self].left = L;
    nodes_[self].right = R;
    nodes_[self].leaf = false;
    return self;
}

bool TriangleBvh::intersectTri(const Tri& tri, const Vec3& orig, const Vec3& dir, float tmax,
                               float& tHit, Vec3& n) const {
    // Möller–Trumbore
    const Vec3 e1 = tri.b - tri.a;
    const Vec3 e2 = tri.c - tri.a;
    const Vec3 p = cross(dir, e2);
    const float det = dot(e1, p);
    if (det > -kEps && det < kEps) return false;
    const float inv = 1.f / det;
    const Vec3 tvec = orig - tri.a;
    const float u = dot(tvec, p) * inv;
    if (u < 0.f || u > 1.f) return false;
    const Vec3 q = cross(tvec, e1);
    const float v = dot(dir, q) * inv;
    if (v < 0.f || u + v > 1.f) return false;
    const float t = dot(e2, q) * inv;
    if (t <= kEps || t >= tmax) return false;
    tHit = t;
    n = det < 0.f ? -tri.n : tri.n;
    if (n.lengthSq() < 1e-12f) n = cross(e1, e2).normalized();
    if (dot(n, dir) > 0.f) n = -n;
    return true;
}

bool TriangleBvh::raycast(const Vec3& origin, const Vec3& dir, float tmax, RayHitResult& hit) const {
    hit = {};
    if (nodes_.empty()) return false;
    const Vec3 invDir{
        std::fabs(dir.x) > kEps ? 1.f / dir.x : (dir.x >= 0.f ? 1e30f : -1e30f),
        std::fabs(dir.y) > kEps ? 1.f / dir.y : (dir.y >= 0.f ? 1e30f : -1e30f),
        std::fabs(dir.z) > kEps ? 1.f / dir.z : (dir.z >= 0.f ? 1e30f : -1e30f),
    };

    int stack[64];
    int sp = 0;
    stack[sp++] = 0;
    float best = tmax;
    Vec3 bestN{};
    bool found = false;

    while (sp) {
        const Node& node = nodes_[stack[--sp]];
        float tEnter = 0.f;
        if (!aabbRay(node.bounds, origin, invDir, best, tEnter)) continue;
        if (node.leaf) {
            for (int i = 0; i < node.right; ++i) {
                const Tri& tri = tris_[order_[static_cast<size_t>(node.left + i)]];
                float tHit;
                Vec3 n;
                if (intersectTri(tri, origin, dir, best, tHit, n)) {
                    best = tHit;
                    bestN = n;
                    found = true;
                }
            }
        } else {
            if (sp < 62) {
                stack[sp++] = node.left;
                stack[sp++] = node.right;
            }
        }
    }
    if (!found) return false;
    hit.hit = true;
    hit.fraction = (tmax > kEps) ? (best / tmax) : 0.f;
    hit.point = origin + dir * best;
    hit.normal = bestN.normalized();
    return true;
}

} // namespace wqs
