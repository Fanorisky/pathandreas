#include "collision_world/collision_world.h"
#include "collision_world/bvh.h"
#include "common/log.h"

#ifdef WQS_USE_BULLET
#include <btBulletCollisionCommon.h>
#endif

namespace wqs {

struct CollisionWorld::Impl {
#ifdef WQS_USE_BULLET
    btDefaultCollisionConfiguration* config = nullptr;
    btCollisionDispatcher* dispatcher = nullptr;
    btDbvtBroadphase* broadphase = nullptr;
    btCollisionWorld* world = nullptr;
    btTriangleIndexVertexArray* iface = nullptr;
    btBvhTriangleMeshShape* shape = nullptr;
    btCollisionObject* object = nullptr;
    std::vector<float> verts;
    std::vector<int> indices;
    Aabb bounds;
    uint32_t triCount = 0;

    ~Impl() {
        if (world && object) world->removeCollisionObject(object);
        delete object;
        delete shape;
        delete iface;
        delete world;
        delete broadphase;
        delete dispatcher;
        delete config;
    }
#else
    TriangleBvh bvh;
#endif
};

CollisionWorld::CollisionWorld() : impl_(new Impl) {}
CollisionWorld::~CollisionWorld() = default;

bool CollisionWorld::build(const CollisionMesh& mesh, std::string& err) {
    if (mesh.triangleCount() == 0) {
        err = "empty mesh";
        return false;
    }
#ifdef WQS_USE_BULLET
    impl_.reset(new Impl);
    impl_->verts = mesh.vertices;
    impl_->indices.resize(mesh.indices.size());
    for (size_t i = 0; i < mesh.indices.size(); ++i)
        impl_->indices[i] = static_cast<int>(mesh.indices[i]);
    impl_->triCount = mesh.triangleCount();
    impl_->bounds = mesh.bounds();

    impl_->iface = new btTriangleIndexVertexArray(
        static_cast<int>(impl_->triCount),
        impl_->indices.data(),
        static_cast<int>(3 * sizeof(int)),
        static_cast<int>(impl_->verts.size() / 3),
        impl_->verts.data(),
        static_cast<int>(3 * sizeof(float)));
    impl_->shape = new btBvhTriangleMeshShape(impl_->iface, true, true);

    impl_->config = new btDefaultCollisionConfiguration();
    impl_->dispatcher = new btCollisionDispatcher(impl_->config);
    impl_->broadphase = new btDbvtBroadphase();
    impl_->world = new btCollisionWorld(impl_->dispatcher, impl_->broadphase, impl_->config);

    impl_->object = new btCollisionObject();
    impl_->object->setCollisionShape(impl_->shape);
    impl_->object->setWorldTransform(btTransform::getIdentity());
    impl_->world->addCollisionObject(impl_->object);
    WQS_INFO("Bullet world: %u tris", impl_->triCount);
    return true;
#else
    impl_->bvh.build(mesh);
    WQS_INFO("BVH world: %u tris", impl_->bvh.triangleCount());
    return true;
#endif
}

bool CollisionWorld::empty() const {
#ifdef WQS_USE_BULLET
    return impl_->triCount == 0;
#else
    return impl_->bvh.empty();
#endif
}

bool CollisionWorld::RayCastLine(const Vec3& from, const Vec3& to, RayHitResult& out) const {
    out = {};
#ifdef WQS_USE_BULLET
    if (!impl_->world) return false;
    btVector3 bfrom(from.x, from.y, from.z);
    btVector3 bto(to.x, to.y, to.z);
    btCollisionWorld::ClosestRayResultCallback cb(bfrom, bto);
    impl_->world->rayTest(bfrom, bto, cb);
    if (!cb.hasHit()) return false;
    out.hit = true;
    out.fraction = cb.m_closestHitFraction;
    out.point = {cb.m_hitPointWorld.x(), cb.m_hitPointWorld.y(), cb.m_hitPointWorld.z()};
    out.normal = {cb.m_hitNormalWorld.x(), cb.m_hitNormalWorld.y(), cb.m_hitNormalWorld.z()};
    return true;
#else
    const Vec3 delta = to - from;
    const float len = delta.length();
    if (len < 1e-8f) return false;
    const Vec3 dir = delta / len;
    if (!impl_->bvh.raycast(from, dir, len, out)) return false;
    return true;
#endif
}

bool CollisionWorld::FindGroundZFrom(float x, float y, float fromZ, float& outZ) const {
    RayHitResult hit;
    const Vec3 from{x, y, fromZ};
    const Vec3 to{x, y, fromZ - 5000.f};
    if (!RayCastLine(from, to, hit)) return false;
    outZ = hit.point.z;
    return true;
}

bool CollisionWorld::FindGroundZ(float x, float y, float& outZ) const {
    float top = 1000.f;
#ifdef WQS_USE_BULLET
    if (impl_->bounds.valid()) top = impl_->bounds.max.z + 50.f;
#else
    if (impl_->bvh.bounds().valid()) top = impl_->bvh.bounds().max.z + 50.f;
#endif
    return FindGroundZFrom(x, y, top, outZ);
}

const Aabb& CollisionWorld::bounds() const {
#ifdef WQS_USE_BULLET
    return impl_->bounds;
#else
    return impl_->bvh.bounds();
#endif
}

uint32_t CollisionWorld::triangleCount() const {
#ifdef WQS_USE_BULLET
    return impl_->triCount;
#else
    return impl_->bvh.triangleCount();
#endif
}

} // namespace wqs
