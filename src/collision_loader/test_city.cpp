#include "collision_loader/collision_loader.h"

namespace wqs {
namespace {

void addBoxModel(CadbDatabase& db, uint16_t id, const Vec3& half) {
    CadbModel m;
    m.modelId = id;
    m.boxCenters.push_back({0, 0, 0});
    m.boxHalfExtents.push_back(half);
    db.models.push_back(std::move(m));
}

void addFaceModel(CadbDatabase& db, uint16_t id, const std::vector<Vec3>& tris) {
    CadbModel m;
    m.modelId = id;
    for (size_t i = 0; i + 2 < tris.size(); i += 3) {
        m.faceA.push_back(tris[i]);
        m.faceB.push_back(tris[i + 1]);
        m.faceC.push_back(tris[i + 2]);
    }
    db.models.push_back(std::move(m));
}

void place(CadbDatabase& db, uint16_t id, const Vec3& p, const Quat& q = {0, 0, 0, 1}) {
    db.placements.push_back({id, p, q});
}

// Two triangles making a quad in XY, at local z=0, size 2*hx by 2*hy.
void addQuadModel(CadbDatabase& db, uint16_t id, float hx, float hy) {
    addFaceModel(db, id, {
        {-hx, -hy, 0}, { hx, -hy, 0}, { hx,  hy, 0},
        {-hx, -hy, 0}, { hx,  hy, 0}, {-hx,  hy, 0},
    });
}

} // namespace

CadbDatabase MakeTestCity() {
    CadbDatabase db;
    db.version = 1;

    // 1: ground slab 8x8 x 0.25 thick, centered so top is at z=+0.25
    addBoxModel(db, 1, {4.f, 4.f, 0.25f});
    // 2: building storey 6x6 x 3
    addBoxModel(db, 2, {3.f, 3.f, 1.5f});
    // 3: stair step 2 x 0.8 x 0.2
    addBoxModel(db, 3, {1.0f, 0.4f, 0.2f});
    // 4: overpass deck 20 x 3 x 0.25
    addBoxModel(db, 4, {10.f, 1.5f, 0.25f});
    // 5: overpass pillar 0.4 x 0.4 x 4
    addBoxModel(db, 5, {0.4f, 0.4f, 4.f});
    // 6: water quad (tagged later via faces; here a thin box below ground)
    addBoxModel(db, 6, {8.f, 3.f, 0.05f});
    // 7: slope ramp 8 x 4, rising 3 units (triangle mesh)
    addFaceModel(db, 7, {
        {-4, -2, 0}, { 4, -2, 3}, { 4,  2, 3},
        {-4, -2, 0}, { 4,  2, 3}, {-4,  2, 0},
    });
    // 8: wall 0.3 x 6 x 3
    addBoxModel(db, 8, {0.15f, 3.f, 1.5f});
    // 9: plaza raised platform 8x8 x 0.4
    addBoxModel(db, 9, {4.f, 4.f, 0.4f});
    addQuadModel(db, 10, 60.f, 60.f); // giant ground plane at z=0 of model

    // Ground plane at z = 0 (model 10 is a quad on z=0).
    place(db, 10, {0, 0, 0});

    // Road-adjacent ground fill is the plane itself.

    // City blocks: 3x3 buildings around origin, leaving a plaza at center.
    const float block = 18.f;
    int b = 0;
    for (int gx = -2; gx <= 2; ++gx) {
        for (int gy = -2; gy <= 2; ++gy) {
            if (gx == 0 && gy == 0) continue; // plaza
            if (gx == 0 && gy == 1) continue; // stairs approach
            const float x = gx * block;
            const float y = gy * block;
            const int floors = 1 + ((gx + gy + 8) % 3);
            for (int f = 0; f < floors; ++f) {
                place(db, 2, {x, y, 1.5f + f * 3.0f});
            }
            ++b;
        }
    }

    // Plaza platform and stairs leading up from -Y.
    place(db, 9, {0, 0, 0.4f}); // top at z=0.8
    for (int i = 0; i < 4; ++i) {
        const float z = 0.2f + i * 0.4f;
        const float y = -6.0f + i * 0.8f;
        place(db, 3, {0, y, z});
    }

    // Overpass along X through y=+36, deck at z=8.
    place(db, 4, {0, 36, 8.0f});
    place(db, 4, {-20, 36, 8.0f});
    place(db, 4, {20, 36, 8.0f});
    place(db, 5, {-18, 36, 4.0f});
    place(db, 5, {0, 36, 4.0f});
    place(db, 5, {18, 36, 4.0f});

    // Canal / water at y=-40, z=-0.2 (below ground, unwalkable visually).
    place(db, 6, {0, -42, -0.2f});
    place(db, 6, {16, -42, -0.2f});
    place(db, 6, {-16, -42, -0.2f});

    // Slope in the +X +Y corner.
    place(db, 7, {40, 40, 0});

    // Blocking wall on the plaza's +X side (forces path around).
    place(db, 8, {6.0f, 0.0f, 1.5f});

    return db;
}

CollisionMesh MakeTestCityMesh() {
    return AssembleWorldMesh(MakeTestCity(), {});
}

} // namespace wqs
