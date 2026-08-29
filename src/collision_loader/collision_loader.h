#pragma once

#include "common/vec3.h"
#include <string>
#include <vector>
#include <cstdint>

namespace wqs {

struct CadbModel {
    uint16_t modelId = 0;
    std::vector<Vec3> sphereCenters;
    std::vector<float> sphereRadii;
    // Boxes stored as center + half-extents (ColAndreas/Bullet convention).
    std::vector<Vec3> boxCenters;
    std::vector<Vec3> boxHalfExtents;
    std::vector<Vec3> faceA, faceB, faceC;
};

struct CadbPlacement {
    uint16_t modelId = 0;
    Vec3 position{};
    Quat rotation{};
};

struct CadbDatabase {
    uint16_t version = 0;
    std::vector<CadbModel> models;
    std::vector<CadbPlacement> placements;
};

struct LoaderOptions {
    // Tessellation quality for spheres converted to triangles.
    int sphereSubdiv = 2;
    // Skip water-tagged faces when assembling a walkable mesh.
    bool excludeWater = false;
    // Optional axis-aligned region filter in GTA coordinates (inclusive).
    bool clipRegion = false;
    Aabb region{};
};

// --- CADB (ColAndreas Wizard database) ---
// Format (original parser, not a GPL port):
//   magic "cadf" (4)
//   uint16 version
//   uint16 modelCount
//   uint32 iplCount
//   per model: uint16 modelId, sphereCount, boxCount, faceCount
//              spheres {vec3 offset, float radius} *
//              boxes   {vec3 center, vec3 size} *     size = half-extents
//              faces   {vec3 a, vec3 b, vec3 c} *
//   per IPL:   uint16 modelId, vec3 pos, quat xyzw
// Also accepts the older documented "Cskp" layout (uint32 counts, no version).
bool LoadCadbFile(const std::string& path, CadbDatabase& out, std::string& err);
CollisionMesh AssembleWorldMesh(const CadbDatabase& db, const LoaderOptions& opt = {});

// Convenience: load + assemble.
CollisionMesh LoadFromCadb(const std::string& path, const LoaderOptions& opt = {});

// Write a CADB (used by tests / synthetic maps). Always writes cadf v1.
bool WriteCadbFile(const std::string& path, const CadbDatabase& db, std::string& err);

// --- Raw GTA SA .col (COL1/COL2/COL3/COL4) ---
// A .col file is a concatenation of collision chunks. Result is model-local
// (no IPL). Spheres/boxes are tessellated into triangles.
CollisionMesh LoadFromCol(const std::string& path, const LoaderOptions& opt = {});

// --- Synthetic city used for tests and the live demo (no GTA IP). ---
CadbDatabase MakeTestCity();
CollisionMesh MakeTestCityMesh();

// Tessellation helpers (also used by COL/CADB assembly).
void AppendBox(CollisionMesh& mesh, const Vec3& center, const Vec3& halfExtents,
               const Vec3& pos, const Quat& rot, uint8_t mat = 0);
void AppendSphere(CollisionMesh& mesh, const Vec3& center, float radius,
                  const Vec3& pos, const Quat& rot, int subdiv, uint8_t mat = 0);
void AppendTriangle(CollisionMesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c,
                    const Vec3& pos, const Quat& rot, uint8_t mat = 0);

} // namespace wqs
