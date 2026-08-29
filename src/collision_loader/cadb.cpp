#include "collision_loader/collision_loader.h"
#include "common/log.h"

#include <fstream>
#include <cstring>
#include <unordered_map>

namespace wqs {
namespace {

struct Reader {
    const uint8_t* p = nullptr;
    const uint8_t* end = nullptr;
    bool ok = true;

    explicit Reader(const std::vector<uint8_t>& buf)
        : p(buf.data()), end(buf.data() + buf.size()) {}

    size_t remain() const { return static_cast<size_t>(end - p); }

    template <class T>
    T read() {
        T v{};
        if (remain() < sizeof(T)) {
            ok = false;
            return v;
        }
        std::memcpy(&v, p, sizeof(T));
        p += sizeof(T);
        return v;
    }

    Vec3 readVec3() {
        Vec3 v;
        v.x = read<float>();
        v.y = read<float>();
        v.z = read<float>();
        return v;
    }
    Quat readQuat() {
        Quat q;
        q.x = read<float>();
        q.y = read<float>();
        q.z = read<float>();
        q.w = read<float>();
        return q;
    }
};

bool parseCadf(Reader& r, CadbDatabase& out, std::string& err) {
    out.version = r.read<uint16_t>();
    const uint16_t modelCount = r.read<uint16_t>();
    const uint32_t iplCount = r.read<uint32_t>();
    if (!r.ok) {
        err = "CADB header truncated";
        return false;
    }
    out.models.resize(modelCount);
    for (uint16_t i = 0; i < modelCount; ++i) {
        CadbModel& m = out.models[i];
        m.modelId = r.read<uint16_t>();
        const uint16_t nSph = r.read<uint16_t>();
        const uint16_t nBox = r.read<uint16_t>();
        const uint16_t nFace = r.read<uint16_t>();
        if (!r.ok) {
            err = "CADB model header truncated";
            return false;
        }
        m.sphereCenters.reserve(nSph);
        m.sphereRadii.reserve(nSph);
        for (uint16_t s = 0; s < nSph; ++s) {
            m.sphereCenters.push_back(r.readVec3());
            m.sphereRadii.push_back(r.read<float>());
        }
        m.boxCenters.reserve(nBox);
        m.boxHalfExtents.reserve(nBox);
        for (uint16_t b = 0; b < nBox; ++b) {
            m.boxCenters.push_back(r.readVec3());
            m.boxHalfExtents.push_back(r.readVec3());
        }
        m.faceA.reserve(nFace);
        m.faceB.reserve(nFace);
        m.faceC.reserve(nFace);
        for (uint16_t f = 0; f < nFace; ++f) {
            m.faceA.push_back(r.readVec3());
            m.faceB.push_back(r.readVec3());
            m.faceC.push_back(r.readVec3());
        }
        if (!r.ok) {
            err = "CADB model body truncated";
            return false;
        }
    }
    out.placements.resize(iplCount);
    for (uint32_t i = 0; i < iplCount; ++i) {
        CadbPlacement& p = out.placements[i];
        p.modelId = r.read<uint16_t>();
        p.position = r.readVec3();
        p.rotation = r.readQuat();
        if (!r.ok) {
            err = "CADB IPL truncated";
            return false;
        }
    }
    return true;
}

// Older documented layout: magic Cskp, uint32 modelCount, uint32 iplCount,
// per model uint32 modelid/spheres/boxes/faces.
bool parseCskp(Reader& r, CadbDatabase& out, std::string& err) {
    out.version = 0;
    const uint32_t modelCount = r.read<uint32_t>();
    const uint32_t iplCount = r.read<uint32_t>();
    if (!r.ok) {
        err = "Cskp header truncated";
        return false;
    }
    if (modelCount > 100000 || iplCount > 5000000) {
        err = "Cskp counts look invalid";
        return false;
    }
    out.models.resize(modelCount);
    for (uint32_t i = 0; i < modelCount; ++i) {
        CadbModel& m = out.models[i];
        m.modelId = static_cast<uint16_t>(r.read<uint32_t>());
        const uint32_t nSph = r.read<uint32_t>();
        const uint32_t nBox = r.read<uint32_t>();
        const uint32_t nFace = r.read<uint32_t>();
        for (uint32_t s = 0; s < nSph; ++s) {
            m.sphereCenters.push_back(r.readVec3());
            m.sphereRadii.push_back(r.read<float>());
        }
        for (uint32_t b = 0; b < nBox; ++b) {
            m.boxCenters.push_back(r.readVec3());
            m.boxHalfExtents.push_back(r.readVec3());
        }
        for (uint32_t f = 0; f < nFace; ++f) {
            m.faceA.push_back(r.readVec3());
            m.faceB.push_back(r.readVec3());
            m.faceC.push_back(r.readVec3());
        }
        if (!r.ok) {
            err = "Cskp model truncated";
            return false;
        }
    }
    out.placements.resize(iplCount);
    for (uint32_t i = 0; i < iplCount; ++i) {
        CadbPlacement& p = out.placements[i];
        p.modelId = static_cast<uint16_t>(r.read<uint32_t>());
        p.position = r.readVec3();
        p.rotation = r.readQuat();
        if (!r.ok) {
            err = "Cskp IPL truncated";
            return false;
        }
    }
    return true;
}

} // namespace

bool LoadCadbFile(const std::string& path, CadbDatabase& out, std::string& err) {
    out = {};
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        err = "cannot open " + path;
        return false;
    }
    in.seekg(0, std::ios::end);
    const std::streamoff n = in.tellg();
    in.seekg(0, std::ios::beg);
    if (n < 8) {
        err = "file too small to be CADB";
        return false;
    }
    std::vector<uint8_t> buf(static_cast<size_t>(n));
    in.read(reinterpret_cast<char*>(buf.data()), n);
    Reader r(buf);
    char magic[4];
    magic[0] = static_cast<char>(r.read<uint8_t>());
    magic[1] = static_cast<char>(r.read<uint8_t>());
    magic[2] = static_cast<char>(r.read<uint8_t>());
    magic[3] = static_cast<char>(r.read<uint8_t>());
    if (std::memcmp(magic, "cadf", 4) == 0) {
        if (!parseCadf(r, out, err)) return false;
        WQS_INFO("CADB cadf v%u: %zu models, %zu placements",
                 out.version, out.models.size(), out.placements.size());
        return true;
    }
    if (std::memcmp(magic, "Cskp", 4) == 0) {
        if (!parseCskp(r, out, err)) return false;
        WQS_INFO("CADB Cskp: %zu models, %zu placements",
                 out.models.size(), out.placements.size());
        return true;
    }
    err = "unrecognised CADB magic (expected cadf or Cskp)";
    return false;
}

CollisionMesh AssembleWorldMesh(const CadbDatabase& db, const LoaderOptions& opt) {
    std::unordered_map<uint16_t, const CadbModel*> byId;
    byId.reserve(db.models.size() * 2);
    for (const auto& m : db.models) byId[m.modelId] = &m;

    CollisionMesh mesh;
    size_t missing = 0;
    for (const auto& p : db.placements) {
        auto it = byId.find(p.modelId);
        if (it == byId.end()) {
            ++missing;
            continue;
        }
        const CadbModel& m = *it->second;
        if (opt.clipRegion) {
            Aabb pb;
            pb.expand(p.position);
            // Cheap reject: if the placement origin is far outside the region,
            // skip. (Models can extend a few tens of units.)
            Aabb padded = opt.region;
            padded.min -= Vec3{80, 80, 80};
            padded.max += Vec3{80, 80, 80};
            if (!pb.overlaps(padded)) continue;
        }
        for (size_t i = 0; i < m.faceA.size(); ++i) {
            AppendTriangle(mesh, m.faceA[i], m.faceB[i], m.faceC[i],
                           p.position, p.rotation, 0);
        }
        for (size_t i = 0; i < m.boxCenters.size(); ++i) {
            AppendBox(mesh, m.boxCenters[i], m.boxHalfExtents[i],
                      p.position, p.rotation, 0);
        }
        for (size_t i = 0; i < m.sphereCenters.size(); ++i) {
            AppendSphere(mesh, m.sphereCenters[i], m.sphereRadii[i],
                         p.position, p.rotation, opt.sphereSubdiv, 0);
        }
    }
    if (missing) WQS_WARN("CADB: %zu placements referenced unknown models", missing);
    WQS_INFO("World mesh: %u verts, %u tris", mesh.vertexCount(), mesh.triangleCount());
    return mesh;
}

CollisionMesh LoadFromCadb(const std::string& path, const LoaderOptions& opt) {
    CadbDatabase db;
    std::string err;
    if (!LoadCadbFile(path, db, err)) {
        WQS_ERROR("LoadFromCadb: %s", err.c_str());
        return {};
    }
    return AssembleWorldMesh(db, opt);
}

bool WriteCadbFile(const std::string& path, const CadbDatabase& db, std::string& err) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        err = "cannot write " + path;
        return false;
    }
    auto wr = [&](const void* p, size_t n) { out.write(reinterpret_cast<const char*>(p), static_cast<std::streamsize>(n)); };
    auto wr16 = [&](uint16_t v) { wr(&v, 2); };
    auto wr32 = [&](uint32_t v) { wr(&v, 4); };
    auto wrf = [&](float v) { wr(&v, 4); };
    auto wrv = [&](const Vec3& v) { wrf(v.x); wrf(v.y); wrf(v.z); };
    auto wrq = [&](const Quat& q) { wrf(q.x); wrf(q.y); wrf(q.z); wrf(q.w); };

    wr("cadf", 4);
    wr16(db.version ? db.version : 1);
    wr16(static_cast<uint16_t>(db.models.size()));
    wr32(static_cast<uint32_t>(db.placements.size()));
    for (const auto& m : db.models) {
        wr16(m.modelId);
        wr16(static_cast<uint16_t>(m.sphereCenters.size()));
        wr16(static_cast<uint16_t>(m.boxCenters.size()));
        wr16(static_cast<uint16_t>(m.faceA.size()));
        for (size_t i = 0; i < m.sphereCenters.size(); ++i) {
            wrv(m.sphereCenters[i]);
            wrf(m.sphereRadii[i]);
        }
        for (size_t i = 0; i < m.boxCenters.size(); ++i) {
            wrv(m.boxCenters[i]);
            wrv(m.boxHalfExtents[i]);
        }
        for (size_t i = 0; i < m.faceA.size(); ++i) {
            wrv(m.faceA[i]);
            wrv(m.faceB[i]);
            wrv(m.faceC[i]);
        }
    }
    for (const auto& p : db.placements) {
        wr16(p.modelId);
        wrv(p.position);
        wrq(p.rotation);
    }
    if (!out) {
        err = "write failed";
        return false;
    }
    return true;
}

} // namespace wqs
