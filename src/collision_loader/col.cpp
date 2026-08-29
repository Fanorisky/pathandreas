#include "collision_loader/collision_loader.h"
#include "common/log.h"

#include <fstream>
#include <cstring>
#include <vector>

namespace wqs {
namespace {

struct Reader {
    const uint8_t* base = nullptr;
    const uint8_t* p = nullptr;
    const uint8_t* end = nullptr;
    bool ok = true;

    Reader(const uint8_t* b, size_t n) : base(b), p(b), end(b + n) {}

    size_t remain() const { return static_cast<size_t>(end - p); }
    size_t offset() const { return static_cast<size_t>(p - base); }

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
    Vec3 readVec3() { return {read<float>(), read<float>(), read<float>()}; }

    void seekAbs(size_t off) {
        if (off > static_cast<size_t>(end - base)) {
            ok = false;
            return;
        }
        p = base + off;
    }
};

void appendCol1(Reader& r, CollisionMesh& mesh, const LoaderOptions& opt) {
    // After fourcc+size+name+modelId+bounds (already consumed by caller except body).
    // COL1 body starts at: numSpheres ...
    const uint32_t nSph = r.read<uint32_t>();
    for (uint32_t i = 0; i < nSph; ++i) {
        Vec3 c = r.readVec3();
        float rad = r.read<float>();
        r.read<uint32_t>(); // TSurface
        AppendSphere(mesh, c, rad, {}, {0, 0, 0, 1}, opt.sphereSubdiv, 0);
    }
    r.read<uint32_t>(); // unknown
    const uint32_t nBox = r.read<uint32_t>();
    for (uint32_t i = 0; i < nBox; ++i) {
        Vec3 mn = r.readVec3();
        Vec3 mx = r.readVec3();
        r.read<uint32_t>(); // TSurface
        Vec3 center = (mn + mx) * 0.5f;
        Vec3 he = (mx - mn) * 0.5f;
        AppendBox(mesh, center, he, {}, {0, 0, 0, 1}, 0);
    }
    const uint32_t nVert = r.read<uint32_t>();
    std::vector<Vec3> verts(nVert);
    for (uint32_t i = 0; i < nVert; ++i) verts[i] = r.readVec3();
    const uint32_t nFace = r.read<uint32_t>();
    for (uint32_t i = 0; i < nFace; ++i) {
        uint32_t a = r.read<uint32_t>();
        uint32_t b = r.read<uint32_t>();
        uint32_t c = r.read<uint32_t>();
        r.read<uint32_t>(); // TSurface
        if (a < nVert && b < nVert && c < nVert) {
            AppendTriangle(mesh, verts[a], verts[b], verts[c], {}, {0, 0, 0, 1}, 0);
        }
    }
}

void appendCol23(Reader& r, const uint8_t* chunk, size_t chunkSize, int version,
                 CollisionMesh& mesh, const LoaderOptions& opt) {
    // chunk points at fourcc. Offsets in header are relative to fourcc.
    // We are currently sitting just after TBounds (offset 0x2C from fourcc).
    const uint16_t nSph = r.read<uint16_t>();
    const uint16_t nBox = r.read<uint16_t>();
    const uint16_t nFace = r.read<uint16_t>();
    r.read<uint8_t>(); // lines
    r.read<uint8_t>(); // pad
    const uint32_t flags = r.read<uint32_t>();
    const uint32_t offSph = r.read<uint32_t>();
    const uint32_t offBox = r.read<uint32_t>();
    r.read<uint32_t>(); // lines offset
    const uint32_t offVert = r.read<uint32_t>();
    const uint32_t offFace = r.read<uint32_t>();
    r.read<uint32_t>(); // planes
    (void)flags;
    (void)version;
    (void)chunkSize;

    auto at = [&](uint32_t off) -> Reader {
        return Reader(chunk + off, chunkSize > off ? chunkSize - off : 0);
    };

    if (nSph && offSph) {
        Reader s = at(offSph);
        for (uint16_t i = 0; i < nSph; ++i) {
            Vec3 c = s.readVec3();
            float rad = s.read<float>();
            s.read<uint32_t>();
            if (s.ok) AppendSphere(mesh, c, rad, {}, {0, 0, 0, 1}, opt.sphereSubdiv, 0);
        }
    }
    if (nBox && offBox) {
        Reader b = at(offBox);
        for (uint16_t i = 0; i < nBox; ++i) {
            Vec3 mn = b.readVec3();
            Vec3 mx = b.readVec3();
            b.read<uint32_t>();
            if (b.ok) {
                AppendBox(mesh, (mn + mx) * 0.5f, (mx - mn) * 0.5f, {}, {0, 0, 0, 1}, 0);
            }
        }
    }
    if (nFace && offFace && offVert) {
        // Vertices are int16 / 128. Count inferred from max index.
        Reader f = at(offFace);
        struct Face { uint16_t a, b, c; uint8_t mat, light; };
        std::vector<Face> faces(nFace);
        uint16_t maxIdx = 0;
        for (uint16_t i = 0; i < nFace; ++i) {
            faces[i].a = f.read<uint16_t>();
            faces[i].b = f.read<uint16_t>();
            faces[i].c = f.read<uint16_t>();
            faces[i].mat = f.read<uint8_t>();
            faces[i].light = f.read<uint8_t>();
            maxIdx = std::max(maxIdx, std::max(faces[i].a, std::max(faces[i].b, faces[i].c)));
        }
        const uint32_t nVert = static_cast<uint32_t>(maxIdx) + 1;
        Reader v = at(offVert);
        std::vector<Vec3> verts(nVert);
        for (uint32_t i = 0; i < nVert; ++i) {
            int16_t x = v.read<int16_t>();
            int16_t y = v.read<int16_t>();
            int16_t z = v.read<int16_t>();
            verts[i] = {x / 128.f, y / 128.f, z / 128.f};
        }
        if (v.ok && f.ok) {
            for (const auto& fc : faces) {
                if (fc.a < nVert && fc.b < nVert && fc.c < nVert) {
                    AppendTriangle(mesh, verts[fc.a], verts[fc.b], verts[fc.c],
                                   {}, {0, 0, 0, 1}, 0);
                }
            }
        }
    }
}

} // namespace

CollisionMesh LoadFromCol(const std::string& path, const LoaderOptions& opt) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        WQS_ERROR("LoadFromCol: cannot open %s", path.c_str());
        return {};
    }
    in.seekg(0, std::ios::end);
    const std::streamoff n = in.tellg();
    in.seekg(0, std::ios::beg);
    std::vector<uint8_t> buf(static_cast<size_t>(n));
    in.read(reinterpret_cast<char*>(buf.data()), n);

    CollisionMesh mesh;
    size_t off = 0;
    int chunks = 0;
    while (off + 8 <= buf.size()) {
        char fourcc[4];
        std::memcpy(fourcc, buf.data() + off, 4);
        uint32_t size = 0;
        std::memcpy(&size, buf.data() + off + 4, 4);
        // size is the rest of the chunk excluding fourcc (includes the size field itself).
        const size_t chunkTotal = static_cast<size_t>(size) + 4;
        if (chunkTotal < 8 || off + chunkTotal > buf.size()) {
            WQS_WARN("COL chunk at %zu has invalid size %u", off, size);
            break;
        }
        const uint8_t* chunk = buf.data() + off;
        Reader r(chunk, chunkTotal);
        r.read<uint32_t>(); // fourcc
        r.read<uint32_t>(); // size
        char name[22];
        for (int i = 0; i < 22; ++i) name[i] = static_cast<char>(r.read<uint8_t>());
        r.read<int16_t>(); // model id
        (void)name;

        int version = 0;
        if (std::memcmp(fourcc, "COLL", 4) == 0) version = 1;
        else if (std::memcmp(fourcc, "COL2", 4) == 0) version = 2;
        else if (std::memcmp(fourcc, "COL3", 4) == 0) version = 3;
        else if (std::memcmp(fourcc, "COL4", 4) == 0) version = 4;
        else {
            WQS_WARN("Unknown COL fourcc at %zu", off);
            break;
        }

        if (version == 1) {
            // TBounds: radius, center, min, max
            r.read<float>();
            r.readVec3();
            r.readVec3();
            r.readVec3();
            appendCol1(r, mesh, opt);
        } else {
            // TBounds: min, max, center, radius
            r.readVec3();
            r.readVec3();
            r.readVec3();
            r.read<float>();
            appendCol23(r, chunk, chunkTotal, version, mesh, opt);
        }
        ++chunks;
        off += chunkTotal;
        // COL chunks are often 4-byte aligned.
        if (off % 4) off += 4 - (off % 4);
    }
    WQS_INFO("COL %s: %d chunks, %u tris", path.c_str(), chunks, mesh.triangleCount());
    return mesh;
}

} // namespace wqs
