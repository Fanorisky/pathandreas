// navmesh_components.cpp
// Connectivity report for a WQS1 navmesh file: connected-component census of
// the polygon graph plus per-component centroids. Use it to sanity-check a
// bake - a healthy full-map bake has one dominant component (the mainland
// road network) and small fragments only for interiors, rooftops and islands.
//
// The graph is built from two edge kinds:
//   - poly->neis[] entries without DT_EXT_LINK (same-tile adjacency), and
//   - tile->links[] entries whose target resolves in a DIFFERENT tile
//     (cross-tile stitches). Same-tile links[] entries are just the
//     adjacency above stored as links and are skipped to avoid double counts.

#include "DetourNavMesh.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <vector>

namespace {

dtNavMesh* loadWqs1(const char* path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return nullptr;
    char magic[4];
    in.read(magic, 4);
    if (std::memcmp(magic, "WQS1", 4) != 0) return nullptr;
    uint32_t version = 0;
    in.read(reinterpret_cast<char*>(&version), 4);
    dtNavMeshParams params{};
    in.read(reinterpret_cast<char*>(&params), sizeof(params));
    uint32_t tileCount = 0;
    in.read(reinterpret_cast<char*>(&tileCount), 4);
    dtNavMesh* mesh = dtAllocNavMesh();
    if (!mesh || dtStatusFailed(mesh->init(&params))) return nullptr;
    for (uint32_t i = 0; i < tileCount; ++i) {
        int32_t tx = 0, ty = 0, sz = 0;
        in.read(reinterpret_cast<char*>(&tx), 4);
        in.read(reinterpret_cast<char*>(&ty), 4);
        in.read(reinterpret_cast<char*>(&sz), 4);
        if (!in || sz <= 0) return nullptr;
        unsigned char* data = static_cast<unsigned char*>(dtAlloc(sz, DT_ALLOC_PERM));
        in.read(reinterpret_cast<char*>(data), sz);
        if (!in || mesh->addTile(data, sz, DT_TILE_FREE_DATA, 0, nullptr) == DT_FAILURE)
            dtFree(data);
    }
    return mesh;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file.navmesh>\n", argv[0]);
        return 2;
    }
    dtNavMesh* mesh = loadWqs1(argv[1]);
    if (!mesh) {
        std::fprintf(stderr, "failed to load %s\n", argv[1]);
        return 1;
    }

    // Enumerate all poly refs through the public tile lookup.
    std::vector<dtPolyRef> refs;
    for (int tx = 0; tx < 64; ++tx) {
        for (int ty = 0; ty < 64; ++ty) {
            const dtPolyRef tref = mesh->getTileRefAt(tx, ty, 0);
            if (!tref) continue;
            const dtMeshTile* t = nullptr;
            const dtPoly* poly = nullptr;
            if (dtStatusFailed(mesh->getTileAndPolyByRef(tref, &t, &poly)) || !t) continue;
            const unsigned int salt = mesh->decodePolyIdSalt(tref);
            const unsigned int it = mesh->decodePolyIdTile(tref);
            for (int p = 0; p < t->header->polyCount; ++p)
                refs.push_back(mesh->encodePolyId(salt, it, static_cast<unsigned int>(p)));
        }
    }
    const size_t n = refs.size();

    std::map<dtPolyRef, size_t> idx;
    for (size_t i = 0; i < n; ++i) idx[refs[i]] = i;

    std::vector<size_t> parent(n);
    for (size_t i = 0; i < n; ++i) parent[i] = i;
    auto find = [&](size_t x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };
    auto unite = [&](size_t a, size_t b) {
        a = find(a); b = find(b);
        if (a != b) parent[b] = a;
    };

    long internalEdges = 0, crossTileLinks = 0;
    long tilesSeen = 0;
    for (size_t i = 0; i < n; ++i) {
        const dtMeshTile* t = nullptr;
        const dtPoly* poly = nullptr;
        if (dtStatusFailed(mesh->getTileAndPolyByRef(refs[i], &t, &poly)) || !t) continue;
        const dtPolyRef tref = mesh->getTileRef(t);
        const unsigned int salt = mesh->decodePolyIdSalt(tref);
        const unsigned int it = mesh->decodePolyIdTile(tref);

        // Same-tile adjacency from poly->neis.
        for (int j = 0; j < poly->vertCount; ++j) {
            const unsigned int nei = poly->neis[j];
            if ((nei & DT_EXT_LINK) || nei == 0) continue;
            auto e = idx.find(mesh->encodePolyId(salt, it, nei - 1));
            if (e != idx.end()) { unite(i, e->second); ++internalEdges; }
        }
        // Links: only count (and union) those crossing into another tile.
        for (unsigned int l = poly->firstLink; l != DT_NULL_LINK; l = t->links[l].next) {
            auto e = idx.find(t->links[l].ref);
            if (e == idx.end()) continue;
            const dtMeshTile* t2 = nullptr;
            const dtPoly* p2 = nullptr;
            if (dtStatusFailed(mesh->getTileAndPolyByRef(t->links[l].ref, &t2, &p2)) || t2 == t) continue;
            unite(i, e->second);
            ++crossTileLinks;
        }
    }

    // Per-component stats: size and centroid (GTA coords).
    struct Stat { long count = 0; double cx = 0, cy = 0, cz = 0; };
    std::map<size_t, Stat> st;
    for (size_t i = 0; i < n; ++i) {
        const dtMeshTile* t = nullptr;
        const dtPoly* poly = nullptr;
        if (dtStatusFailed(mesh->getTileAndPolyByRef(refs[i], &t, &poly)) || !t) continue;
        float cx = 0, cy = 0, cz = 0;
        for (int j = 0; j < poly->vertCount; ++j) {
            const float* v = &t->verts[poly->verts[j] * 3];
            cx += v[0]; cy += -v[2]; cz += v[1];
        }
        // Poly centroid, not vertex sum - forgetting the division inflated
        // component centroids by the average vertex count (~4x) and made
        // ordinary city fragments look like far-away stray geometry.
        cx /= poly->vertCount; cy /= poly->vertCount; cz /= poly->vertCount;
        Stat& s = st[find(i)];
        s.count++; s.cx += cx; s.cy += cy; s.cz += cz;
    }
    for (int tx = 0; tx < 64; ++tx)
        for (int ty = 0; ty < 64; ++ty)
            if (mesh->getTileRefAt(tx, ty, 0)) ++tilesSeen;

    std::vector<std::pair<long, size_t>> bySize;
    for (auto& kv : st) bySize.push_back({kv.second.count, kv.first});
    std::sort(bySize.rbegin(), bySize.rend());

    long c1 = 0, c2 = 0, c3 = 0, c4 = 0, c5 = 0;
    for (auto& kv : bySize) {
        if (kv.first == 1) ++c1;
        else if (kv.first <= 10) ++c2;
        else if (kv.first <= 100) ++c3;
        else if (kv.first <= 1000) ++c4;
        else ++c5;
    }

    std::printf("tiles: %ld  polys: %zu\n", tilesSeen, n);
    std::printf("edges: internal=%ld cross-tile=%ld\n", internalEdges, crossTileLinks);
    std::printf("components: %zu  (1 poly: %ld, 2-10: %ld, 11-100: %ld, 101-1000: %ld, >1000: %ld)\n",
                st.size(), c1, c2, c3, c4, c5);
    const long biggest = bySize.empty() ? 0 : bySize[0].first;
    std::printf("biggest component: %ld polys (%.1f%% of total)\n",
                biggest, n ? 100.0 * biggest / static_cast<double>(n) : 0.0);
    std::printf("\ntop 10 components:\n");
    for (size_t i = 0; i < 10 && i < bySize.size(); ++i) {
        const Stat& s = st[bySize[i].second];
        std::printf("  %6ld polys  centroid=(%7.0f, %7.0f, z=%6.1f)\n",
                    s.count, s.cx / s.count, s.cy / s.count, s.cz / s.count);
    }
    return 0;
}
