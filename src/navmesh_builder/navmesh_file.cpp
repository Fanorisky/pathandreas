#include "navmesh_builder/navmesh_file.h"
#include "common/log.h"

#include "DetourNavMesh.h"

#include <fstream>
#include <cstring>
#include <vector>

namespace wqs {

bool SaveNavMesh(const std::string& path, const dtNavMesh* mesh, std::string& err) {
    if (!mesh) {
        err = "null navmesh";
        return false;
    }
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        err = "cannot write " + path;
        return false;
    }
    const dtNavMeshParams* params = mesh->getParams();
    out.write("WQS1", 4);
    uint32_t version = 1;
    out.write(reinterpret_cast<const char*>(&version), 4);
    out.write(reinterpret_cast<const char*>(params), sizeof(dtNavMeshParams));

    // Count tiles.
    uint32_t tileCount = 0;
    for (int i = 0; i < mesh->getMaxTiles(); ++i) {
        const dtMeshTile* tile = mesh->getTile(i);
        if (tile && tile->header && tile->dataSize) ++tileCount;
    }
    out.write(reinterpret_cast<const char*>(&tileCount), 4);

    for (int i = 0; i < mesh->getMaxTiles(); ++i) {
        const dtMeshTile* tile = mesh->getTile(i);
        if (!tile || !tile->header || !tile->dataSize) continue;
        int32_t tx = tile->header->x;
        int32_t ty = tile->header->y;
        int32_t sz = tile->dataSize;
        out.write(reinterpret_cast<const char*>(&tx), 4);
        out.write(reinterpret_cast<const char*>(&ty), 4);
        out.write(reinterpret_cast<const char*>(&sz), 4);
        out.write(reinterpret_cast<const char*>(tile->data), sz);
    }
    if (!out) {
        err = "navmesh write failed";
        return false;
    }
    WQS_INFO("Wrote %u tiles to %s", tileCount, path.c_str());
    return true;
}

dtNavMesh* LoadNavMesh(const std::string& path, std::string& err) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        err = "cannot open " + path;
        return nullptr;
    }
    char magic[4];
    in.read(magic, 4);
    if (std::memcmp(magic, "WQS1", 4) != 0) {
        err = "not a WQS1 navmesh file";
        return nullptr;
    }
    uint32_t version = 0;
    in.read(reinterpret_cast<char*>(&version), 4);
    if (version != 1) {
        err = "unsupported navmesh version";
        return nullptr;
    }
    dtNavMeshParams params{};
    in.read(reinterpret_cast<char*>(&params), sizeof(params));
    uint32_t tileCount = 0;
    in.read(reinterpret_cast<char*>(&tileCount), 4);
    if (!in) {
        err = "truncated navmesh header";
        return nullptr;
    }

    dtNavMesh* mesh = dtAllocNavMesh();
    if (!mesh || dtStatusFailed(mesh->init(&params))) {
        err = "dtNavMesh::init failed";
        dtFreeNavMesh(mesh);
        return nullptr;
    }
    for (uint32_t i = 0; i < tileCount; ++i) {
        int32_t tx = 0, ty = 0, sz = 0;
        in.read(reinterpret_cast<char*>(&tx), 4);
        in.read(reinterpret_cast<char*>(&ty), 4);
        in.read(reinterpret_cast<char*>(&sz), 4);
        if (!in || sz <= 0) {
            err = "truncated tile header";
            dtFreeNavMesh(mesh);
            return nullptr;
        }
        unsigned char* data = static_cast<unsigned char*>(dtAlloc(sz, DT_ALLOC_PERM));
        in.read(reinterpret_cast<char*>(data), sz);
        if (!in) {
            dtFree(data);
            err = "truncated tile body";
            dtFreeNavMesh(mesh);
            return nullptr;
        }
        dtStatus st = mesh->addTile(data, sz, DT_TILE_FREE_DATA, 0, nullptr);
        if (dtStatusFailed(st)) {
            dtFree(data);
            WQS_WARN("addTile failed for tile %d,%d", tx, ty);
        }
    }
    WQS_INFO("Loaded %u navmesh tiles from %s", tileCount, path.c_str());
    return mesh;
}

} // namespace wqs
