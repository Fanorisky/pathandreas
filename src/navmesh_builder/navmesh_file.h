#pragma once

#include <string>
#include <vector>
#include <cstdint>

class dtNavMesh;

namespace wqs {

// Custom tiled navmesh container. Magic "WQS1".
//   uint32 magic 'WQS1'
//   uint32 version = 1
//   dtNavMeshParams (orig[3], tileWidth, tileHeight, maxTiles, maxPolys)
//   uint32 tileCount
//   per tile: int32 tileX, tileY, int32 dataSize, bytes[dataSize]
// Tile bytes are the Detour blob from dtCreateNavMeshData.

struct NavMeshFileHeader {
    char magic[4];
    uint32_t version;
};

bool SaveNavMesh(const std::string& path, const dtNavMesh* mesh, std::string& err);
dtNavMesh* LoadNavMesh(const std::string& path, std::string& err);

} // namespace wqs
