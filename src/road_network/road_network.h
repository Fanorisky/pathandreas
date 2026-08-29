#pragma once

#include "common/vec3.h"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace wqs {

// The GTA SA traffic node graph - the road network the game's own vehicle AI
// drives on. Loaded from the GPS.dat format distributed with the samp-gps
// plugin (the format only; this loader is original - the plugin itself is
// unlicensed and none of its code is used):
//   "0 x y z ignore id"     - a node
//   "1 source target dir"   - a one-directional connection; dir 2 = none
//
// This is macro-scale road routing: ~27k nodes covering every drivable road
// including the inter-city bridges, which makes it the right backend for
// vehicles where the pedestrian navmesh is the right backend for people.
class RoadNetwork {
public:
    bool loadFile(const std::string& path, std::string& err);
    bool ready() const { return !nodes_.empty(); }
    long nodeCount() const { return static_cast<long>(nodes_.size()); }
    long connectionCount() const { return connections_; }

    // Position of a node index returned by nearestNode (undefined if invalid).
    Vec3 nodePos(long index) const { return nodes_[static_cast<size_t>(index)].pos; }

    // Nearest node to a world position, -1 if the graph is empty.
    // Grid-hashed; falls back to a linear scan only for positions far from
    // any road (middle of the sea, interior pockets).
    long nearestNode(const Vec3& pos) const;

    struct RouteResult {
        bool success = false;
        // Node positions in GTA coords, ordered start to goal. The first and
        // last entries are the nodes nearest to `from`/`to` - callers that
        // need exact endpoints should keep their own positions.
        std::vector<Vec3> waypoints;
    };
    // A* over the node graph with euclidean edge costs and heuristic.
    RouteResult findPath(const Vec3& from, const Vec3& to) const;

private:
    struct Node {
        Vec3 pos{};
        std::vector<std::pair<int, float>> adj; // (dense index, edge length)
    };
    std::vector<Node> nodes_;
    std::unordered_map<long, int> idToIndex_; // original node id -> dense index
    long connections_ = 0;

    // Uniform grid over node positions for nearest-node queries.
    static constexpr float kCell = 200.f;
    std::unordered_map<uint64_t, std::vector<int>> grid_;
    static uint64_t cellKey(int cx, int cy) {
        return (static_cast<uint64_t>(static_cast<uint32_t>(cx)) << 32) |
               static_cast<uint32_t>(cy);
    }
};

} // namespace wqs
