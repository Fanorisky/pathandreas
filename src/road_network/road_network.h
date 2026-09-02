#pragma once

#include "common/vec3.h"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace wqs {

// Path node flag bits, from the GTAMods "Paths (GTA SA)" format description.
// Only the bits this service acts on are named; the rest (spawn probability,
// traffic level, roadblock hints) are kept in RoadNodeInfo::flags so a
// consumer can read them without the service having to understand them.
namespace SaFlags {
constexpr uint32_t kLinkCountMask   = 0x0Fu;      // bits 0-3
constexpr uint32_t kTrafficLevelBits = 0x30u;     // bits 4-5: 0 full .. 3 low
constexpr uint32_t kRoadBlock  = 1u << 6;
constexpr uint32_t kBoat       = 1u << 7;
constexpr uint32_t kEmergency  = 1u << 8;         // emergency vehicles only
constexpr uint32_t kNotHighway = 1u << 12;
constexpr uint32_t kHighway    = 1u << 13;
constexpr uint32_t kSpawnBits  = 0xF0000u;        // bits 16-19
constexpr uint32_t kParking    = 1u << 21;
} // namespace SaFlags

// Per-node data carried by the SA path files. All zero when the network came
// from GPS.dat, which stores positions and connectivity and nothing else.
struct RoadNodeInfo {
    uint32_t flags = 0;
    // Path width in 1/8 world units, 0 when unset. Set on only ~4% of vehicle
    // nodes in the stock files (so it is not a usable road width), but on
    // every pedestrian node - where the wiki leaves its meaning unconfirmed.
    uint8_t width8 = 0;
    // 1 = car, 2 = boat, 3+ = race/mission paths. Always 0 for pedestrian
    // nodes: that byte holds a per-area id for them, not a type.
    uint8_t type = 0;
};

// Which nodes and directions a route may use. A default-constructed profile
// filters nothing, which is what diagnostics want; the named factories below
// are the ones queries should use.
struct RouteProfile {
    // Reject a directed edge with no lane in the direction of travel. That is
    // how SA records one-way streets - the link graph itself is undirected.
    bool respectOneWay = false;
    bool allowEmergency = true;    // nodes flagged emergency-vehicles-only
    bool allowBoat = true;
    // 0 = accept any node type; otherwise only nodes of exactly this type,
    // which keeps a car off the boat network and vice versa.
    uint8_t requireType = 0;
    // Multiplier applied to edges leaving a highway node. Below 1 makes long
    // trips prefer the freeway the way the game's own traffic does; the
    // heuristic is scaled to match so A* stays admissible.
    float highwayCost = 1.0f;
    // Car(): civilian traffic - one-way respected, boats excluded, freeway
    // preferred. Boat(): the boat network only. Ped(): no vehicle rule at all.

    static RouteProfile Car();
    static RouteProfile Boat();
    static RouteProfile Ped();
};

// A GTA SA path node network. One instance is the vehicle graph, another the
// pedestrian graph; both are the same structure, and the profile decides what
// a given query may traverse.
class RoadNetwork {
public:
    // --- loading ---------------------------------------------------------
    // GPS.dat text format (samp-gps-plugin): "0 x y z ignore id" nodes and
    // "1 src dst dir" connections, dir 2 meaning no link. Kept because it
    // needs no game install; carries no pedestrian graph and no flags.
    bool loadFile(const std::string& path, std::string& err);

    // --- construction, used by the loaders -------------------------------
    void clear();
    void reserveNodes(size_t n);
    int addNode(const Vec3& pos, const RoadNodeInfo& info = {});
    // `lanes` is the number of lanes in this direction of travel; 0 means
    // either "no lane data" or "one-way against this direction" - the two are
    // distinguished by hasLaneData, set when any edge carried navi data.
    void addEdge(int from, int to, float len, uint8_t lanes = 0);
    void setHasLaneData(bool v) { hasLaneData_ = v; }
    // Builds the spatial index. Must be called once after the last addNode.
    void finishBuild();

    // --- queries ---------------------------------------------------------
    bool ready() const { return !nodes_.empty(); }
    bool hasLaneData() const { return hasLaneData_; }
    long nodeCount() const { return static_cast<long>(nodes_.size()); }
    long connectionCount() const { return connections_; }
    Vec3 nodePos(long index) const { return nodes_[static_cast<size_t>(index)].pos; }
    const RoadNodeInfo& nodeInfo(long index) const {
        return nodes_[static_cast<size_t>(index)].info;
    }

    // Read-only adjacency, for drawing the graph and for diagnostics. Edges are
    // directed: a one-way street appears from one end only, with lanes 0 on the
    // side that may not be driven.
    long degree(long node) const {
        return static_cast<long>(nodes_[static_cast<size_t>(node)].adj.size());
    }
    long neighbour(long node, long k) const {
        return nodes_[static_cast<size_t>(node)].adj[static_cast<size_t>(k)].to;
    }
    uint8_t neighbourLanes(long node, long k) const {
        return nodes_[static_cast<size_t>(node)].adj[static_cast<size_t>(k)].lanes;
    }

    // Node indices whose position falls inside an XY rectangle, capped at
    // `limit` (the caller is drawing a viewport, not the whole state). Uses the
    // same grid as nearestNode, so it costs the overlapped cells, not a scan.
    // Returns true when the cap cut the result short.
    bool nodesInRect(float minX, float minY, float maxX, float maxY, long limit,
                     std::vector<long>& out) const;

    // Nearest node the profile accepts, -1 if there is none. Grid-hashed,
    // with a linear fallback for positions far from any node.
    long nearestNode(const Vec3& pos, const RouteProfile& profile = {}) const;

    struct RouteResult {
        bool success = false;
        // Node positions in GTA coords, start to goal. The first and last are
        // the nodes nearest `from`/`to`, not those positions themselves.
        std::vector<Vec3> waypoints;
        // Lanes available on the leg leaving waypoint i, so size() is
        // waypoints.size() - 1. Zero where the network has no lane data.
        std::vector<uint8_t> lanes;
        // Flags of each waypoint's node, parallel to waypoints.
        std::vector<uint32_t> flags;
    };
    RouteResult findPath(const Vec3& from, const Vec3& to,
                         const RouteProfile& profile = {}) const;

    // Which connected component a node belongs to, over the undirected and
    // unfiltered view of the graph, or -1 for an invalid index. Labelled once
    // by finishBuild(). This is what lets a route tell "can I walk from here
    // to there at all" apart from "is it far" without running a search: the
    // pedestrian graph has 179 components and answering that per corridor
    // point is how a walk hands over between the sidewalk network and the road
    // network.
    int componentId(long node) const {
        return (node < 0 || node >= static_cast<long>(component_.size()))
                   ? -1 : component_[static_cast<size_t>(node)];
    }
    long componentCount() const { return componentCount_; }

    // Connected-component sizes over the undirected view, largest first.
    // Diagnostic: it is how the three-city split of the pedestrian graph and
    // the separate boat network were found.
    std::vector<long> componentSizes(const RouteProfile& profile = {}) const;

private:
    struct Edge {
        int to = 0;
        float len = 0.f;
        uint8_t lanes = 0;
    };
    struct Node {
        Vec3 pos{};
        RoadNodeInfo info{};
        std::vector<Edge> adj;
    };
    bool accepts(const Node& n, const RouteProfile& p) const;
    float costMul(const Node& n, const RouteProfile& p) const;

    void labelComponents();

    std::vector<Node> nodes_;
    std::unordered_map<long, int> idToIndex_; // GPS.dat node id -> dense index
    long connections_ = 0;
    bool hasLaneData_ = false;
    std::vector<int> component_;   // per node, filled by finishBuild()
    long componentCount_ = 0;

    static constexpr float kCell = 200.f;
    std::unordered_map<uint64_t, std::vector<int>> grid_;
    static uint64_t cellKey(int cx, int cy) {
        return (static_cast<uint64_t>(static_cast<uint32_t>(cx)) << 32) |
               static_cast<uint32_t>(cy);
    }
};

} // namespace wqs
