#include "road_network/road_network.h"
#include "common/log.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>

namespace wqs {

bool RoadNetwork::loadFile(const std::string& path, std::string& err) {
    nodes_.clear();
    idToIndex_.clear();
    grid_.clear();
    connections_ = 0;

    std::ifstream in(path);
    if (!in) {
        err = "cannot open " + path;
        return false;
    }

    // First pass: nodes ("0 x y z ignore id"), second pass over the same
    // buffer for connections ("1 source target dir") so that connection
    // endpoints can be resolved to dense indices as they are read.
    std::vector<std::tuple<long, long, int>> pending;
    std::string line;
    long lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        std::istringstream ss(line);
        int type = 0;
        if (!(ss >> type)) continue; // blank line
        if (type == 0) {
            float x = 0, y = 0, z = 0;
            long ignore = 0, id = 0;
            if (!(ss >> x >> y >> z >> ignore >> id)) {
                WQS_WARN("road network %s:%ld: malformed node line, skipped", path.c_str(), lineNo);
                continue;
            }
            if (idToIndex_.count(id)) continue; // duplicate id: first wins
            Node n;
            n.pos = {x, y, z};
            idToIndex_[id] = static_cast<int>(nodes_.size());
            nodes_.push_back(n);
        } else if (type == 1) {
            long a = 0, b = 0, dir = 0;
            if (!(ss >> a >> b >> dir)) {
                WQS_WARN("road network %s:%ld: malformed connection line, skipped", path.c_str(), lineNo);
                continue;
            }
            if (dir == 2) continue; // explicitly no link
            pending.emplace_back(a, b, 0);
        }
        // Unknown line types are ignored (the shipped GPS.dat has none).
    }

    for (const auto& [a, b, ign] : pending) {
        const auto ia = idToIndex_.find(a);
        const auto ib = idToIndex_.find(b);
        if (ia == idToIndex_.end() || ib == idToIndex_.end()) {
            WQS_WARN("road network %s: connection %ld->%ld references unknown node, skipped",
                     path.c_str(), a, b);
            continue;
        }
        const float len = (nodes_[ib->second].pos - nodes_[ia->second].pos).length();
        nodes_[ia->second].adj.emplace_back(ib->second, len);
        ++connections_;
    }

    // Spatial grid for nearest-node.
    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        const int cx = static_cast<int>(std::floor(nodes_[i].pos.x / kCell));
        const int cy = static_cast<int>(std::floor(nodes_[i].pos.y / kCell));
        grid_[cellKey(cx, cy)].push_back(i);
    }

    WQS_INFO("Road network: %ld nodes, %ld connections from %s",
             static_cast<long>(nodes_.size()), connections_, path.c_str());
    return !nodes_.empty();
}

long RoadNetwork::nearestNode(const Vec3& pos) const {
    if (nodes_.empty()) return -1;
    const int cx = static_cast<int>(std::floor(pos.x / kCell));
    const int cy = static_cast<int>(std::floor(pos.y / kCell));
    // Expanding ring search over the grid; roads cover the whole map, so a
    // couple of rings is the common case.
    for (int r = 0; r < 16; ++r) {
        long best = -1;
        float bestD = std::numeric_limits<float>::max();
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                if (std::max(std::abs(dx), std::abs(dy)) != r) continue; // ring only
                const auto it = grid_.find(cellKey(cx + dx, cy + dy));
                if (it == grid_.end()) continue;
                for (const int idx : it->second) {
                    const float d = (nodes_[idx].pos - pos).lengthSq();
                    if (d < bestD) { bestD = d; best = idx; }
                }
            }
        }
        if (best >= 0) return best;
    }
    // Nothing within 16 cells (~3.2km): fall back to a full scan. Positions
    // this far from any road happen (deep sea, mountains) and callers deserve
    // a real answer rather than a silent failure.
    long best = -1;
    float bestD = std::numeric_limits<float>::max();
    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        const float d = (nodes_[i].pos - pos).lengthSq();
        if (d < bestD) { bestD = d; best = i; }
    }
    return best;
}

RoadNetwork::RouteResult RoadNetwork::findPath(const Vec3& from, const Vec3& to) const {
    RouteResult out;
    const long s = nearestNode(from);
    const long g = nearestNode(to);
    if (s < 0 || g < 0) return out;
    const int start = static_cast<int>(s), goal = static_cast<int>(g);
    if (start == goal) {
        out.waypoints.push_back(nodes_[start].pos);
        out.success = true;
        return out;
    }

    const int n = static_cast<int>(nodes_.size());
    constexpr float kInf = std::numeric_limits<float>::max();
    std::vector<float> gScore(n, kInf);
    std::vector<int> cameFrom(n, -1);
    // (f, index); ties broken by index for determinism.
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>,
                        std::greater<std::pair<float, int>>> open;
    gScore[start] = 0.f;
    open.push({(nodes_[goal].pos - nodes_[start].pos).length(), start});

    while (!open.empty()) {
        const auto [f, cur] = open.top();
        open.pop();
        if (cur == goal) break;
        // Stale queue entry (a better path to `cur` was found after this one
        // was pushed) - skip instead of re-relaxing its neighbours.
        const float gc = gScore[cur];
        if (f > gc + (nodes_[goal].pos - nodes_[cur].pos).length() + 1e-4f) continue;
        for (const auto& [next, len] : nodes_[cur].adj) {
            const float cand = gc + len;
            if (cand < gScore[next] - 1e-4f) {
                gScore[next] = cand;
                cameFrom[next] = cur;
                open.push({cand + (nodes_[goal].pos - nodes_[next].pos).length(), next});
            }
        }
    }

    if (cameFrom[goal] < 0 && goal != start) return out; // unreachable

    std::vector<Vec3> reversed;
    for (int at = goal; at >= 0; at = cameFrom[at]) {
        reversed.push_back(nodes_[at].pos);
        if (at == start) break;
    }
    std::reverse(reversed.begin(), reversed.end());
    out.waypoints = std::move(reversed);
    out.success = true;
    return out;
}

} // namespace wqs
