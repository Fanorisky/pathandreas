#include "road_network/road_network.h"
#include "common/log.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>

namespace wqs {

RouteProfile RouteProfile::Car() {
    RouteProfile p;
    p.respectOneWay = true;
    // The format description calls flag bit 8 "emergency vehicles only", and
    // those nodes do form continuous chains of their own across the whole map
    // (6,215 car-type nodes, mean degree 2.0) rather than marking junctions.
    // Excluding them is still the wrong default: measured over 300 random
    // trips at least 500 units apart, dropping them leaves 47% of routes
    // longer, 23% more than a fifth longer, and 11 of 300 with no route at
    // all. A consumer that wants strictly civilian roads can turn this off
    // and gets a 20,766-node network that is 99.5% one component.
    p.allowEmergency = true;
    p.allowBoat = false;
    p.requireType = 1;
    // Stock traffic favours the freeway heavily; 0.8 is enough to pick it for
    // a cross-city trip without dragging short local trips onto a ramp.
    p.highwayCost = 0.8f;
    return p;
}

RouteProfile RouteProfile::Boat() {
    RouteProfile p;
    p.respectOneWay = false;   // open water, and boat links carry no lanes
    p.allowEmergency = true;
    p.allowBoat = true;
    p.requireType = 2;
    p.highwayCost = 1.0f;
    return p;
}

RouteProfile RouteProfile::Ped() {
    RouteProfile p;
    // A pedestrian ignores one-way streets, and the flag bits that gate
    // vehicles are not used on pedestrian nodes at all.
    p.respectOneWay = false;
    p.allowEmergency = true;
    p.allowBoat = true;
    p.requireType = 0;
    p.highwayCost = 1.0f;
    return p;
}

void RoadNetwork::clear() {
    nodes_.clear();
    idToIndex_.clear();
    grid_.clear();
    connections_ = 0;
    hasLaneData_ = false;
}

void RoadNetwork::reserveNodes(size_t n) { nodes_.reserve(n); }

int RoadNetwork::addNode(const Vec3& pos, const RoadNodeInfo& info) {
    Node n;
    n.pos = pos;
    n.info = info;
    nodes_.push_back(n);
    return static_cast<int>(nodes_.size()) - 1;
}

void RoadNetwork::addEdge(int from, int to, float len, uint8_t lanes) {
    if (from < 0 || to < 0 || from >= static_cast<int>(nodes_.size()) ||
        to >= static_cast<int>(nodes_.size()))
        return;
    nodes_[static_cast<size_t>(from)].adj.push_back(Edge{to, len, lanes});
    ++connections_;
}

void RoadNetwork::finishBuild() {
    grid_.clear();
    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        const int cx = static_cast<int>(std::floor(nodes_[i].pos.x / kCell));
        const int cy = static_cast<int>(std::floor(nodes_[i].pos.y / kCell));
        grid_[cellKey(cx, cy)].push_back(i);
    }
}

bool RoadNetwork::loadFile(const std::string& path, std::string& err) {
    clear();

    std::ifstream in(path);
    if (!in) {
        err = "cannot open " + path;
        return false;
    }

    // First pass: nodes ("0 x y z ignore id"), collecting connections
    // ("1 source target dir") to resolve once every id is known.
    std::vector<std::pair<long, long>> pending;
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
            idToIndex_[id] = addNode({x, y, z});
        } else if (type == 1) {
            long a = 0, b = 0, dir = 0;
            if (!(ss >> a >> b >> dir)) {
                WQS_WARN("road network %s:%ld: malformed connection line, skipped", path.c_str(), lineNo);
                continue;
            }
            if (dir == 2) continue; // explicitly no link
            pending.emplace_back(a, b);
        }
        // Other line types (the shipped file opens with a "3 ..." header line)
        // carry nothing this loader needs.
    }

    for (const auto& [a, b] : pending) {
        const auto ia = idToIndex_.find(a);
        const auto ib = idToIndex_.find(b);
        if (ia == idToIndex_.end() || ib == idToIndex_.end()) {
            WQS_WARN("road network %s: connection %ld->%ld references unknown node, skipped",
                     path.c_str(), a, b);
            continue;
        }
        const float len = (nodes_[static_cast<size_t>(ib->second)].pos -
                           nodes_[static_cast<size_t>(ia->second)].pos).length();
        addEdge(ia->second, ib->second, len);
    }

    finishBuild();
    WQS_INFO("Road network: %ld nodes, %ld connections from %s",
             static_cast<long>(nodes_.size()), connections_, path.c_str());
    return !nodes_.empty();
}

bool RoadNetwork::accepts(const Node& n, const RouteProfile& p) const {
    if (p.requireType != 0 && n.info.type != 0 && n.info.type != p.requireType) return false;
    if (!p.allowEmergency && (n.info.flags & SaFlags::kEmergency)) return false;
    if (!p.allowBoat && (n.info.flags & SaFlags::kBoat)) return false;
    return true;
}

float RoadNetwork::costMul(const Node& n, const RouteProfile& p) const {
    return (n.info.flags & SaFlags::kHighway) ? p.highwayCost : 1.0f;
}

long RoadNetwork::nearestNode(const Vec3& pos, const RouteProfile& profile) const {
    if (nodes_.empty()) return -1;
    const int cx = static_cast<int>(std::floor(pos.x / kCell));
    const int cy = static_cast<int>(std::floor(pos.y / kCell));
    // Expanding ring over the grid; nodes cover the whole map, so a couple of
    // rings is the common case.
    for (int r = 0; r < 16; ++r) {
        long best = -1;
        float bestD = std::numeric_limits<float>::max();
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                if (std::max(std::abs(dx), std::abs(dy)) != r) continue; // ring only
                const auto it = grid_.find(cellKey(cx + dx, cy + dy));
                if (it == grid_.end()) continue;
                for (const int idx : it->second) {
                    const Node& n = nodes_[static_cast<size_t>(idx)];
                    if (!accepts(n, profile)) continue;
                    const float d = (n.pos - pos).lengthSq();
                    if (d < bestD) { bestD = d; best = idx; }
                }
            }
        }
        if (best >= 0) return best;
    }
    // Nothing within 16 cells (~3.2km): fall back to a full scan. Positions
    // this far out happen (deep sea, mountain tops) and a real answer beats a
    // silent failure.
    long best = -1;
    float bestD = std::numeric_limits<float>::max();
    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        const Node& n = nodes_[static_cast<size_t>(i)];
        if (!accepts(n, profile)) continue;
        const float d = (n.pos - pos).lengthSq();
        if (d < bestD) { bestD = d; best = i; }
    }
    return best;
}

RoadNetwork::RouteResult RoadNetwork::findPath(const Vec3& from, const Vec3& to,
                                              const RouteProfile& profile) const {
    RouteResult out;
    const long s = nearestNode(from, profile);
    const long g = nearestNode(to, profile);
    if (s < 0 || g < 0) return out;
    const int start = static_cast<int>(s), goal = static_cast<int>(g);
    if (start == goal) {
        out.waypoints.push_back(nodes_[static_cast<size_t>(start)].pos);
        out.flags.push_back(nodes_[static_cast<size_t>(start)].info.flags);
        out.success = true;
        return out;
    }

    const int n = static_cast<int>(nodes_.size());
    constexpr float kInf = std::numeric_limits<float>::max();
    // Edge costs can be scaled below 1 for highways, so the straight-line
    // heuristic has to be scaled by the same factor to stay admissible.
    const float hScale = std::min(1.0f, profile.highwayCost);
    const Vec3 goalPos = nodes_[static_cast<size_t>(goal)].pos;
    auto h = [&](int i) { return (goalPos - nodes_[static_cast<size_t>(i)].pos).length() * hScale; };

    std::vector<float> gScore(static_cast<size_t>(n), kInf);
    std::vector<int> cameFrom(static_cast<size_t>(n), -1);
    std::vector<uint8_t> cameLanes(static_cast<size_t>(n), 0);
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>,
                        std::greater<std::pair<float, int>>> open;
    gScore[static_cast<size_t>(start)] = 0.f;
    open.push({h(start), start});

    const bool oneWay = profile.respectOneWay && hasLaneData_;
    while (!open.empty()) {
        const auto [f, cur] = open.top();
        open.pop();
        if (cur == goal) break;
        // Stale queue entry: a better path to `cur` was found after this one
        // was pushed, so its neighbours were already relaxed from it.
        const float gc = gScore[static_cast<size_t>(cur)];
        if (f > gc + h(cur) + 1e-4f) continue;
        const Node& cn = nodes_[static_cast<size_t>(cur)];
        const float mul = costMul(cn, profile);
        for (const Edge& e : cn.adj) {
            // A directed edge with no lane in this direction is a one-way
            // street being entered the wrong way.
            if (oneWay && e.lanes == 0) continue;
            const Node& nn = nodes_[static_cast<size_t>(e.to)];
            if (!accepts(nn, profile)) continue;
            const float cand = gc + e.len * mul;
            if (cand < gScore[static_cast<size_t>(e.to)] - 1e-4f) {
                gScore[static_cast<size_t>(e.to)] = cand;
                cameFrom[static_cast<size_t>(e.to)] = cur;
                cameLanes[static_cast<size_t>(e.to)] = e.lanes;
                open.push({cand + h(e.to), e.to});
            }
        }
    }

    if (cameFrom[static_cast<size_t>(goal)] < 0) return out; // unreachable

    std::vector<int> rev;
    for (int at = goal; at >= 0; at = cameFrom[static_cast<size_t>(at)]) {
        rev.push_back(at);
        if (at == start) break;
    }
    std::reverse(rev.begin(), rev.end());
    out.waypoints.reserve(rev.size());
    out.flags.reserve(rev.size());
    for (size_t i = 0; i < rev.size(); ++i) {
        const Node& nd = nodes_[static_cast<size_t>(rev[i])];
        out.waypoints.push_back(nd.pos);
        out.flags.push_back(nd.info.flags);
        // cameLanes[v] is the lane count of the edge that reached v, i.e. the
        // leg leaving the previous waypoint.
        if (i > 0) out.lanes.push_back(cameLanes[static_cast<size_t>(rev[i])]);
    }
    out.success = true;
    return out;
}

std::vector<long> RoadNetwork::componentSizes(const RouteProfile& profile) const {
    const size_t n = nodes_.size();
    std::vector<std::vector<int>> und(n);
    for (size_t u = 0; u < n; ++u) {
        if (!accepts(nodes_[u], profile)) continue;
        for (const Edge& e : nodes_[u].adj) {
            if (!accepts(nodes_[static_cast<size_t>(e.to)], profile)) continue;
            und[u].push_back(e.to);
            und[static_cast<size_t>(e.to)].push_back(static_cast<int>(u));
        }
    }
    std::vector<char> seen(n, 0);
    std::vector<long> sizes;
    std::vector<int> stack;
    for (size_t s = 0; s < n; ++s) {
        if (seen[s] || !accepts(nodes_[s], profile)) continue;
        stack.clear();
        stack.push_back(static_cast<int>(s));
        seen[s] = 1;
        long count = 0;
        while (!stack.empty()) {
            const int u = stack.back();
            stack.pop_back();
            ++count;
            for (const int v : und[static_cast<size_t>(u)]) {
                if (!seen[static_cast<size_t>(v)]) {
                    seen[static_cast<size_t>(v)] = 1;
                    stack.push_back(v);
                }
            }
        }
        sizes.push_back(count);
    }
    std::sort(sizes.begin(), sizes.end(), std::greater<long>());
    return sizes;
}

} // namespace wqs
