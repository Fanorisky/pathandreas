#include "road_network/sa_paths.h"
#include "common/log.h"
#include "road_network/road_network.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <vector>

namespace wqs {
namespace {

constexpr size_t kHeaderSize   = 20;
constexpr size_t kNodeSize     = 28;
constexpr size_t kNaviSize     = 14;
constexpr size_t kLinkSize     = 4;
constexpr size_t kFillerSize   = 768;  // FF FF 00 00 x192 in every stock file
constexpr size_t kTrailerExtra = 384;  // trailer measures linkCount + this
constexpr int    kAreaCount    = 64;

template <typename T>
T rd(const std::vector<uint8_t>& d, size_t off) {
    T v{};
    std::memcpy(&v, d.data() + off, sizeof(T));
    return v;
}

struct RawNode {
    float x = 0, y = 0, z = 0;
    uint16_t linkId = 0;
    uint8_t width = 0, type = 0;
    uint32_t flags = 0;
    bool ped = false;
};

struct RawNavi {
    uint16_t targetArea = 0, targetNode = 0;
    float dx = 0, dy = 0;
    uint8_t left = 0, right = 0;
};

struct Area {
    bool present = false;
    std::vector<RawNode> nodes;
    std::vector<RawNavi> navi;
    std::vector<std::pair<uint16_t, uint16_t>> links;
    std::vector<uint16_t> naviLinks;
    std::vector<uint8_t> linkLens;
    // Global index of each local node, and which graph it landed in.
    std::vector<int> gidx;
};

bool readArea(const std::string& path, Area& a, std::string& err) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        err = "cannot open " + path;
        return false;
    }
    std::vector<uint8_t> d((std::istreambuf_iterator<char>(in)),
                           std::istreambuf_iterator<char>());
    if (d.size() < kHeaderSize) {
        err = path + ": shorter than a header";
        return false;
    }
    const uint32_t total = rd<uint32_t>(d, 0);
    const uint32_t veh   = rd<uint32_t>(d, 4);
    const uint32_t ped   = rd<uint32_t>(d, 8);
    const uint32_t navi  = rd<uint32_t>(d, 12);
    const uint32_t links = rd<uint32_t>(d, 16);
    if (veh + ped != total) {
        err = path + ": header node counts disagree";
        return false;
    }
    // Every section is fixed-stride, so the whole file size is predictable.
    // Checking it up front means a wrong assumption about the two
    // undocumented blocks fails loudly here instead of silently shifting the
    // lane data by a few bytes.
    const size_t expect = kHeaderSize + total * kNodeSize + navi * kNaviSize +
                          links * kLinkSize + kFillerSize + links * 2 + links +
                          links + kTrailerExtra;
    if (d.size() != expect) {
        err = path + ": unexpected size (" + std::to_string(d.size()) + ", expected " +
              std::to_string(expect) + ")";
        return false;
    }

    size_t off = kHeaderSize;
    a.nodes.resize(total);
    for (uint32_t i = 0; i < total; ++i) {
        const size_t o = off + i * kNodeSize;
        RawNode& n = a.nodes[i];
        // Positions are int16 in eighths of a world unit.
        n.x = rd<int16_t>(d, o + 8) / 8.0f;
        n.y = rd<int16_t>(d, o + 10) / 8.0f;
        n.z = rd<int16_t>(d, o + 12) / 8.0f;
        n.linkId = rd<uint16_t>(d, o + 16);
        n.width = d[o + 22];
        n.type = d[o + 23];
        n.flags = rd<uint32_t>(d, o + 24);
        n.ped = i >= veh;
        // Vehicle nodes come first, then pedestrian nodes; the node id field
        // equals the local index (verified across all 64 stock files), which
        // is what the link table references.
    }
    off += total * kNodeSize;

    a.navi.resize(navi);
    for (uint32_t i = 0; i < navi; ++i) {
        const size_t o = off + i * kNaviSize;
        RawNavi& v = a.navi[i];
        v.targetArea = rd<uint16_t>(d, o + 4);
        v.targetNode = rd<uint16_t>(d, o + 6);
        // Direction is a normalised vector with components scaled to +-100.
        v.dx = static_cast<int8_t>(d[o + 8]) / 100.0f;
        v.dy = static_cast<int8_t>(d[o + 9]) / 100.0f;
        const uint32_t f = rd<uint32_t>(d, o + 10);
        v.left  = static_cast<uint8_t>((f >> 8) & 0x7);   // backward lanes
        v.right = static_cast<uint8_t>((f >> 11) & 0x7);  // forward lanes
    }
    off += navi * kNaviSize;

    a.links.resize(links);
    for (uint32_t i = 0; i < links; ++i) {
        const size_t o = off + i * kLinkSize;
        a.links[i] = {rd<uint16_t>(d, o), rd<uint16_t>(d, o + 2)};
    }
    off += links * kLinkSize;

    // The filler block's size matters (everything after it would shift), its
    // content does not - warn rather than refuse if a modded file differs.
    bool filler = true;
    for (size_t i = 0; i + 3 < kFillerSize && filler; i += 4)
        if (!(d[off + i] == 0xFF && d[off + i + 1] == 0xFF && d[off + i + 2] == 0 &&
              d[off + i + 3] == 0) &&
            !(d[off + i] == 0 && d[off + i + 1] == 0 && d[off + i + 2] == 0 &&
              d[off + i + 3] == 0))
            filler = false;
    if (!filler)
        WQS_WARN("%s: filler block is neither the stock pattern nor zeroed", path.c_str());
    off += kFillerSize;

    a.naviLinks.resize(links);
    for (uint32_t i = 0; i < links; ++i) a.naviLinks[i] = rd<uint16_t>(d, off + i * 2);
    off += links * 2;

    a.linkLens.assign(d.begin() + static_cast<long>(off),
                      d.begin() + static_cast<long>(off + links));

    a.present = true;
    return true;
}

} // namespace

bool LoadSaPaths(const std::string& dir, RoadNetwork& vehicles, RoadNetwork& peds,
                 SaPathsStats& stats, std::string& err) {
    stats = SaPathsStats{};
    vehicles.clear();
    peds.clear();

    std::vector<Area> areas(kAreaCount);
    for (int i = 0; i < kAreaCount; ++i) {
        const std::string base = dir + "/NODES" + std::to_string(i);
        std::string e;
        if (!readArea(base + ".DAT", areas[static_cast<size_t>(i)], e)) {
            std::string e2;
            const std::string lower = dir + "/nodes" + std::to_string(i) + ".dat";
            if (!readArea(lower, areas[static_cast<size_t>(i)], e2)) {
                WQS_WARN("SA paths: area %d unreadable (%s)", i, e.c_str());
                continue;
            }
        }
        ++stats.areasLoaded;
        stats.naviNodes += static_cast<long>(areas[static_cast<size_t>(i)].navi.size());
        stats.links += static_cast<long>(areas[static_cast<size_t>(i)].links.size());
    }
    if (stats.areasLoaded == 0) {
        err = "no readable NODES*.DAT in " + dir;
        return false;
    }

    // Pass 1: assign a global index in the graph each node belongs to.
    for (Area& a : areas) {
        if (!a.present) continue;
        a.gidx.assign(a.nodes.size(), -1);
        for (size_t i = 0; i < a.nodes.size(); ++i) {
            const RawNode& n = a.nodes[i];
            RoadNodeInfo info;
            info.flags = n.flags;
            info.width8 = n.width;
            // That byte is a node type only for vehicle nodes; for pedestrian
            // nodes it holds a per-area id, so it must not be read as a type.
            info.type = n.ped ? 0 : n.type;
            a.gidx[i] = n.ped ? peds.addNode({n.x, n.y, n.z}, info)
                              : vehicles.addNode({n.x, n.y, n.z}, info);
        }
    }
    stats.vehNodes = vehicles.nodeCount();
    stats.pedNodes = peds.nodeCount();

    // Pass 2: resolve links. SA stores them double-linked, so both directions
    // appear; the one-way information is in the navi node's lane counts, not
    // in the link table.
    for (size_t ai = 0; ai < areas.size(); ++ai) {
        Area& a = areas[ai];
        if (!a.present) continue;
        for (size_t i = 0; i < a.nodes.size(); ++i) {
            const RawNode& n = a.nodes[i];
            const uint32_t count = n.flags & SaFlags::kLinkCountMask;
            for (uint32_t j = 0; j < count; ++j) {
                const size_t k = n.linkId + j;
                if (k >= a.links.size()) { ++stats.unresolved; continue; }
                const auto [ta, tn] = a.links[k];
                if (ta >= areas.size() || !areas[ta].present ||
                    tn >= areas[ta].nodes.size()) { ++stats.unresolved; continue; }
                const RawNode& t = areas[ta].nodes[tn];
                if (t.ped != n.ped) { ++stats.unresolved; continue; } // never happens in stock data
                const float len = static_cast<float>(a.linkLens[k]);

                uint8_t lanes = 0;
                if (!n.ped) {
                    // The navi link packs a navi node reference: low 10 bits
                    // the navi id, high 6 the area. It is only trusted when
                    // that navi node names one end of this very link, which
                    // also rejects the zero entries written for ped links.
                    const uint16_t nv = a.naviLinks[k];
                    const uint16_t nid = nv & 0x3FF, narea = nv >> 10;
                    if (narea < areas.size() && areas[narea].present &&
                        nid < areas[narea].navi.size()) {
                        const RawNavi& v = areas[narea].navi[nid];
                        const bool matches =
                            (v.targetArea == ai && v.targetNode == i) ||
                            (v.targetArea == ta && v.targetNode == tn);
                        if (matches) {
                            // Lane counts are given relative to the navi
                            // direction vector, so which side is "forward"
                            // depends on which way this edge travels.
                            const float ex = t.x - n.x, ey = t.y - n.y;
                            lanes = (ex * v.dx + ey * v.dy) > 0.f ? v.right : v.left;
                            ++stats.lanedEdges;
                            if (lanes == 0) ++stats.oneWayEdges;
                        }
                    }
                }
                if (n.ped) {
                    peds.addEdge(a.gidx[i], areas[ta].gidx[tn], len);
                    ++stats.pedEdges;
                } else {
                    vehicles.addEdge(a.gidx[i], areas[ta].gidx[tn], len, lanes);
                    ++stats.vehEdges;
                }
            }
        }
    }

    vehicles.setHasLaneData(stats.lanedEdges > 0);
    peds.setHasLaneData(false); // pedestrian links carry no navi node
    vehicles.finishBuild();
    peds.finishBuild();

    WQS_INFO("SA paths: %ld areas | vehicles %ld nodes / %ld edges (%ld laned, %ld one-way) "
             "| pedestrians %ld nodes / %ld edges",
             stats.areasLoaded, stats.vehNodes, stats.vehEdges, stats.lanedEdges,
             stats.oneWayEdges, stats.pedNodes, stats.pedEdges);
    if (stats.unresolved)
        WQS_WARN("SA paths: %ld link entries could not be resolved", stats.unresolved);
    return stats.vehNodes > 0 || stats.pedNodes > 0;
}

} // namespace wqs
