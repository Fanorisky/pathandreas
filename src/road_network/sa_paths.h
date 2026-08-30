#pragma once

#include <string>

namespace wqs {

class RoadNetwork;

// GTA SA's own path files - NODES0.DAT .. NODES63.DAT, one per 750x750 unit
// map square - hold two separate graphs: the vehicle network the game's
// traffic AI drives on, and the pedestrian network its peds walk on. Loading
// them replaces the GPS.dat road network with strictly more information:
// the pedestrian graph (absent from GPS.dat entirely), per-node class flags
// (highway, emergency-only, boat, parking), and per-segment lane counts,
// which is where one-way streets are actually recorded.
//
// The layout comes from the public format description on the GTAMods wiki
// ("Paths (GTA SA)"); this reader is original. Two sections the wiki leaves
// open were measured against the shipped files instead: the constant block
// after the link array is 768 bytes (FF FF 00 00 x192, byte-identical in all
// 64 files) and the trailing block is exactly linkCount + 384 bytes in every
// file. Both are skipped, but their sizes have to be right or every later
// section reads garbage - hence the explicit checks in the loader.
struct SaPathsStats {
    long areasLoaded = 0;
    long vehNodes = 0, vehEdges = 0;
    long pedNodes = 0, pedEdges = 0;
    long naviNodes = 0, links = 0;
    long oneWayEdges = 0;   // directed edges with no lane in the travel direction
    long lanedEdges = 0;    // directed edges that got lane data from a navi node
    long unresolved = 0;    // link entries naming a node that does not exist
};

// Fills `vehicles` and `peds` from `dir`. Both graphs are rebuilt from
// scratch. A missing area file is not fatal (some installs ship fewer), but
// zero readable areas is.
bool LoadSaPaths(const std::string& dir, RoadNetwork& vehicles, RoadNetwork& peds,
                 SaPathsStats& stats, std::string& err);

} // namespace wqs
