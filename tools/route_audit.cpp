// Route regression audit.
//
// Everything this service answers about routes was verified by hand until now:
// scratch tools, run once, never committed. This is that verification made
// repeatable. It links the routing code directly rather than talking to a
// running server, so it sees structured results (lane counts, off-road leg
// reasons, waypoint arrays) instead of JSON.
//
// It checks PROPERTIES, not numbers. "The route succeeds, stays out of the
// water, never travels against the lanes, keeps its endpoints, and the
// navmesh confirmed all but a few hops" survives a cost-function tweak;
// "the route is 7,207 units long" does not, and a test that breaks whenever
// anything is tuned gets deleted rather than fixed.
//
// Each case reduces to a short verdict of categorical flags. Verdicts are
// compared against a committed baseline, so a case that is known to fail stays
// known: it is listed with its failure, and the audit only complains when a
// verdict CHANGES. That makes both regressions and fixes visible.
//
// Note on scope: this cannot run in CI. It needs ColAndreas.cadb, the baked
// navmeshes and the game's NODES*.DAT, none of which may be redistributed. The
// unit tests in tests/test_all.cpp cover what can be checked synthetically;
// this is a local gate to run before committing anything that touches routing.

#include "collision_loader/collision_loader.h"
#include "collision_world/collision_world.h"
#include "pathfinder/pathfinder.h"
#include "road_network/road_network.h"
#include "road_network/sa_paths.h"
#include "route_planner/route_planner.h"
#include "common/log.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace wqs;

namespace {

// The widest legitimate node spacing measured in the stock files is 123 units;
// an off-road bracket leg can be longer, so this only flags the absurd.
constexpr float kMaxGap = 200.f;
constexpr float kEndpointTolerance = 3.f;
// A generic sedan, for the clearance and fit checks.
RoutePlanner::VehicleSpec carSpec() {
    RoutePlanner::VehicleSpec v;
    v.width = 2.0f; v.length = 4.5f; v.height = 1.6f; v.turnRadius = 5.5f;
    return v;
}

struct Case {
    std::string kind;   // vehicle | walk | boat
    std::string name;
    Vec3 from{}, to{};
    // Baseline key. The kind has to be part of it: the same pair of positions
    // is a perfectly reasonable case for both driving and walking, and keying
    // on the name alone silently dropped one of the two.
    std::string id() const { return kind + "/" + name; }
};

struct Backends {
    CollisionWorld world;
    Pathfinder ped, vehicleMesh;
    RoadNetwork roads, pedRoads;
    bool haveWorld = false, havePed = false, haveVehicleMesh = false, haveGraphs = false;
};

// Ratios alone mislead on short routes - one unconfirmed hop out of three is
// 33% and means almost nothing - so a small absolute count counts as low
// whatever the ratio.
std::string bucket(long part, long total) {
    if (total <= 0 || part == 0) return "none";
    if (part <= 2 || part * 100 / total <= 5) return "low";
    return "high";
}

// How far off the ground a waypoint may sit before it is not on the ground.
// Node heights are approximate, so this is generous.
constexpr float kGroundTolerance = 3.f;

// A route through open water or through nothing at all. An absolute height test
// cannot answer this: San Andreas has road tunnels down at z -46, which are
// below sea level and perfectly solid. What distinguishes them is that a tunnel
// has ground directly under the waypoint and open water does not.
bool ungrounded(const CollisionWorld& world, const std::vector<Vec3>& p) {
    long bad = 0;
    for (const Vec3& v : p) {
        float gz = 0.f;
        if (!world.FindGroundZFrom(v.x, v.y, v.z + 2.f, gz) ||
            std::fabs(gz - v.z) > kGroundTolerance)
            ++bad;
    }
    // One stray waypoint is noise; a route that is repeatedly nowhere is not.
    return bad > 2 && bad * 100 / static_cast<long>(p.size()) > 5;
}

float pathLength(const std::vector<Vec3>& p) {
    float l = 0.f;
    for (size_t i = 1; i < p.size(); ++i) l += (p[i] - p[i - 1]).length();
    return l;
}

float maxGap(const std::vector<Vec3>& p) {
    float m = 0.f;
    for (size_t i = 1; i < p.size(); ++i) m = std::max(m, (p[i] - p[i - 1]).length());
    return m;
}

// Walk the route the way a consumer must: navmesh movement per tick, and when
// a tick stops making progress, direct movement until the NEXT waypoint. That
// last detail is the whole lesson from the first end-to-end test - clearing
// recovery mode early makes an NPC oscillate in place forever.
struct SimResult {
    bool arrived = false;
    long ticks = 0;
    long recoveryTicks = 0;
    float finalDistance = 0.f;
};

SimResult simulateWalk(const Pathfinder& mesh, const CollisionWorld* world,
                       const std::vector<Vec3>& route) {
    constexpr float kStep = 2.5f;      // ~one movement tick
    constexpr float kReach = 3.0f;     // waypoint counts as reached
    SimResult out;
    if (route.size() < 2) return out;
    const long maxTicks = static_cast<long>(pathLength(route) / kStep * 4.f) + 1000;
    Vec3 pos = route.front();
    size_t idx = 1;
    bool recovery = false;
    while (out.ticks < maxTicks) {
        const Vec3 target = route[idx];
        const float dx = target.x - pos.x, dy = target.y - pos.y;
        const float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < kReach) {
            if (++idx >= route.size()) { out.arrived = true; break; }
            recovery = false;
            continue;
        }
        const float s = std::min(kStep, dist);
        const Vec3 delta{dx / dist * s, dy / dist * s, 0.f};
        if (!recovery) {
            const Vec3 next = mesh.MoveAlongSurface(pos, delta);
            const float moved = std::sqrt((next.x - pos.x) * (next.x - pos.x) +
                                          (next.y - pos.y) * (next.y - pos.y));
            if (moved < s * 0.3f) recovery = true;   // stalled: gap in the mesh
            else pos = next;
        }
        if (recovery) {
            pos = {pos.x + delta.x, pos.y + delta.y, pos.z};
            float gz = 0.f;
            if (world && world->FindGroundZFrom(pos.x, pos.y, pos.z + 3.f, gz)) pos.z = gz;
            ++out.recoveryTicks;
        }
        ++out.ticks;
    }
    // Horizontal only: z comes from the ground under the NPC, and a route that
    // ends on a kerb 2 units below its last waypoint has still arrived.
    const float dx = pos.x - route.back().x, dy = pos.y - route.back().y;
    out.finalDistance = std::sqrt(dx * dx + dy * dy);
    if (out.finalDistance > 5.f) out.arrived = false;
    return out;
}

} // namespace

namespace {

// --- the checks ------------------------------------------------------------

std::string auditVehicle(Backends& b, const Case& c, bool boat) {
    std::vector<std::string> f;
    const RouteProfile profile = boat ? RouteProfile::Boat() : RouteProfile::Car();
    const RoadNetwork::RouteResult r = b.roads.findPath(c.from, c.to, profile);
    if (!r.success || r.waypoints.empty()) return "route=none";
    f.push_back("route=ok");

    if (!boat && b.haveWorld && ungrounded(b.world, r.waypoints)) f.push_back("ungrounded=yes");
    if (maxGap(r.waypoints) > kMaxGap) f.push_back("gap=wide");

    if (boat) {
        // Boat nodes sit at water level; anything far off it is not a boat route.
        float worst = 0.f;
        for (const Vec3& v : r.waypoints) worst = std::max(worst, std::fabs(v.z));
        if (worst > 5.f) f.push_back("offwater=yes");
        return [&] { std::string s; for (auto& x : f) { if (!s.empty()) s += ' '; s += x; } return s; }();
    }

    // One-way enforcement is a property of the answer, so assert it rather than
    // trusting that the profile was applied: every leg must have a lane.
    if (b.roads.hasLaneData()) {
        long noLane = 0;
        for (const uint8_t l : r.lanes) if (l == 0) ++noLane;
        if (noLane) f.push_back("againstlanes=yes");
    }

    const RoutePlanner::VehicleSpec car = carSpec();
    const RoutePlanner::OffroadLeg s =
        RoutePlanner::CheckOffroadLeg(b.haveWorld ? &b.world : nullptr, c.from, r.waypoints.front(), car);
    const RoutePlanner::OffroadLeg g =
        RoutePlanner::CheckOffroadLeg(b.haveWorld ? &b.world : nullptr, r.waypoints.back(), c.to, car);
    f.push_back(std::string("offroad_start=") + (s.drivable ? "ok" : s.reason));
    f.push_back(std::string("offroad_goal=") + (g.drivable ? "ok" : g.reason));

    if (b.haveWorld) {
        const RoutePlanner::ClearanceReport cl =
            RoutePlanner::CheckClearance(&b.world, r.waypoints, car);
        if (!cl.hits.empty()) f.push_back("clearance=low");
    }
    std::string out;
    for (auto& x : f) { if (!out.empty()) out += ' '; out += x; }
    return out;
}

std::string auditWalk(Backends& b, const Case& c, bool simulate) {
    std::vector<std::string> f;
    const RoutePlanner::HybridResult r = RoutePlanner::ComposeHybridRoute(
        b.haveGraphs ? &b.pedRoads : nullptr, b.haveGraphs ? &b.roads : nullptr,
        b.haveWorld ? &b.world : nullptr, b.havePed ? &b.ped : nullptr, c.from, c.to);
    if (!r.success || r.waypoints.empty()) return "route=none";
    f.push_back("route=ok");

    const char* src = "?";
    switch (r.source) {
        case RoutePlanner::HybridResult::SourcePed: src = "ped"; break;
        case RoutePlanner::HybridResult::SourceVehicle: src = "vehicle"; break;
        case RoutePlanner::HybridResult::SourceStitched: src = "ped+vehicle"; break;
        default: break;
    }
    f.push_back(std::string("src=") + src);
    f.push_back("unverified=" + bucket(r.straightSegments,
                                       r.straightSegments + r.repairedSegments));

    if (b.haveWorld && ungrounded(b.world, r.waypoints)) f.push_back("ungrounded=yes");
    // The raw waypoint spacing says nothing here: a long hop inside a confirmed
    // stretch is a straight run over open ground. What matters is the longest
    // hop nothing confirmed.
    if (r.longestUnconfirmed > 60.f) f.push_back("unconfirmed_hop=long");
    // The contract says the first and last waypoints are the caller's own
    // positions; ground snapping moves z, so compare horizontally.
    auto flat = [](const Vec3& a, const Vec3& b2) {
        const float dx = a.x - b2.x, dy = a.y - b2.y;
        return std::sqrt(dx * dx + dy * dy);
    };
    if (flat(r.waypoints.front(), c.from) > kEndpointTolerance ||
        flat(r.waypoints.back(), c.to) > kEndpointTolerance)
        f.push_back("endpoints=off");

    if (simulate && b.havePed) {
        const SimResult sim = simulateWalk(b.ped, b.haveWorld ? &b.world : nullptr, r.waypoints);
        f.push_back(std::string("sim=") + (sim.arrived ? "arrived" : "stuck"));
        f.push_back("sim_recovery=" + bucket(sim.recoveryTicks, sim.ticks));
    }
    std::string out;
    for (auto& x : f) { if (!out.empty()) out += ' '; out += x; }
    return out;
}

// A failure is the service answering wrongly or unusably. Everything else in a
// verdict is recorded so that a change in it is noticed, but is not a defect:
// "the last 40 units to your goal are not drivable in a straight line" and
// "this tunnel has 0.98 units of headroom" are correct answers about the world,
// and marking them as failures would bury the real ones.
bool isPass(const std::string& verdict) {
    static const char* bad[] = {"route=none", "ungrounded=yes", "gap=wide",
                                "endpoints=off", "againstlanes=yes", "offwater=yes",
                                "unconfirmed_hop=long", "sim=stuck",
                                "sim_recovery=high"};
    for (const char* x : bad)
        if (verdict.find(x) != std::string::npos) return false;
    return true;
}

} // namespace

namespace {

bool loadCases(const std::string& path, std::vector<Case>& out, std::string& err) {
    std::ifstream in(path);
    if (!in) { err = "cannot open " + path; return false; }
    std::string line;
    long no = 0;
    while (std::getline(in, line)) {
        ++no;
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        Case c;
        if (!(ss >> c.kind >> c.name >> c.from.x >> c.from.y >> c.from.z >>
              c.to.x >> c.to.y >> c.to.z)) {
            err = path + ":" + std::to_string(no) + ": expected 'kind name fx fy fz tx ty tz'";
            return false;
        }
        if (c.kind != "vehicle" && c.kind != "walk" && c.kind != "boat") {
            err = path + ":" + std::to_string(no) + ": unknown kind " + c.kind;
            return false;
        }
        out.push_back(c);
    }
    return true;
}

// Random cases are generated from node positions with a fixed seed, so the same
// data files always produce the same corpus. They are the breadth the
// hand-picked list cannot give; the hand-picked list is the depth, since it
// names the places already known to be difficult.
void addRandomCases(const Backends& b, unsigned seed, int perKind, std::vector<Case>& out) {
    if (!b.haveGraphs) return;
    std::mt19937 rng(seed);
    auto pick = [&](const RoadNetwork& g, const RouteProfile& p) {
        std::uniform_int_distribution<long> d(0, g.nodeCount() - 1);
        for (int tries = 0; tries < 64; ++tries) {
            const long n = d(rng);
            if (g.nodeInfo(n).type == 0 || p.requireType == 0 ||
                g.nodeInfo(n).type == p.requireType)
                return g.nodePos(n);
        }
        return g.nodePos(d(rng));
    };
    struct Spec { const char* kind; const RoadNetwork* g; RouteProfile p; };
    const Spec specs[] = {
        {"vehicle", &b.roads, RouteProfile::Car()},
        {"walk", &b.pedRoads, RouteProfile::Ped()},
        {"boat", &b.roads, RouteProfile::Boat()},
    };
    for (const Spec& s : specs) {
        for (int i = 0; i < perKind; ++i) {
            Case c;
            c.kind = s.kind;
            char nm[64];
            std::snprintf(nm, sizeof nm, "random/%u/%03d", seed, i);
            c.name = nm;
            // Off-node start positions are the realistic case: an NPC is never
            // standing exactly on a node.
            std::uniform_real_distribution<float> jitter(-8.f, 8.f);
            c.from = pick(*s.g, s.p);
            c.to = pick(*s.g, s.p);
            c.from.x += jitter(rng); c.from.y += jitter(rng);
            c.to.x += jitter(rng); c.to.y += jitter(rng);
            if ((c.to - c.from).length() < 200.f) { --i; continue; }
            out.push_back(c);
        }
    }
}

void usage() {
    std::printf(
        "route_audit - repeatable property checks over real routes\n\n"
        "  --cadb PATH             collision world (enables ground, width, clearance checks)\n"
        "  --navmesh PATH          pedestrian navmesh (enables hop confirmation and --walk-sim)\n"
        "  --paths DIR             GTA SA NODES*.DAT (required)\n"
        "  --cases PATH            hand-picked cases (default tests/route_cases.txt)\n"
        "  --baseline PATH         committed verdicts (default tests/route_baseline.txt,\n"
        "                          or route_baseline_sim.txt with --walk-sim)\n"
        "  --update-baseline       rewrite the baseline from this run\n"
        "  --random N              N seeded random cases per kind (default 30, which is\n"
        "                          what the committed baselines were generated with)\n"
        "  --seed N                random seed (default 1)\n"
        "  --walk-sim              also walk each walking route tick by tick (slow)\n"
        "  --help\n\n"
        "Exit 1 on any regression against the baseline.\n");
}

} // namespace

int main(int argc, char** argv) {
    std::string cadb, navmesh, paths, casesPath = "tests/route_cases.txt", baselinePath;
    bool update = false, walkSim = false;
    int randomPerKind = 30;
    unsigned seed = 1;
    for (int i = 1; i < argc; ++i) {
        auto next = [&](std::string& dst) { if (i + 1 >= argc) return false; dst = argv[++i]; return true; };
        if (!std::strcmp(argv[i], "--help")) { usage(); return 0; }
        else if (!std::strcmp(argv[i], "--cadb")) { if (!next(cadb)) { usage(); return 2; } }
        else if (!std::strcmp(argv[i], "--navmesh")) { if (!next(navmesh)) { usage(); return 2; } }
        else if (!std::strcmp(argv[i], "--paths")) { if (!next(paths)) { usage(); return 2; } }
        else if (!std::strcmp(argv[i], "--cases")) { if (!next(casesPath)) { usage(); return 2; } }
        else if (!std::strcmp(argv[i], "--baseline")) { if (!next(baselinePath)) { usage(); return 2; } }
        else if (!std::strcmp(argv[i], "--update-baseline")) update = true;
        else if (!std::strcmp(argv[i], "--walk-sim")) walkSim = true;
        else if (!std::strcmp(argv[i], "--random")) { std::string v; if (!next(v)) { usage(); return 2; } randomPerKind = std::atoi(v.c_str()); }
        else if (!std::strcmp(argv[i], "--seed")) { std::string v; if (!next(v)) { usage(); return 2; } seed = static_cast<unsigned>(std::atol(v.c_str())); }
        else { std::fprintf(stderr, "unknown arg %s\n", argv[i]); usage(); return 2; }
    }
    if (paths.empty()) { std::fprintf(stderr, "--paths is required\n"); return 2; }
    // Simulation adds flags to every walking verdict, so the two modes cannot
    // share one baseline; default to the matching file rather than letting a
    // --walk-sim run overwrite the plain one with verdicts it cannot compare.
    if (baselinePath.empty())
        baselinePath = walkSim ? "tests/route_baseline_sim.txt" : "tests/route_baseline.txt";

    Backends b;
    std::string err;
    SaPathsStats st;
    if (!LoadSaPaths(paths, b.roads, b.pedRoads, st, err)) {
        std::fprintf(stderr, "SA paths: %s\n", err.c_str());
        return 1;
    }
    b.haveGraphs = true;
    if (!cadb.empty()) {
        CadbDatabase db;
        if (!LoadCadbFile(cadb, db, err)) { std::fprintf(stderr, "cadb: %s\n", err.c_str()); return 1; }
        const CollisionMesh mesh = AssembleWorldMesh(db, {});
        if (!b.world.build(mesh, err)) { std::fprintf(stderr, "collision: %s\n", err.c_str()); return 1; }
        b.haveWorld = true;
    }
    if (!navmesh.empty()) {
        if (!b.ped.loadFile(navmesh, err)) { std::fprintf(stderr, "navmesh: %s\n", err.c_str()); return 1; }
        b.havePed = true;
    }
    if (walkSim && !b.havePed) {
        std::fprintf(stderr, "--walk-sim needs --navmesh\n");
        return 2;
    }

    std::vector<Case> cases;
    if (!loadCases(casesPath, cases, err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }
    const size_t handPicked = cases.size();
    addRandomCases(b, seed, randomPerKind, cases);

    std::map<std::string, std::string> baseline;
    {
        std::ifstream in(baselinePath);
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '#') continue;
            const size_t tab = line.find('\t');
            if (tab == std::string::npos) continue;
            baseline[line.substr(0, tab)] = line.substr(tab + 1);
        }
    }

    std::map<std::string, std::string> verdicts;
    long pass = 0, fail = 0, regressed = 0, fixed = 0, changed = 0, isNew = 0;
    std::vector<std::string> notes;
    for (const Case& c : cases) {
        const std::string v = (c.kind == "walk") ? auditWalk(b, c, walkSim)
                                                 : auditVehicle(b, c, c.kind == "boat");
        verdicts[c.id()] = v;
        const bool ok = isPass(v);
        ok ? ++pass : ++fail;
        const auto prev = baseline.find(c.id());
        if (prev == baseline.end()) {
            ++isNew;
        } else if (prev->second != v) {
            const bool wasOk = isPass(prev->second);
            ++changed;
            if (wasOk && !ok) { ++regressed; notes.push_back("REGRESSED " + c.id() + "\n    was: " + prev->second + "\n    now: " + v); }
            else if (!wasOk && ok) { ++fixed; notes.push_back("FIXED     " + c.id() + "\n    was: " + prev->second + "\n    now: " + v); }
            else notes.push_back("changed   " + c.id() + "\n    was: " + prev->second + "\n    now: " + v);
        }
    }

    std::printf("\nroute audit: %zu cases (%zu hand-picked, %zu random, seed %u)%s\n",
                cases.size(), handPicked, cases.size() - handPicked, seed,
                walkSim ? ", with per-tick walk simulation" : "");
    std::printf("  pass %ld   fail %ld   (failures are expected where the baseline records them)\n",
                pass, fail);
    std::printf("  vs baseline: %ld new, %ld changed (%ld regressed, %ld fixed)\n",
                isNew, changed, regressed, fixed);
    for (const std::string& n : notes) std::printf("\n%s\n", n.c_str());
    if (!baseline.empty() && isNew)
        std::printf("\n%ld case(s) are not in the baseline; rerun with --update-baseline to record them.\n", isNew);

    if (update) {
        std::ofstream out(baselinePath);
        out << "# route_audit verdicts. Regenerate with --update-baseline.\n"
            << "# A line recording a failure is a KNOWN failure kept on purpose:\n"
            << "# the audit only complains when a verdict changes.\n";
        for (const auto& [name, v] : verdicts) out << name << '\t' << v << '\n';
        std::printf("\nbaseline written to %s (%zu cases)\n", baselinePath.c_str(), verdicts.size());
        return 0;
    }
    return regressed ? 1 : 0;
}
