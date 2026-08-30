#include "route_planner/route_planner.h"
#include "collision_world/collision_world.h"
#include "pathfinder/pathfinder.h"
#include "road_network/road_network.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace wqs {
namespace RoutePlanner {

namespace {

// Drop a waypoint onto the walkable surface below it (the node graph carries
// road-centre heights, which are close but not exact, and endpoint z values
// from callers can be anything). Falls back to the input z when nothing is
// hit from above or the ground is far below - bridges and overpasses keep
// their own height that way.
Vec3 groundSnap(const CollisionWorld* world, const Vec3& p) {
    if (!world) return p;
    float gz = 0.f;
    if (world->FindGroundZFrom(p.x, p.y, p.z + 2.f, gz) && gz > p.z - 5.f)
        return {p.x, p.y, gz};
    return p;
}

// Replace the head and tail of a road corridor with sidewalk routes. The
// handover points are found by component identity rather than by search: the
// first corridor point whose nearest sidewalk node shares a component with the
// goal, and the last one before it that shares a component with the start.
// A corridor point out in the countryside would otherwise "belong" to whatever
// city's sidewalk node happens to be nearest, so a handover only counts when a
// sidewalk node is actually alongside the road there.
std::vector<Vec3> stitchSidewalkLegs(const RoadNetwork& ped,
                                     const std::vector<Vec3>& corridor,
                                     const Vec3& from, const Vec3& to,
                                     HybridResult& out) {
    constexpr float kHandoverRadius = 30.f;
    const RouteProfile pp = RouteProfile::Ped();
    const long startNode = ped.nearestNode(from, pp);
    const long goalNode = ped.nearestNode(to, pp);
    if (startNode < 0 || goalNode < 0) return corridor;
    const int startComp = ped.componentId(startNode);
    const int goalComp = ped.componentId(goalNode);
    // Same component means step 1 already routed it on sidewalks; different
    // components are what this function exists for.
    if (startComp < 0 || goalComp < 0 || startComp == goalComp) return corridor;

    auto sidewalkComp = [&](const Vec3& p) {
        const long n = ped.nearestNode(p, pp);
        if (n < 0) return -1;
        if ((ped.nodePos(n) - p).length() > kHandoverRadius) return -1;
        return ped.componentId(n);
    };

    long firstGoal = -1;
    for (size_t i = 0; i < corridor.size(); ++i)
        if (sidewalkComp(corridor[i]) == goalComp) { firstGoal = static_cast<long>(i); break; }
    if (firstGoal < 0) return corridor;
    long lastStart = -1;
    for (long i = 0; i < firstGoal; ++i)
        if (sidewalkComp(corridor[static_cast<size_t>(i)]) == startComp) lastStart = i;
    if (lastStart < 0) return corridor;

    const RoadNetwork::RouteResult head =
        ped.findPath(from, corridor[static_cast<size_t>(lastStart)], pp);
    const RoadNetwork::RouteResult tail =
        ped.findPath(corridor[static_cast<size_t>(firstGoal)], to, pp);
    if (!head.success || !tail.success) return corridor;

    std::vector<Vec3> merged;
    merged.reserve(head.waypoints.size() + static_cast<size_t>(firstGoal - lastStart + 1) +
                   tail.waypoints.size());
    for (const Vec3& v : head.waypoints) merged.push_back(v);
    // The road stretch between the two cities, inclusive of both handover
    // points so the jump from sidewalk to carriageway is an explicit step.
    for (long i = lastStart; i <= firstGoal; ++i) merged.push_back(corridor[static_cast<size_t>(i)]);
    for (const Vec3& v : tail.waypoints) merged.push_back(v);
    out.source = HybridResult::SourceStitched;
    return merged;
}

} // namespace

HybridResult ComposeHybridRoute(const RoadNetwork* pedRoads,
                                const RoadNetwork* vehRoads,
                                const CollisionWorld* world,
                                const Pathfinder* navmesh,
                                const Vec3& from, const Vec3& to,
                                float minSpacing) {
    HybridResult out;

    // 1. Sidewalks alone. Inside a city this is the whole answer, and it is
    //    the difference between a pedestrian on the pavement and one walking
    //    up the middle of the road.
    std::vector<Vec3> nodes;
    if (pedRoads && pedRoads->ready()) {
        RoadNetwork::RouteResult r = pedRoads->findPath(from, to, RouteProfile::Ped());
        if (r.success && !r.waypoints.empty()) {
            nodes = std::move(r.waypoints);
            out.source = HybridResult::SourcePed;
        }
    }

    // 2. The sidewalk network stops at the city limits, so hand over to the
    //    road graph for the stretch between cities and hand back on arrival.
    if (out.source == HybridResult::SourceNone && vehRoads && vehRoads->ready()) {
        RoadNetwork::RouteResult corridor = vehRoads->findPath(from, to, RouteProfile::Ped());
        if (corridor.success && !corridor.waypoints.empty()) {
            out.source = HybridResult::SourceVehicle;
            nodes = (pedRoads && pedRoads->ready())
                        ? stitchSidewalkLegs(*pedRoads, corridor.waypoints, from, to, out)
                        : std::move(corridor.waypoints);
        }
    }
    if (out.source == HybridResult::SourceNone) return out;

    // Backbone: the node route downsampled to pedestrian pace. The first node
    // is kept even when it is closer than minSpacing to `from` - walking to it
    // is what gets a pedestrian off an off-network start - and the last is
    // always kept so the tail stays on the network before the final approach.
    std::vector<Vec3> backbone;
    backbone.reserve(nodes.size() + 2);
    // Two backbone points at the same place are one place: appending both
    // would create a segment nothing can verify - typically a purely vertical
    // one, where a node's own height and the ground-snapped height of the same
    // spot differ - and it would then be counted as an unconfirmed stretch of
    // route when it is not a stretch of route at all. Compare horizontally and
    // keep the newer point, which for the final push is the caller's exact
    // position rather than the node beside it.
    constexpr float kSamePlace = 1.5f;
    auto pushUnique = [&backbone](const Vec3& v) {
        if (!backbone.empty()) {
            const Vec3& b = backbone.back();
            const float dx = v.x - b.x, dy = v.y - b.y;
            if (dx * dx + dy * dy < kSamePlace * kSamePlace) {
                backbone.back() = v;
                return;
            }
        }
        backbone.push_back(v);
    };
    pushUnique(groundSnap(world, from));
    Vec3 last = from;
    for (size_t i = 0; i < nodes.size(); ++i) {
        const bool isLast = i + 1 == nodes.size();
        if (!isLast && (nodes[i] - last).lengthSq() < minSpacing * minSpacing) continue;
        pushUnique(nodes[i]);
        last = nodes[i];
    }
    pushUnique(groundSnap(world, to));
    for (size_t i = 1; i + 1 < backbone.size(); ++i)
        backbone[i] = groundSnap(world, backbone[i]);

    if (!navmesh || !navmesh->ready() || backbone.size() < 2) {
        out.straightSegments = backbone.size() > 1
                                   ? static_cast<long>(backbone.size()) - 1 : 0;
        for (size_t i = 1; i < backbone.size(); ++i)
            out.longestUnconfirmed =
                std::max(out.longestUnconfirmed, (backbone[i] - backbone[i - 1]).length());
        out.waypoints = std::move(backbone);
        out.success = true;
        return out;
    }

    // 3. Each straight hop between backbone points replaced by a route the
    //    navmesh knows is walkable. A mesh detour far longer than the hop means
    //    the backbone and the mesh disagree about this stretch; the node graph
    //    is the more trustworthy of the two for macro connectivity, so keep the
    //    straight line there rather than bolting on a large loop, and let the
    //    consumer's recovery mode handle it as before.
    constexpr float kMaxDetour = 6.f;
    std::vector<Vec3> pts;
    pts.push_back(backbone.front());
    for (size_t i = 1; i < backbone.size(); ++i) {
        const Vec3& a = backbone[i - 1];
        const Vec3& b = backbone[i];
        const PathResult leg = navmesh->FindPath(a, b, world);
        bool repaired = false;
        if (leg.success && !leg.partial && leg.waypoints.size() >= 2) {
            float len = 0.f;
            for (size_t k = 1; k < leg.waypoints.size(); ++k)
                len += (leg.waypoints[k] - leg.waypoints[k - 1]).length();
            const float direct = (b - a).length();
            if (len <= std::max(direct * kMaxDetour, minSpacing)) {
                // waypoints[0] is the mesh's snap of `a`, already in `pts`.
                for (size_t k = 1; k < leg.waypoints.size(); ++k)
                    pts.push_back(leg.waypoints[k]);
                repaired = true;
            }
        }
        if (repaired) ++out.repairedSegments;
        else {
            pts.push_back(b);
            ++out.straightSegments;
            out.longestUnconfirmed = std::max(out.longestUnconfirmed, (b - a).length());
        }
    }
    // A repaired last segment ends on the mesh's snap of the goal; the contract
    // is that the final waypoint is the caller's own position.
    if ((pts.back() - backbone.back()).length() > 0.5f) pts.push_back(backbone.back());

    out.waypoints = std::move(pts);
    out.success = true;
    return out;
}

OffroadLeg CheckOffroadLeg(const CollisionWorld* world, const Vec3& from, const Vec3& to,
                           const VehicleSpec& vehicle) {
    OffroadLeg leg;
    leg.distance = (to - from).length();
    if (!world || leg.distance < 1.f) return leg;
    // Sample the ground every ~5 units, and at each step across the vehicle's
    // track as well as down its centre line. A leg stops being drivable when
    // the ground disappears (cliff overhang, water), when consecutive centre
    // samples differ by more than 3 units of height (~31 degrees over 5 units,
    // steeper than a car climbs), or when the ground is not level enough
    // across the track. That last test is also how a gap too narrow to fit
    // through is caught: the outer samples land on top of whatever forms the
    // gap and the spread blows up.
    const int samples = std::max(2, static_cast<int>(leg.distance / 5.f));
    const float half = std::max(0.f, vehicle.width * 0.5f);
    // A wheel rides over a kerb-height step whatever the track width, so allow
    // that much unconditionally; beyond it, read the spread as a cross-slope
    // and cap it near 27 degrees, about where a car stops staying upright.
    // Neither limit is the one that matters most in practice: a wall or a
    // building inside the corridor produces a spread of several units and is
    // refused either way. Without the flat allowance the check turns into a
    // false-refusal machine on rough but perfectly rideable ground.
    constexpr float kKerbAllowance = 0.5f;
    constexpr float kMaxCrossSlope = 0.5f;
    const float spreadLimit = std::max(kKerbAllowance, vehicle.width * kMaxCrossSlope);
    // Perpendicular to the leg in the XY plane; z is irrelevant, the samples
    // are vertical raycasts.
    float px = -(to.y - from.y), py = (to.x - from.x);
    const float plen = std::sqrt(px * px + py * py);
    if (plen > 1e-6f) { px /= plen; py /= plen; }
    else { px = 1.f; py = 0.f; }

    float prevZ = from.z;
    bool havePrev = false;
    for (int i = 0; i <= samples; ++i) {
        const float t = static_cast<float>(i) / samples;
        const float cx = from.x + (to.x - from.x) * t;
        const float cy = from.y + (to.y - from.y) * t;
        // Cast from a little above the previous sample, then much higher: a
        // hill crest between samples would otherwise look like missing ground.
        auto ground = [&](float x, float y, float& z) {
            return world->FindGroundZFrom(x, y, prevZ + 10.f, z) ||
                   world->FindGroundZFrom(x, y, prevZ + 60.f, z);
        };
        float zc = 0.f;
        if (!ground(cx, cy, zc)) {
            leg.drivable = false;
            leg.reason = "no_ground";
            break;
        }
        if (havePrev && std::fabs(zc - prevZ) > 3.f) {
            leg.drivable = false;
            leg.reason = "step";
            break;
        }
        if (half > 0.05f) {
            float zl = 0.f, zr = 0.f;
            if (!ground(cx - px * half, cy - py * half, zl) ||
                !ground(cx + px * half, cy + py * half, zr)) {
                leg.drivable = false;
                leg.reason = "no_ground";
                break;
            }
            const float spread = std::max({zc, zl, zr}) - std::min({zc, zl, zr});
            if (spread > spreadLimit) {
                leg.drivable = false;
                leg.reason = "width";
                break;
            }
        }
        prevZ = zc;
        havePrev = true;
    }
    return leg;
}

ClearanceReport CheckClearance(const CollisionWorld* world,
                               const std::vector<Vec3>& waypoints,
                               const VehicleSpec& vehicle) {
    // A long route would otherwise return hundreds of entries; the caller
    // needs to know where the first problems are and how bad the worst is,
    // not a transcript.
    constexpr size_t kMaxReported = 32;
    // Start the upward ray just off the ground so a road surface right at the
    // sample height is not reported as an obstacle.
    constexpr float kFootOffset = 0.15f;
    ClearanceReport out;
    if (!world || vehicle.height <= 0.f) return out;
    float worst = std::numeric_limits<float>::max();
    for (size_t i = 0; i < waypoints.size(); ++i) {
        const Vec3& w = waypoints[i];
        // Finding the surface has to start just above the waypoint, not well
        // above it: a low roof 1.2 units over the road is exactly what this
        // function looks for, and a downward ray started higher would find
        // that roof and call it the ground.
        float gz = 0.f;
        bool have = world->FindGroundZFrom(w.x, w.y, w.z + 0.5f, gz);
        if (!have) {
            // The waypoint may sit below the surface - node heights are
            // approximate. Retry from higher up, but refuse a surface clearly
            // above the waypoint, which would be that same mistake.
            have = world->FindGroundZFrom(w.x, w.y, w.z + 3.f, gz) && gz <= w.z + 1.f;
        }
        if (!have) continue;
        ++out.measured;
        const Vec3 foot{w.x, w.y, gz + kFootOffset};
        const Vec3 head{w.x, w.y, gz + vehicle.height};
        RayHitResult hit;
        if (!world->RayCastLine(foot, head, hit) || !hit.hit) continue;
        const float available = hit.point.z - gz;
        worst = std::min(worst, available);
        if (out.hits.size() < kMaxReported) out.hits.push_back({i, available});
    }
    // Nothing overhead within the vehicle's height is the good case, and
    // reporting 0 for it would read as "no room at all". Report the height
    // asked about instead, so the number always means "at least this much
    // room, everywhere" and larger is always better.
    if (out.measured > 0)
        out.minHeight = (worst == std::numeric_limits<float>::max()) ? vehicle.height : worst;
    return out;
}

TurnReport CheckTurns(const std::vector<Vec3>& waypoints, const VehicleSpec& vehicle) {
    constexpr size_t kMaxReported = 32;
    TurnReport out;
    if (waypoints.size() < 3) return out;
    float smallest = std::numeric_limits<float>::max();
    for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
        const Vec3& a = waypoints[i - 1];
        const Vec3& b = waypoints[i];
        const Vec3& c = waypoints[i + 1];
        // Radius of the circle through the three points, in the XY plane.
        // Twice the signed area of the triangle vanishes when they are
        // collinear, which is a straight run rather than a corner.
        const float area2 = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        if (std::fabs(area2) < 1e-4f) continue;
        const float ab = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
        const float bc = std::sqrt((c.x - b.x) * (c.x - b.x) + (c.y - b.y) * (c.y - b.y));
        const float ca = std::sqrt((a.x - c.x) * (a.x - c.x) + (a.y - c.y) * (a.y - c.y));
        const float radius = (ab * bc * ca) / (2.f * std::fabs(area2));
        smallest = std::min(smallest, radius);
        if (vehicle.turnRadius > 0.f && radius < vehicle.turnRadius &&
            out.tight.size() < kMaxReported)
            out.tight.push_back({i, radius});
    }
    if (smallest != std::numeric_limits<float>::max()) out.minRadius = smallest;
    return out;
}

} // namespace RoutePlanner
} // namespace wqs
