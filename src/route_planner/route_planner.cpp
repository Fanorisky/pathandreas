#include "route_planner/route_planner.h"
#include "collision_world/collision_world.h"
#include "common/nav_area.h"
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

// Fraction of a route standing on the marked pedestrian corridor. Sampled
// rather than exhaustive: a long route can carry thousands of waypoints and one
// findNearestPoly each would cost more than the route did.
float measureSidewalkRatio(const Pathfinder& navmesh, const std::vector<Vec3>& pts) {
    constexpr size_t kMaxSamples = 200;
    if (pts.empty()) return -1.f;
    const size_t stride = std::max<size_t>(1, pts.size() / kMaxSamples);
    long on = 0, measured = 0;
    for (size_t i = 0; i < pts.size(); i += stride) {
        const unsigned char a = navmesh.AreaAt(pts[i]);
        if (a == 0) continue;   // nothing under it; not evidence either way
        ++measured;
        if (a == NavArea::kSidewalk) ++on;
    }
    return measured ? static_cast<float>(on) / static_cast<float>(measured) : -1.f;
}

HybridResult ComposeHybridRoute(const RoadNetwork* pedRoads,
                                const RoadNetwork* vehRoads,
                                const CollisionWorld* world,
                                const Pathfinder* navmesh,
                                const Vec3& from, const Vec3& to,
                                float minSpacing, float offroadCost) {
    HybridResult out;

    // 0. Ask the navmesh first. With the pedestrian corridor baked in as area
    //    ids, a plain mesh query already follows the sidewalk network - the
    //    nodes are inside the search rather than a corridor wrapped around it,
    //    so there is no second stage to arbitrate against and nothing to
    //    straighten afterwards. This is the answer for any trip inside one
    //    connected piece of mesh, which is most walking inside a city.
    //
    //    The node graphs below stay for what this cannot do: the mesh's largest
    //    walkable component is only ~23% of the map, so a trip between cities
    //    is not one mesh query, however it is priced.
    if (navmesh && navmesh->ready()) {
        const PathResult direct = navmesh->FindPath(from, to, world, offroadCost);
        if (direct.success && direct.waypoints.size() >= 2) {
            const Vec3& end = direct.waypoints.back();
            const float gapH = std::sqrt((to.x - end.x) * (to.x - end.x) +
                                         (to.y - end.y) * (to.y - end.y));
            const float gapV = to.z - end.z;
            // Same rule as the corridor path uses for its final approach: a
            // partial result that lands beside the goal is the route, and a
            // mostly-vertical remainder is a level change to report, not walk.
            const bool nearGoal = gapH <= 8.f;
            if (!direct.partial || nearGoal) {
                out.waypoints = direct.waypoints;
                out.source = HybridResult::SourceMesh;
                out.repairedSegments = static_cast<long>(direct.waypoints.size()) - 1;
                for (size_t i = 0; i + 1 < direct.offMesh.size(); ++i)
                    if (direct.offMesh[i]) out.climbAt.push_back(i);
                if (direct.partial && std::fabs(gapV) > 3.f) {
                    out.reachedGoal = false;
                    out.goalGapHoriz = gapH;
                    out.goalGapVert = gapV;
                } else if (gapH > 0.5f) {
                    out.waypoints.push_back(to);
                }
                out.sidewalkRatio = measureSidewalkRatio(*navmesh, out.waypoints);
                out.success = true;
                return out;
            }
        }
    }

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

    // 3. Pull the route tight through the corridor with the navmesh. This is
    //    the crux of combining the two backends: the node graph is the
    //    reference for WHICH WAY to go and for crossing the gaps where the mesh
    //    fragments, but the navmesh finds the actual line. Tracing the route
    //    node to node produced a path that hugged the road/sidewalk centre and
    //    zig-zagged through every node - up to 40% longer than the mesh's own
    //    path over the same ground, with several times the turning.
    //
    //    From each anchor we reach for the FURTHEST corridor point the mesh can
    //    get to by a path no longer than the corridor between them allows. A
    //    run of nodes across open ground then collapses to a single straight
    //    mesh leg, and where the whole route is one connected mesh region it
    //    becomes the direct optimal path with the nodes skipped entirely. Where
    //    the mesh cannot reach the next node - a fragment boundary - we keep the
    //    straight hop and count it unverified, exactly as before: that is where
    //    a consumer still needs its recovery mode.
    //
    //    The reach is found by doubling (anchor+1, +2, +4, ...) until a probe
    //    fails the length test, so a connected stretch of N nodes costs about
    //    log2(N) navmesh queries rather than N. A mesh leg longer than the
    //    corridor arc by more than kSlack is rejected: that means the mesh had
    //    to detour around something the node corridor ignores, and the node
    //    graph is the one to trust for macro direction, so we take a shorter
    //    jump instead. Corner-cutting only ever makes the leg shorter, so it is
    //    always accepted - which is where the optimisation comes from.
    constexpr float kSlack = 1.25f;         // gates skipping intermediate nodes
    constexpr float kAdjacentDetour = 6.f;  // gates confirming the very next node
    // How close to the goal a partial mesh path must land to count as reaching
    // it; the last few units to an off-mesh goal are the consumer's approach.
    constexpr float kGoalReachTolerance = 8.f;
    std::vector<Vec3> pts;
    pts.push_back(backbone.front());
    size_t i = 0;
    while (i + 1 < backbone.size()) {
        const Vec3& a = backbone[i];
        size_t bestJ = i;                 // i means "nothing reachable beyond the next node"
        std::vector<Vec3> bestLeg;
        std::vector<uint8_t> bestLegOffMesh;
        float arc = 0.f;                  // corridor arc-length from i to the current probe
        size_t reached = i;               // how far arc has been accumulated
        for (size_t step = 1;; step *= 2) {
            const size_t cand = std::min(i + step, backbone.size() - 1);
            for (size_t k = reached + 1; k <= cand; ++k)
                arc += (backbone[k] - backbone[k - 1]).length();
            reached = cand;
            const PathResult leg = navmesh->FindPath(a, backbone[cand], world, offroadCost);
            const bool isGoal = cand == backbone.size() - 1;
            // A leg is normally only trusted when it reaches its target poly
            // (not partial). The one exception is the goal itself: unlike every
            // intermediate gate, which is a node and therefore on the mesh, the
            // goal is the caller's own position and may sit just off the mesh -
            // on a kerb, a slope, a tiny fragment. When the mesh gets within a
            // few units of it, that is the real route and the last step is the
            // consumer's approach; rejecting it for the partial flag is what
            // used to drag the whole route back onto the detouring node
            // corridor when a near-straight mesh path existed.
            bool usable = leg.success && leg.waypoints.size() >= 2;
            if (usable && leg.partial) {
                const Vec3& end = leg.waypoints.back();
                const Vec3& tgt = backbone[cand];
                const float d = std::sqrt((end.x - tgt.x) * (end.x - tgt.x) +
                                          (end.y - tgt.y) * (end.y - tgt.y));
                usable = isGoal && d <= kGoalReachTolerance;
            }
            bool ok = false;
            if (usable) {
                float l = 0.f;
                for (size_t k = 1; k < leg.waypoints.size(); ++k)
                    l += (leg.waypoints[k] - leg.waypoints[k - 1]).length();
                // Two different questions with two different thresholds. For the
                // adjacent node it is only "is the next node walkable from here"
                // - the mesh may curve well around a corner and still be the
                // route, so this is lenient. For a node further along it is
                // "can I skip everything in between", which must stay near the
                // corridor or the shortcut leaves it, so it is tight.
                const float threshold = (cand == i + 1) ? kAdjacentDetour : kSlack;
                if (l <= std::max(arc, minSpacing) * threshold) {
                    ok = true;
                    bestJ = cand;
                    bestLeg = leg.waypoints;
                    bestLegOffMesh = leg.offMesh;
                }
            }
            if (!ok || isGoal) break;
        }
        if (bestJ > i) {
            // waypoints[0] is the mesh's snap of `a`, already the last point in pts.
            // Carry the leg's climb markers across, rebased onto pts. Index k of
            // the leg lands at pts.size() + k - 1 once the rest is appended.
            const size_t base = pts.size() - 1;
            for (size_t k = 0; k + 1 < bestLeg.size(); ++k)
                if (k < bestLegOffMesh.size() && bestLegOffMesh[k])
                    out.climbAt.push_back(base + k);
            for (size_t k = 1; k < bestLeg.size(); ++k) pts.push_back(bestLeg[k]);
            out.repairedSegments += static_cast<long>(bestJ - i);
            i = bestJ;
        } else {
            const Vec3& b = backbone[i + 1];
            pts.push_back(b);
            ++out.straightSegments;
            out.longestUnconfirmed = std::max(out.longestUnconfirmed, (b - a).length());
            i += 1;
        }
    }
    // Finish at the goal. The last pulled leg may have stopped at the best
    // reachable point rather than the goal itself: the goal is the caller's
    // own position and can sit off the mesh, and - the case that matters - it
    // can be on a surface the route cannot walk to at all, a different floor
    // reached by an elevator. Whether to append the exact goal depends on why
    // there is a gap. A small gap at the same level is a genuine final
    // approach: append it. A gap that is mostly vertical is a level change the
    // service cannot model; stop at the reachable point and say so, rather than
    // fabricate a step that walks up through a ceiling.
    constexpr float kSameLevel = 3.f;
    const Vec3 end = pts.back();
    const float gapH = std::sqrt((to.x - end.x) * (to.x - end.x) +
                                 (to.y - end.y) * (to.y - end.y));
    const float gapV = to.z - end.z;
    if (gapH <= kGoalReachTolerance && std::fabs(gapV) <= kSameLevel) {
        if ((end - backbone.back()).length() > 0.5f) pts.push_back(backbone.back());
    } else if (gapH <= kGoalReachTolerance && std::fabs(gapV) > kSameLevel) {
        // Reached the base under (or beside) a goal on another level.
        out.reachedGoal = false;
        out.goalGapHoriz = gapH;
        out.goalGapVert = gapV;
    } else {
        // The route did not get near the goal - a genuinely fragmented long
        // trip. Keep the existing behaviour: the goal anchor closes it out and
        // the unconfirmed accounting already flags the stretch.
        if ((end - backbone.back()).length() > 0.5f) pts.push_back(backbone.back());
        out.reachedGoal = false;
        out.goalGapHoriz = gapH;
        out.goalGapVert = gapV;
    }

    out.waypoints = std::move(pts);
    out.sidewalkRatio = measureSidewalkRatio(*navmesh, out.waypoints);
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
