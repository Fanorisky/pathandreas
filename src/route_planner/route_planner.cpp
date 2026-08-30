#include "route_planner/route_planner.h"
#include "collision_world/collision_world.h"
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

} // namespace

HybridResult ComposeHybridRoute(const RoadNetwork* pedRoads,
                                const RoadNetwork* vehRoads,
                                const CollisionWorld* world,
                                const Vec3& from, const Vec3& to,
                                float minSpacing) {
    HybridResult out;

    // Sidewalks first. Inside a city this is the whole answer, and it is the
    // difference between a pedestrian on the pavement and one walking up the
    // middle of the road, which is what routing a person on the vehicle graph
    // produces.
    RoadNetwork::RouteResult route;
    if (pedRoads && pedRoads->ready()) {
        route = pedRoads->findPath(from, to, RouteProfile::Ped());
        if (route.success && !route.waypoints.empty()) out.onSidewalks = true;
    }
    if (!out.onSidewalks) {
        if (!vehRoads || !vehRoads->ready()) return out;
        route = vehRoads->findPath(from, to, RouteProfile::Ped());
        if (!route.success || route.waypoints.empty()) return out;
    }

    // Backbone: node route downsampled to pedestrian pace. The first node is
    // kept even if it is closer than minSpacing to `from` - walking toward it
    // first is what gets a pedestrian off an off-graph start. The last node is
    // always kept so the tail stays on the network before the final approach.
    std::vector<Vec3> backbone;
    backbone.reserve(route.waypoints.size() + 2);
    backbone.push_back(groundSnap(world, from));
    Vec3 last = from;
    for (size_t i = 0; i < route.waypoints.size(); ++i) {
        const Vec3& node = route.waypoints[i];
        const bool isLast = i + 1 == route.waypoints.size();
        if (!isLast && (node - last).lengthSq() < minSpacing * minSpacing) continue;
        backbone.push_back(node);
        last = node;
    }
    backbone.push_back(groundSnap(world, to));

    // Ground-snap every intermediate node so the z stays truthful, but leave
    // the downsampled spacing alone - the consumer walks this with
    // move_along_surface and needs course corrections, not a dense polyline.
    for (size_t i = 1; i + 1 < backbone.size(); ++i)
        backbone[i] = groundSnap(world, backbone[i]);

    out.waypoints = std::move(backbone);
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
