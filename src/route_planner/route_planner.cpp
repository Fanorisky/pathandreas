#include "route_planner/route_planner.h"
#include "collision_world/collision_world.h"
#include "road_network/road_network.h"

#include <cmath>

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

HybridResult ComposeHybridRoute(const RoadNetwork& roads,
                                const CollisionWorld* world,
                                const Vec3& from, const Vec3& to,
                                float minSpacing) {
    HybridResult out;
    if (!roads.ready()) return out;
    RoadNetwork::RouteResult route = roads.findPath(from, to);
    if (!route.success || route.waypoints.empty()) return out;

    // Backbone: node route downsampled to pedestrian pace. The first node is
    // kept even if it is closer than minSpacing to `from` - walking toward a
    // road node first is what gets a pedestrian out of an off-road start.
    // The last node is always kept so the tail of the route stays on the road
    // before the final approach to `to`.
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

    // The final approach (last node -> goal) can be long when the goal is far
    // from any road; ground-snap every backbone node so intermediate z stays
    // truthful, but leave the downsampled spacing alone - the consumer walks
    // this with move_along_surface and only needs course corrections, not a
    // perfect polyline.
    for (size_t i = 1; i + 1 < backbone.size(); ++i)
        backbone[i] = groundSnap(world, backbone[i]);

    out.waypoints = std::move(backbone);
    out.success = true;
    return out;
}

OffroadLeg CheckOffroadLeg(const CollisionWorld* world, const Vec3& from, const Vec3& to) {
    OffroadLeg leg;
    leg.distance = (to - from).length();
    if (!world || leg.distance < 1.f) return leg;
    // Sample the ground every ~5 units. A leg stops being drivable when the
    // ground disappears (cliff overhang, water) or consecutive samples differ
    // by more than 3 units of height (~31 degrees over 5 units - steeper than
    // a normal car climb). Coarse by design: no width, no obstacles below car
    // height; it answers "is there drivable ground in a straight line", not
    // "is this a road".
    const int samples = std::max(2, static_cast<int>(leg.distance / 5.f));
    float prevZ = from.z;
    bool havePrev = false;
    for (int i = 0; i <= samples; ++i) {
        const float t = static_cast<float>(i) / samples;
        const float x = from.x + (to.x - from.x) * t;
        const float y = from.y + (to.y - from.y) * t;
        float z = 0.f;
        // Cast from a little above the previous sample, then much higher: a
        // hill crest between samples would otherwise look like missing ground.
        if (!world->FindGroundZFrom(x, y, prevZ + 10.f, z) &&
            !world->FindGroundZFrom(x, y, prevZ + 60.f, z)) {
            leg.drivable = false;
            break;
        }
        if (havePrev && std::fabs(z - prevZ) > 3.f) {
            leg.drivable = false;
            break;
        }
        prevZ = z;
        havePrev = true;
    }
    return leg;
}

} // namespace RoutePlanner
} // namespace wqs
