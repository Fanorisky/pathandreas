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

} // namespace RoutePlanner
} // namespace wqs
