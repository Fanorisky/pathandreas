#pragma once

#include "common/vec3.h"
#include <vector>

namespace wqs {

class CollisionWorld;
class RoadNetwork;

// Composed long-distance walking routes: the traffic node graph picks the
// corridor (it is one connected component over every road in the state, where
// the navmesh fragments between cities), and the collision world grounds each
// waypoint so a pedestrian follows real surfaces along it.
//
// Semantics: this guarantees a road-following route from the node nearest
// `from` to the node nearest `to`, plus exact endpoint positions - not exact
// reachability of `to`. If the goal sits off-road (a backyard, a rooftop),
// the route ends at the closest road point and the caller decides whether
// that is close enough.
namespace RoutePlanner {

struct HybridResult {
    bool success = false;
    std::vector<Vec3> waypoints; // GTA coords, start to goal
};

// minSpacing is the waypoint spacing in world units (pedestrian pace, not
// vehicle turn points). The collision world is optional; without it waypoints
// keep the node z coordinates.
HybridResult ComposeHybridRoute(const RoadNetwork& roads,
                                const CollisionWorld* world,
                                const Vec3& from, const Vec3& to,
                                float minSpacing = 25.f);

} // namespace RoutePlanner

} // namespace wqs
