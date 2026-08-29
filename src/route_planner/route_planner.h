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

// A straight off-road leg (vehicle position <-> nearest road node). The node
// graph has no edges there, so the route is a direct line; this check walks
// the line through the collision world to report whether a car can actually
// drive it - ground must exist along the way and rise in car-friendly steps.
struct OffroadLeg {
    float distance = 0.f;  // straight-line length of the leg
    bool drivable = true;  // ground-continuity check passed
};
OffroadLeg CheckOffroadLeg(const CollisionWorld* world, const Vec3& from, const Vec3& to);

} // namespace RoutePlanner

} // namespace wqs
