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
    // True when the route came from the pedestrian node graph rather than the
    // road graph, i.e. it follows sidewalks instead of road centre lines.
    bool onSidewalks = false;
};

// Routes on the pedestrian graph when it can reach the goal and falls back to
// the road graph when it cannot. That split is not a heuristic: SA's own
// pedestrian network is excellent inside a city but breaks into 179
// components, the three largest being Los Santos (8,880 nodes), San Fierro
// (8,332) and Las Venturas (7,567) - it simply has no sidewalk between
// cities. The road graph is one connected component, so it is what carries an
// inter-city walk. Either graph may be null.
//
// minSpacing is the waypoint spacing in world units (pedestrian pace, not
// vehicle turn points). The collision world is optional; without it waypoints
// keep the node z coordinates.
HybridResult ComposeHybridRoute(const RoadNetwork* pedRoads,
                                const RoadNetwork* vehRoads,
                                const CollisionWorld* world,
                                const Vec3& from, const Vec3& to,
                                float minSpacing = 25.f);

// Vehicle dimensions in world units; the defaults describe a generic SA
// sedan. The service uses them to answer "does this route fit", never to
// drive: width widens the off-road ground check from a line into a corridor,
// height becomes an overhead clearance scan, and turnRadius flags corners the
// vehicle cannot take. turnRadius 0 means "do not check turns" - deriving one
// from the length would be a guess dressed up as a measurement, and the
// caller knows its own steering lock.
struct VehicleSpec {
    float width = 2.0f;
    float length = 4.5f;
    float height = 1.6f;
    float turnRadius = 0.f;
};

// A straight off-road leg (vehicle position <-> nearest road node). The node
// graph has no edges there, so the route is a direct line; this check walks
// the line through the collision world to report whether a car can actually
// drive it - ground must exist across the vehicle's width and rise in
// car-friendly steps.
struct OffroadLeg {
    float distance = 0.f;  // straight-line length of the leg
    bool drivable = true;  // ground checks passed
    // Why it failed, empty when it passed. "no_ground" - nothing below a
    // sample point; "step" - a height jump along the leg no car climbs;
    // "width" - the ground is not level enough across the vehicle's track,
    // which is also what a gap too narrow to fit through looks like.
    const char* reason = "";
};
OffroadLeg CheckOffroadLeg(const CollisionWorld* world, const Vec3& from, const Vec3& to,
                           const VehicleSpec& vehicle = {});

// Overhead clearance along a route: for each waypoint, how much room there is
// between the ground and whatever is above it. Only waypoints with less than
// the vehicle needs are returned, plus the smallest figure seen anywhere -
// this is what catches tunnels, garages and low underpasses for a tall
// vehicle. Waypoints with no ground below them are skipped, not reported.
struct ClearanceHit {
    size_t index = 0;    // index into the waypoint list
    float height = 0.f;  // room actually available there
};
struct ClearanceReport {
    // Tightest room found anywhere on the route, as a lower bound: when
    // nothing was hit within the vehicle's height this is that height, since
    // the scan does not look further up. 0 only when nothing was measured.
    float minHeight = 0.f;
    std::vector<ClearanceHit> hits;    // capped; see kMaxReported in the source
    long measured = 0;                 // waypoints that had ground to measure from
};
ClearanceReport CheckClearance(const CollisionWorld* world,
                               const std::vector<Vec3>& waypoints,
                               const VehicleSpec& vehicle);

// Corner sharpness along a route, as the radius of the circle through each
// waypoint and its two neighbours. IMPORTANT: this measures the polyline the
// service returned, not the road - a consumer that splines the route will be
// able to take a wider line than these numbers suggest. Read it as "these
// corners demand more than the vehicle can turn, slow down or cut wide", not
// as "this route is impossible".
struct TightTurn {
    size_t index = 0;
    float radius = 0.f;
};
struct TurnReport {
    float minRadius = 0.f;           // smallest radius on the route, 0 if straight
    std::vector<TightTurn> tight;    // only corners below vehicle.turnRadius
};
TurnReport CheckTurns(const std::vector<Vec3>& waypoints, const VehicleSpec& vehicle);

} // namespace RoutePlanner

} // namespace wqs
