#pragma once

namespace wqs {

// Polygon area ids the navmesh bake assigns, shared by the builder that writes
// them and the queries that price them. Recast allows 0..63 and 63 is its own
// RC_WALKABLE_AREA, so generic walkable ground keeps that value and the marked
// pedestrian corridor gets its own id.
//
// Making one cheaper than the other at query time is the whole mechanism by
// which a route follows the sidewalk network the game's authors laid out
// instead of the geometrically shortest line across roads and plazas. It is a
// preference, not a wall: a route still leaves the corridor when staying on it
// would cost more than the detour saves - crossing a street, reaching a goal
// off the network.
namespace NavArea {
constexpr unsigned char kWalkable = 63;   // any walkable surface
constexpr unsigned char kSidewalk = 62;   // within reach of a pedestrian node
} // namespace NavArea

} // namespace wqs
