#pragma once

#include "common/vec3.h"
#include "collision_world/collision_world.h"
#include "pathfinder/pathfinder.h"
#include "road_network/road_network.h"
#include "query_server/world_editor.h"
#include <string>

namespace wqs {

// Stateless JSON request/response. The "id" field is echoed verbatim.
std::string HandleQueryJson(const std::string& request,
                            const CollisionWorld* world,
                            const Pathfinder* pathfinder,
                            const RoadNetwork* roads = nullptr,
                            const Pathfinder* vehiclePathfinder = nullptr,
                            WorldEditor* editor = nullptr);

} // namespace wqs
