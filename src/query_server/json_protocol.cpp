#include "query_server/json_protocol.h"
#include "nlohmann/json.hpp"
#include "road_network/road_network.h"
#include "route_planner/route_planner.h"
#include "world_manager/world_manager.h"

using json = nlohmann::json;

namespace wqs {
namespace {

json vecArr(const Vec3& v) { return json::array({v.x, v.y, v.z}); }

// Waypoint indices where the step to the next waypoint crosses an off-mesh
// connection - a baked step or climb. Sent as indices, not a per-waypoint
// array: a route can carry thousands of waypoints and only a handful of climbs.
json climbArr(const std::vector<uint8_t>& offMesh) {
    json a = json::array();
    for (size_t i = 0; i + 1 < offMesh.size(); ++i)
        if (offMesh[i]) a.push_back(i);
    return a;
}

bool parseVec(const json& j, Vec3& out, std::string& err) {
    if (!j.is_array() || j.size() != 3) {
        err = "expected [x,y,z]";
        return false;
    }
    out.x = j[0].get<float>();
    out.y = j[1].get<float>();
    out.z = j[2].get<float>();
    return true;
}

json errorResp(const json& id, const std::string& msg) {
    return {{"type", "error"}, {"id", id}, {"error", msg}};
}

// "vehicle": {"width":2.0,"length":4.5,"height":1.6,"turn_radius":5.5}
// Every field is optional and falls back to the generic-sedan default, so a
// caller can send only what it knows.
RoutePlanner::VehicleSpec parseVehicle(const json& v) {
    RoutePlanner::VehicleSpec s;
    if (v.contains("width")) s.width = v["width"].get<float>();
    if (v.contains("length")) s.length = v["length"].get<float>();
    if (v.contains("height")) s.height = v["height"].get<float>();
    if (v.contains("turn_radius")) s.turnRadius = v["turn_radius"].get<float>();
    return s;
}

// Fit report for a finished route. Kept out of the query bodies because both
// find_vehicle_path and find_offroad_path answer it the same way.
json vehicleCheck(const CollisionWorld* world, const std::vector<Vec3>& pts,
                  const RoutePlanner::VehicleSpec& veh, const MeshAgent& agent,
                  bool meshLoaded) {
    const RoutePlanner::ClearanceReport clear = RoutePlanner::CheckClearance(world, pts, veh);
    const RoutePlanner::TurnReport turns = RoutePlanner::CheckTurns(pts, veh);
    json low = json::array();
    for (const auto& h : clear.hits)
        low.push_back({{"index", h.index}, {"height", h.height}});
    json tight = json::array();
    for (const auto& t : turns.tight)
        tight.push_back({{"index", t.index}, {"radius", t.radius}});
    json out = {
        {"width", veh.width}, {"length", veh.length}, {"height", veh.height},
        {"turn_radius", veh.turnRadius},
        // Clearance is only measured where there was ground below a waypoint.
        {"measured_waypoints", clear.measured},
        {"min_clearance", clear.minHeight},
        {"low_clearance", low},
        // Curvature of the returned polyline, not of the road; a consumer that
        // splines the route can take a wider line than this.
        {"min_turn_radius", turns.minRadius},
        {"tight_turns", tight},
    };
    if (meshLoaded) {
        // Detour bakes the agent radius into the mesh by eroding it, so a mesh
        // route is only valid for a vehicle that fits the bake.
        out["mesh_agent_radius"] = agent.radius;
        out["mesh_agent_height"] = agent.height;
        out["exceeds_mesh_agent"] =
            (veh.width * 0.5f > agent.radius) || (veh.height > agent.height);
    }
    return out;
}

// Optional per-request overrides on a named profile, so a consumer can ask for
// a delivery van that may cut through service roads or a police car that may
// drive the wrong way, without the service inventing vehicle classes.
RouteProfile applyProfileOverrides(RouteProfile p, const json& req) {
    if (req.contains("one_way")) p.respectOneWay = req["one_way"].get<bool>();
    if (req.contains("allow_emergency")) p.allowEmergency = req["allow_emergency"].get<bool>();
    if (req.contains("highway_cost")) p.highwayCost = req["highway_cost"].get<float>();
    return p;
}

} // namespace

std::string HandleQueryJson(const std::string& request,
                            const CollisionWorld* world,
                            const Pathfinder* pathfinder,
                            const RoadNetwork* roads,
                            const Pathfinder* vehiclePathfinder,
                            WorldEditor* editor,
                            const RoadNetwork* pedRoads,
                            const MeshAgent& vehicleAgent) {
    json req;
    try {
        req = json::parse(request);
    } catch (const std::exception& e) {
        return json{{"type", "error"}, {"id", nullptr}, {"error", e.what()}}.dump();
    }
    const json id = req.contains("id") ? req["id"] : json(nullptr);
    const std::string type = req.value("type", "");
    std::string err;

    if (type == "ping") {
        return json{{"type", "pong"}, {"id", id}}.dump();
    }

    if (type == "raycast") {
        if (!world) return errorResp(id, "collision world not loaded").dump();
        Vec3 from, to;
        if (!parseVec(req["from"], from, err) || !parseVec(req["to"], to, err))
            return errorResp(id, err).dump();
        RayHitResult hit;
        world->RayCastLine(from, to, hit);
        json resp = {{"type", "raycast_result"}, {"id", id}, {"hit", hit.hit}};
        if (hit.hit) {
            resp["point"] = vecArr(hit.point);
            resp["normal"] = vecArr(hit.normal);
            resp["fraction"] = hit.fraction;
        }
        return resp.dump();
    }

    if (type == "find_ground_z") {
        if (!world) return errorResp(id, "collision world not loaded").dump();
        if (!req.contains("x") || !req.contains("y"))
            return errorResp(id, "x and y required").dump();
        const float x = req["x"].get<float>();
        const float y = req["y"].get<float>();
        float z = 0.f;
        bool found = false;
        if (req.contains("from_z")) found = world->FindGroundZFrom(x, y, req["from_z"].get<float>(), z);
        else found = world->FindGroundZ(x, y, z);
        return json{{"type", "find_ground_z_result"}, {"id", id}, {"found", found}, {"z", z}}.dump();
    }

    if (type == "find_path") {
        if (!pathfinder || !pathfinder->ready())
            return errorResp(id, "navmesh not loaded").dump();
        Vec3 from, to;
        if (!parseVec(req["from"], from, err) || !parseVec(req["to"], to, err))
            return errorResp(id, err).dump();
        // Pass the collision world so FindPath can ground-snap the endpoints and
        // raycast-validate each segment. When it is absent (navmesh-only build) the
        // pathfinder runs without those extra checks.
        PathResult p = pathfinder->FindPath(from, to, world);
        json wps = json::array();
        for (const auto& v : p.waypoints) wps.push_back(vecArr(v));
        // partial=true: the goal is unreachable; waypoints lead only part of the way.
        return json{{"type", "find_path_result"}, {"id", id}, {"success", p.success},
                    {"partial", p.partial}, {"waypoints", wps},
                    // Steps that must be moved through directly:
                    // move_along_surface cannot cross an off-mesh link.
                    {"climb_at", climbArr(p.offMesh)}}.dump();
    }

    if (type == "move_along_surface") {
        if (!pathfinder || !pathfinder->ready())
            return errorResp(id, "navmesh not loaded").dump();
        Vec3 from, delta;
        if (!parseVec(req["from"], from, err) || !parseVec(req["delta"], delta, err))
            return errorResp(id, err).dump();
        Vec3 pos = pathfinder->MoveAlongSurface(from, delta);
        return json{{"type", "move_along_surface_result"}, {"id", id}, {"position", vecArr(pos)}}.dump();
    }

    if (type == "find_offroad_path") {
        if (!vehiclePathfinder || !vehiclePathfinder->ready())
            return errorResp(id, "vehicle navmesh not loaded").dump();
        Vec3 from, to;
        if (!parseVec(req["from"], from, err) || !parseVec(req["to"], to, err))
            return errorResp(id, err).dump();
        // Car-agent navmesh routing for trips that never touch a road
        // (beach to beach, across open desert).
        PathResult p = vehiclePathfinder->FindPath(from, to, world);
        json wps = json::array();
        for (const auto& v : p.waypoints) wps.push_back(vecArr(v));
        json resp = {{"type", "find_offroad_path_result"}, {"id", id},
                     {"success", p.success}, {"partial", p.partial},
                     {"waypoints", wps}, {"climb_at", climbArr(p.offMesh)}};
        if (req.contains("vehicle"))
            resp["vehicle_check"] = vehicleCheck(world, p.waypoints,
                                                 parseVehicle(req["vehicle"]),
                                                 vehicleAgent, true);
        return resp.dump();
    }

    if (type == "find_hybrid_path") {
        const bool haveGraph = (roads && roads->ready()) || (pedRoads && pedRoads->ready());
        if (!haveGraph) return errorResp(id, "no node graph loaded").dump();
        Vec3 from, to;
        if (!parseVec(req["from"], from, err) || !parseVec(req["to"], to, err))
            return errorResp(id, err).dump();
        // Node-graph corridor with grounded waypoints: walking routes that
        // stay connected where the navmesh fragments. Prefers the pedestrian
        // graph (sidewalks) and falls back to the road graph, which is the
        // only one that spans cities.
        // "repair": false skips asking the navmesh to confirm each hop, for a
        // caller that would rather have the answer sooner than validated.
        const Pathfinder* mesh = req.value("repair", true) ? pathfinder : nullptr;
        RoutePlanner::HybridResult r =
            RoutePlanner::ComposeHybridRoute(pedRoads, roads, world, mesh, from, to);
        json wps = json::array();
        for (const auto& v : r.waypoints) wps.push_back(vecArr(v));
        const char* src = "none";
        switch (r.source) {
            case RoutePlanner::HybridResult::SourcePed: src = "ped"; break;
            case RoutePlanner::HybridResult::SourceVehicle: src = "vehicle"; break;
            case RoutePlanner::HybridResult::SourceStitched: src = "ped+vehicle"; break;
            default: break;
        }
        return json{{"type", "find_hybrid_path_result"}, {"id", id},
                    {"success", r.success}, {"waypoints", wps},
                    {"graph", src},
                    // How much of the route the navmesh confirmed as walkable.
                    // straight segments are where recovery mode is still needed.
                    {"repaired_segments", r.repairedSegments},
                    {"straight_segments", r.straightSegments},
                    // Longest single unconfirmed hop, which is what bounds how
                    // far a controller may have to cross on its own.
                    {"longest_unconfirmed", r.longestUnconfirmed},
                    // False when the walk stops short of the goal - typically a
                    // goal on another floor reached by an elevator, which the
                    // service cannot route. goal_gap is [horizontal, vertical]
                    // from the last waypoint to the requested goal; a large
                    // vertical with a small horizontal is that lift.
                    {"reached_goal", r.reachedGoal},
                    {"goal_gap", json::array({r.goalGapHoriz, r.goalGapVert})},
                    // Waypoints whose next step is a baked climb (stairs and
                    // ledges): move directly there, sliding will stall.
                    {"climb_at", r.climbAt}}.dump();
    }

    if (type == "find_boat_path") {
        if (!roads || !roads->ready())
            return errorResp(id, "road network not loaded").dump();
        if (!roads->hasLaneData())
            return errorResp(id, "boat routing needs the SA path files (--paths)").dump();
        Vec3 from, to;
        if (!parseVec(req["from"], from, err) || !parseVec(req["to"], to, err))
            return errorResp(id, err).dump();
        // SA keeps its boat network in the same files as the roads, as nodes
        // of type 2; it is one connected component of 1,507 nodes.
        RoadNetwork::RouteResult r = roads->findPath(from, to, RouteProfile::Boat());
        json wps = json::array();
        for (const auto& v : r.waypoints) wps.push_back(vecArr(v));
        return json{{"type", "find_boat_path_result"}, {"id", id},
                    {"success", r.success}, {"waypoints", wps}}.dump();
    }

    if (type == "find_vehicle_path") {
        if (!roads || !roads->ready())
            return errorResp(id, "road network not loaded").dump();
        Vec3 from, to;
        if (!parseVec(req["from"], from, err) || !parseVec(req["to"], to, err))
            return errorResp(id, err).dump();
        const RouteProfile profile = applyProfileOverrides(RouteProfile::Car(), req);
        const bool haveVehicle = req.contains("vehicle");
        const RoutePlanner::VehicleSpec veh =
            haveVehicle ? parseVehicle(req["vehicle"]) : RoutePlanner::VehicleSpec{};
        RoadNetwork::RouteResult r = roads->findPath(from, to, profile);
        if (!r.success || r.waypoints.empty())
            return json{{"type", "find_vehicle_path_result"}, {"id", id},
                        {"success", false}, {"waypoints", json::array()}}.dump();
        // The node route covers road to road. The legs from the caller's
        // positions to the end nodes are off-road straight lines - include
        // them as endpoints and report each leg's length and drivability so
        // the consumer knows when it is being asked to cross country.
        const Vec3& firstNode = r.waypoints.front();
        const Vec3& lastNode = r.waypoints.back();
        RoutePlanner::OffroadLeg legStart =
            RoutePlanner::CheckOffroadLeg(world, from, firstNode, veh);
        RoutePlanner::OffroadLeg legGoal =
            RoutePlanner::CheckOffroadLeg(world, lastNode, to, veh);

        // A straight off-road leg that the ground check rejects can often
        // still be driven - just not in a line. When a car-agent navmesh is
        // loaded, route such legs on it and splice the waypoints in, so the
        // returned route stays drivable end to end instead of asking the
        // consumer to cross country blind. Returns an empty vector when the
        // leg stays a straight line (drivable, no mesh, or no mesh route).
        auto spliceLeg = [&](const Vec3& a, const Vec3& b,
                             const RoutePlanner::OffroadLeg& straight) {
            std::vector<Vec3> none;
            if (straight.drivable || !vehiclePathfinder || !vehiclePathfinder->ready())
                return none;
            PathResult leg = vehiclePathfinder->FindPath(a, b, world);
            if (leg.success && !leg.partial && !leg.waypoints.empty())
                return leg.waypoints; // drivable detour through the car mesh
            return none;
        };
        const std::vector<Vec3> startLeg = spliceLeg(from, firstNode, legStart);
        const std::vector<Vec3> goalLeg = spliceLeg(lastNode, to, legGoal);
        const bool startRouted = !startLeg.empty();
        const bool goalRouted = !goalLeg.empty();

        // Waypoints and the lane count of the leg leaving each one are built
        // together: off-road and mesh-routed legs have no lane data, so they
        // report 0 rather than inheriting a neighbouring road's lanes.
        std::vector<Vec3> pts;
        std::vector<int> legLanes; // size pts.size() - 1
        auto push = [&](const Vec3& p, int laneFromPrev) {
            if (!pts.empty()) legLanes.push_back(laneFromPrev);
            pts.push_back(p);
        };
        // Straight legs carry the exact endpoints; mesh legs already start and
        // end on snapped positions adjacent to the node route, so the boundary
        // nodes are skipped to avoid duplicates.
        if (!startRouted && (from - firstNode).lengthSq() > 4.f) push(from, 0);
        for (const auto& v : startLeg) push(v, 0);
        const size_t firstIdx = startRouted ? 1 : 0;
        const size_t lastIdx = r.waypoints.size() - (goalRouted ? 1 : 0);
        for (size_t i = firstIdx; i < lastIdx; ++i) {
            // r.lanes[i-1] is the leg from node i-1 to node i; the first node
            // pushed is entered from an off-road leg, which has none.
            const int lane = (i > firstIdx && i - 1 < r.lanes.size())
                                 ? static_cast<int>(r.lanes[i - 1]) : 0;
            push(r.waypoints[i], lane);
        }
        for (const auto& v : goalLeg) push(v, 0);
        if (!goalRouted && (to - lastNode).lengthSq() > 4.f) push(to, 0);

        json wps = json::array();
        for (const auto& v : pts) wps.push_back(vecArr(v));
        json lanes = json::array();
        for (const int l : legLanes) lanes.push_back(l);

        json legS = {{"distance", legStart.distance}, {"drivable", legStart.drivable},
                     {"reason", legStart.reason},
                     {"routed", startRouted ? "mesh" : "straight"}};
        json legG = {{"distance", legGoal.distance}, {"drivable", legGoal.drivable},
                     {"reason", legGoal.reason},
                     {"routed", goalRouted ? "mesh" : "straight"}};
        json resp = {{"type", "find_vehicle_path_result"}, {"id", id},
                     {"success", r.success}, {"waypoints", wps},
                     // lanes[i] = lanes available driving from waypoint i to
                     // i+1, 0 where the network has no lane data for the leg.
                     {"lanes", lanes}, {"has_lane_data", roads->hasLaneData()},
                     {"offroad_start", legS}, {"offroad_goal", legG}};
        // The fit report costs a raycast per waypoint, so it is only produced
        // when the caller actually described a vehicle.
        if (haveVehicle)
            resp["vehicle_check"] = vehicleCheck(
                world, pts, veh, vehicleAgent,
                vehiclePathfinder && vehiclePathfinder->ready());
        return resp.dump();
    }

    if (type == "nearest_node") {
        // graph: "vehicle" (default) or "ped".
        const std::string which = req.value("graph", "vehicle");
        const RoadNetwork* g = (which == "ped") ? pedRoads : roads;
        if (which != "ped" && which != "vehicle")
            return errorResp(id, "graph must be \"vehicle\" or \"ped\"").dump();
        if (!g || !g->ready())
            return errorResp(id, which + " node graph not loaded").dump();
        Vec3 pos;
        if (!parseVec(req["pos"], pos, err))
            return errorResp(id, err).dump();
        const RouteProfile profile = applyProfileOverrides(
            (which == "ped") ? RouteProfile::Ped() : RouteProfile::Car(), req);
        const long node = g->nearestNode(pos, profile);
        const bool found = node >= 0;
        json resp = {{"type", "nearest_node_result"}, {"id", id}, {"found", found},
                     {"graph", which}};
        if (found) {
            const RoadNodeInfo& info = g->nodeInfo(node);
            resp["node"] = node;
            resp["pos"] = vecArr(g->nodePos(node));
            resp["distance"] = (g->nodePos(node) - pos).length();
            resp["flags"] = info.flags;
            // The class bits are only meaningful on vehicle nodes - a
            // pedestrian node reuses those bits for something else, so
            // reporting it as "emergency" would be a lie. Its raw flags are
            // still returned above for anyone who wants to experiment.
            if (which == "vehicle") {
                resp["highway"] = (info.flags & SaFlags::kHighway) != 0;
                resp["emergency"] = (info.flags & SaFlags::kEmergency) != 0;
                resp["parking"] = (info.flags & SaFlags::kParking) != 0;
                resp["boat"] = (info.flags & SaFlags::kBoat) != 0;
                resp["node_type"] = info.type;
            }
        }
        return resp.dump();
    }

    // --- world editing (RemoveBuilding / CreateObject awareness) ---------
    // Edits are recorded instantly and take effect on the next world_commit;
    // committing rebuilds the collision world and navmesh(es) in the
    // background, which takes minutes on the full map.
    if (type == "world_remove_object") {
        if (!editor) return errorResp(id, "world editing not enabled").dump();
        if (!req.contains("model") || !req.contains("radius"))
            return errorResp(id, "model and radius required").dump();
        Vec3 pos;
        if (!parseVec(req["pos"], pos, err)) return errorResp(id, err).dump();
        const long matched = editor->removeBuilding(
            static_cast<uint16_t>(req["model"].get<int>()), pos,
            req["radius"].get<float>(), err);
        if (matched < 0) return errorResp(id, err).dump();
        return json{{"type", "world_remove_object_result"}, {"id", id},
                    {"matched", matched}, {"pending_removes", editor->removeCount()}}.dump();
    }

    if (type == "world_add_object") {
        if (!editor) return errorResp(id, "world editing not enabled").dump();
        if (!req.contains("model")) return errorResp(id, "model required").dump();
        Vec3 pos;
        if (!parseVec(req["pos"], pos, err)) return errorResp(id, err).dump();
        // Rotation is optional and given as SA-MP euler degrees (rx, ry, rz).
        Quat rot;
        if (req.contains("rot")) {
            Vec3 euler;
            if (!parseVec(req["rot"], euler, err)) return errorResp(id, err).dump();
            rot = EulerDegreesToQuat(euler);
        }
        if (!editor->addObject(static_cast<uint16_t>(req["model"].get<int>()), pos, rot, err))
            return errorResp(id, err).dump();
        return json{{"type", "world_add_object_result"}, {"id", id},
                    {"pending_adds", editor->addCount()}}.dump();
    }

    if (type == "world_reset") {
        if (!editor) return errorResp(id, "world editing not enabled").dump();
        editor->reset();
        return json{{"type", "world_reset_result"}, {"id", id}}.dump();
    }

    if (type == "world_commit") {
        if (!editor) return errorResp(id, "world editing not enabled").dump();
        if (!editor->beginCommit(err)) return errorResp(id, err).dump();
        // Accepted, not finished: poll world_edits for committing=false.
        return json{{"type", "world_commit_result"}, {"id", id}, {"started", true}}.dump();
    }

    if (type == "world_edits") {
        if (!editor) return errorResp(id, "world editing not enabled").dump();
        return json{{"type", "world_edits_result"}, {"id", id},
                    {"removes", editor->removeCount()}, {"adds", editor->addCount()},
                    {"committing", editor->committing()}}.dump();
    }

    if (type == "status") {
        json resp = {{"type", "status_result"}, {"id", id}};
        resp["collision"] = world && !world->empty();
        resp["roads"] = roads && roads->ready();
        if (roads && roads->ready()) {
            resp["road_nodes"] = roads->nodeCount();
            resp["lane_data"] = roads->hasLaneData();
        }
        resp["ped_roads"] = pedRoads && pedRoads->ready();
        if (pedRoads && pedRoads->ready()) resp["ped_nodes"] = pedRoads->nodeCount();
        resp["world_editing"] = editor != nullptr;
        if (editor) {
            resp["pending_removes"] = editor->removeCount();
            resp["pending_adds"] = editor->addCount();
            resp["committing"] = editor->committing();
        }
        resp["navmesh"] = pathfinder && pathfinder->ready();
        if (world && !world->empty()) {
            resp["triangles"] = world->triangleCount();
            const Aabb& b = world->bounds();
            resp["bounds"] = {
                {"min", vecArr(b.min)},
                {"max", vecArr(b.max)},
            };
        }
        return resp.dump();
    }

    return errorResp(id, "unknown type: " + type).dump();
}

} // namespace wqs
