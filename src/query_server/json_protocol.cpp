#include "query_server/json_protocol.h"
#include "nlohmann/json.hpp"
#include "road_network/road_network.h"
#include "route_planner/route_planner.h"

using json = nlohmann::json;

namespace wqs {
namespace {

json vecArr(const Vec3& v) { return json::array({v.x, v.y, v.z}); }

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

} // namespace

std::string HandleQueryJson(const std::string& request,
                            const CollisionWorld* world,
                            const Pathfinder* pathfinder,
                            const RoadNetwork* roads) {
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
                    {"partial", p.partial}, {"waypoints", wps}}.dump();
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

    if (type == "find_hybrid_path") {
        if (!roads || !roads->ready())
            return errorResp(id, "road network not loaded").dump();
        Vec3 from, to;
        if (!parseVec(req["from"], from, err) || !parseVec(req["to"], to, err))
            return errorResp(id, err).dump();
        // Road-network corridor with grounded waypoints: long-distance
        // walking routes that stay connected where the navmesh fragments.
        RoutePlanner::HybridResult r = RoutePlanner::ComposeHybridRoute(*roads, world, from, to);
        json wps = json::array();
        for (const auto& v : r.waypoints) wps.push_back(vecArr(v));
        return json{{"type", "find_hybrid_path_result"}, {"id", id},
                    {"success", r.success}, {"waypoints", wps}}.dump();
    }

    if (type == "find_vehicle_path") {
        if (!roads || !roads->ready())
            return errorResp(id, "road network not loaded").dump();
        Vec3 from, to;
        if (!parseVec(req["from"], from, err) || !parseVec(req["to"], to, err))
            return errorResp(id, err).dump();
        RoadNetwork::RouteResult r = roads->findPath(from, to);
        if (!r.success || r.waypoints.empty())
            return json{{"type", "find_vehicle_path_result"}, {"id", id},
                        {"success", false}, {"waypoints", json::array()}}.dump();
        // The node route covers road to road. The legs from the caller's
        // positions to the end nodes are off-road straight lines - include
        // them as endpoints and report each leg's length and drivability so
        // the consumer knows when it is being asked to cross country.
        const Vec3& firstNode = r.waypoints.front();
        const Vec3& lastNode = r.waypoints.back();
        RoutePlanner::OffroadLeg legStart = RoutePlanner::CheckOffroadLeg(world, from, firstNode);
        RoutePlanner::OffroadLeg legGoal = RoutePlanner::CheckOffroadLeg(world, lastNode, to);
        json wps = json::array();
        if ((from - firstNode).lengthSq() > 4.f) wps.push_back(vecArr(from));
        for (const auto& v : r.waypoints) wps.push_back(vecArr(v));
        if ((to - lastNode).lengthSq() > 4.f) wps.push_back(vecArr(to));
        json legS = {{"distance", legStart.distance}, {"drivable", legStart.drivable}};
        json legG = {{"distance", legGoal.distance}, {"drivable", legGoal.drivable}};
        return json{{"type", "find_vehicle_path_result"}, {"id", id},
                    {"success", r.success}, {"waypoints", wps},
                    {"offroad_start", legS}, {"offroad_goal", legG}}.dump();
    }

    if (type == "nearest_node") {
        if (!roads || !roads->ready())
            return errorResp(id, "road network not loaded").dump();
        Vec3 pos;
        if (!parseVec(req["pos"], pos, err))
            return errorResp(id, err).dump();
        const long node = roads->nearestNode(pos);
        const bool found = node >= 0;
        json resp = {{"type", "nearest_node_result"}, {"id", id}, {"found", found}};
        if (found) {
            resp["node"] = node;
            resp["pos"] = vecArr(roads->nodePos(node));
        }
        return resp.dump();
    }

    if (type == "status") {
        json resp = {{"type", "status_result"}, {"id", id}};
        resp["collision"] = world && !world->empty();
        resp["roads"] = roads && roads->ready();
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
