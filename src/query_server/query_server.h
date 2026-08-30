#pragma once

#include "collision_world/collision_world.h"
#include "pathfinder/pathfinder.h"
#include "road_network/road_network.h"
#include "query_server/world_editor.h"
#include "common/thread_pool.h"
#include <atomic>
#include <cstdint>
#include <string>

namespace wqs {

struct ServerConfig {
    std::string bind = "0.0.0.0";
    uint16_t port = 8090;
    unsigned threads = 0; // 0 = hardware concurrency
};

class QueryServer {
public:
    // `backends` is owned by the caller and must outlive the server; a world
    // commit swaps its contents while the server keeps serving.
    QueryServer(Backends* backends, const ServerConfig& cfg, WorldEditor* editor = nullptr);
    ~QueryServer();

    // Blocking serve loop. Returns after stop().
    int run();
    void stop();

private:
    Backends* backends_;
    WorldEditor* editor_;
    ServerConfig cfg_;
    std::atomic<bool> running_{false};
    int listenFd_ = -1;
};

} // namespace wqs
