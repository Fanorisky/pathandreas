#pragma once

#include "collision_world/collision_world.h"
#include "pathfinder/pathfinder.h"
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
    QueryServer(CollisionWorld* world, Pathfinder* pathfinder, const ServerConfig& cfg);
    ~QueryServer();

    // Blocking serve loop. Returns after stop().
    int run();
    void stop();

private:
    CollisionWorld* world_;
    Pathfinder* pathfinder_;
    ServerConfig cfg_;
    std::atomic<bool> running_{false};
    int listenFd_ = -1;
};

} // namespace wqs
