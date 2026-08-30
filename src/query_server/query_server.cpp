#include "query_server/query_server.h"
#include <shared_mutex>
#include "query_server/json_protocol.h"
#include "query_server/sha1.h"
#include "common/log.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>

#include <cstring>
#include <string>
#include <vector>
#include <mutex>
#include <algorithm>
#include <cctype>
#include <memory>
#include <thread>
#include <atomic>

namespace wqs {
namespace {

int setNonBlock(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

std::string headerValue(const std::string& headers, const std::string& key) {
    std::string lower = headers;
    std::string k = key;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });
    std::transform(k.begin(), k.end(), k.begin(), [](unsigned char c) { return std::tolower(c); });
    auto pos = lower.find(k + ":");
    if (pos == std::string::npos) return {};
    pos += k.size() + 1;
    while (pos < headers.size() && (headers[pos] == ' ' || headers[pos] == '\t')) ++pos;
    auto end = headers.find("\r\n", pos);
    if (end == std::string::npos) end = headers.size();
    return headers.substr(pos, end - pos);
}

bool sendAll(int fd, const char* p, size_t n) {
    size_t sent = 0;
    while (sent < n) {
        ssize_t r = ::send(fd, p + sent, n - sent, MSG_NOSIGNAL);
        if (r < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                pollfd pfd{fd, POLLOUT, 0};
                if (poll(&pfd, 1, 5000) <= 0) return false;
                continue;
            }
            return false;
        }
        sent += static_cast<size_t>(r);
    }
    return true;
}

std::string wsFrame(const std::string& payload, uint8_t opcode = 0x1) {
    std::string f;
    f.push_back(static_cast<char>(0x80 | opcode));
    const size_t n = payload.size();
    if (n < 126) {
        f.push_back(static_cast<char>(n));
    } else if (n <= 0xffff) {
        f.push_back(126);
        f.push_back(static_cast<char>((n >> 8) & 0xff));
        f.push_back(static_cast<char>(n & 0xff));
    } else {
        f.push_back(127);
        for (int i = 7; i >= 0; --i) f.push_back(static_cast<char>((n >> (i * 8)) & 0xff));
    }
    f += payload;
    return f;
}

// Returns decoded payload, or empty with *closed=true / *needMore=true.
std::string wsDecode(std::string& buf, bool& closed, bool& needMore, bool& isPing) {
    closed = false;
    needMore = false;
    isPing = false;
    if (buf.size() < 2) {
        needMore = true;
        return {};
    }
    const uint8_t b0 = static_cast<uint8_t>(buf[0]);
    const uint8_t b1 = static_cast<uint8_t>(buf[1]);
    const uint8_t opcode = b0 & 0x0f;
    const bool masked = (b1 & 0x80) != 0;
    uint64_t len = b1 & 0x7f;
    size_t hdr = 2;
    if (len == 126) {
        if (buf.size() < 4) { needMore = true; return {}; }
        len = (static_cast<uint8_t>(buf[2]) << 8) | static_cast<uint8_t>(buf[3]);
        hdr = 4;
    } else if (len == 127) {
        if (buf.size() < 10) { needMore = true; return {}; }
        len = 0;
        for (int i = 0; i < 8; ++i) len = (len << 8) | static_cast<uint8_t>(buf[2 + i]);
        hdr = 10;
    }
    const size_t maskOff = hdr;
    if (masked) hdr += 4;
    if (buf.size() < hdr + len) { needMore = true; return {}; }
    std::string payload = buf.substr(hdr, static_cast<size_t>(len));
    if (masked) {
        uint8_t m[4];
        for (int i = 0; i < 4; ++i) m[i] = static_cast<uint8_t>(buf[maskOff + i]);
        for (size_t i = 0; i < payload.size(); ++i)
            payload[i] = static_cast<char>(static_cast<uint8_t>(payload[i]) ^ m[i % 4]);
    }
    buf.erase(0, hdr + static_cast<size_t>(len));
    if (opcode == 0x8) { closed = true; return {}; }
    if (opcode == 0x9) { isPing = true; return payload; }
    if (opcode == 0xA) return {}; // pong
    return payload;
}

// Every request reads the backends under a shared lock, so an in-flight query
// always completes on the world it started with; a world commit takes the
// exclusive lock only to swap the pointers (see WorldCommitter).
std::string answer(const std::string& request, Backends& b, WorldEditor* editor) {
    std::shared_lock<std::shared_mutex> lk(b.mu);
    return HandleQueryJson(request, b.world.get(), b.pathfinder.get(), b.roads,
                           b.vehiclePathfinder.get(), editor);
}

void handleClient(int fd, Backends& backends, WorldEditor* editor, ThreadPool& pool) {
    struct Conn {
        int fd = -1;
        std::mutex writeMu;
        std::atomic<int> inflight{0};
        std::atomic<bool> dead{false};
        bool writeRaw(const std::string& s) {
            if (dead.load()) return false;
            std::lock_guard<std::mutex> lk(writeMu);
            if (dead.load()) return false;
            return sendAll(fd, s.data(), s.size());
        }
    };
    auto conn = std::make_shared<Conn>();
    conn->fd = fd;

    std::string buf;
    char tmp[4096];
    bool upgraded = false;

    for (;;) {
        ssize_t n = ::recv(fd, tmp, sizeof(tmp), 0);
        if (n == 0) break;
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                pollfd pfd{fd, POLLIN, 0};
                int pr = poll(&pfd, 1, 120000);
                if (pr <= 0) break;
                continue;
            }
            break;
        }
        buf.append(tmp, static_cast<size_t>(n));

        if (!upgraded) {
            auto pos = buf.find("\r\n\r\n");
            if (pos == std::string::npos) {
                if (buf.size() > 65536) break;
                continue;
            }
            std::string headers = buf.substr(0, pos + 4);
            buf.erase(0, pos + 4);

            const std::string upgrade = headerValue(headers, "Upgrade");
            const bool isWs = upgrade == "websocket" || upgrade == "WebSocket";
            const std::string key = headerValue(headers, "Sec-WebSocket-Key");

            if (isWs && !key.empty()) {
                const std::string accept = sha1::acceptKey(key);
                std::string resp =
                    "HTTP/1.1 101 Switching Protocols\r\n"
                    "Upgrade: websocket\r\n"
                    "Connection: Upgrade\r\n"
                    "Sec-WebSocket-Accept: " + accept + "\r\n\r\n";
                conn->writeRaw(resp);
                upgraded = true;
            } else {
                std::string body, ctype = "text/plain; charset=utf-8";
                if (headers.rfind("GET /health", 0) == 0 || headers.rfind("GET /health ", 0) == 0) {
                    body = answer("{\"type\":\"status\",\"id\":\"health\"}", backends, editor);
                    ctype = "application/json";
                } else if (headers.rfind("POST /query", 0) == 0 || headers.rfind("POST /query ", 0) == 0) {
                    // The body may not have arrived yet when the headers are complete -
                    // clients such as cpp-httplib send headers and body as separate TCP
                    // writes, so at this point `buf` can legitimately be empty. Read
                    // Content-Length bytes before parsing; anything else parses "" and
                    // answers "[json.exception] attempting to parse an empty input".
                    const std::string clStr = headerValue(headers, "Content-Length");
                    size_t want = 0;
                    if (!clStr.empty()) want = static_cast<size_t>(strtoull(clStr.c_str(), nullptr, 10));
                    while (buf.size() < want) {
                        ssize_t n = ::recv(fd, tmp, static_cast<size_t>(sizeof(tmp)), 0);
                        if (n == 0) break;
                        if (n < 0) {
                            if (errno == EINTR) continue;
                            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                                pollfd pfd{fd, POLLIN, 0};
                                if (poll(&pfd, 1, 30000) <= 0) break;
                                continue;
                            }
                            break;
                        }
                        buf.append(tmp, static_cast<size_t>(n));
                    }
                    if (buf.size() > want) buf.resize(want);
                    body = answer(buf, backends, editor);
                    ctype = "application/json";
                    buf.clear();
                } else {
                    body = "PathAndreas\n"
                           "WebSocket JSON: raycast, find_ground_z, find_path, find_hybrid_path,\n"
                           "  find_vehicle_path, find_offroad_path, move_along_surface,\n"
                           "  nearest_node, world_* (editing), status\n"
                           "HTTP POST /query  same JSON body\n"
                           "HTTP GET  /health\n";
                }
                std::string resp = "HTTP/1.1 200 OK\r\n"
                    "Content-Type: " + ctype + "\r\n"
                    "Content-Length: " + std::to_string(body.size()) + "\r\n"
                    "Access-Control-Allow-Origin: *\r\n"
                    "Connection: close\r\n\r\n" + body;
                conn->writeRaw(resp);
                break;
            }
        }

        if (upgraded) {
            for (;;) {
                bool closed = false, needMore = false, isPing = false;
                std::string payload = wsDecode(buf, closed, needMore, isPing);
                if (needMore) break;
                if (closed) {
                    conn->writeRaw(wsFrame("", 0x8));
                    goto done;
                }
                if (isPing) {
                    conn->writeRaw(wsFrame(payload, 0xA));
                    continue;
                }
                if (payload.empty()) continue;
                conn->inflight.fetch_add(1);
                pool.submit([payload, &backends, editor, conn] {
                    std::string resp = answer(payload, backends, editor);
                    conn->writeRaw(wsFrame(resp, 0x1));
                    conn->inflight.fetch_sub(1);
                });
            }
        }
    }
done:
    conn->dead.store(true);
    while (conn->inflight.load() > 0) {
        pollfd wait{fd, 0, 0};
        poll(&wait, 1, 10);
    }
    ::close(fd);
}

} // namespace

QueryServer::QueryServer(Backends* backends, const ServerConfig& cfg, WorldEditor* editor)
    : backends_(backends), editor_(editor), cfg_(cfg) {}

QueryServer::~QueryServer() { stop(); }

void QueryServer::stop() {
    running_ = false;
    if (listenFd_ >= 0) {
        ::shutdown(listenFd_, SHUT_RDWR);
        ::close(listenFd_);
        listenFd_ = -1;
    }
}

int QueryServer::run() {
    signal(SIGPIPE, SIG_IGN);
    listenFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd_ < 0) {
        WQS_ERROR("socket: %s", strerror(errno));
        return 1;
    }
    int yes = 1;
    setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#ifdef SO_REUSEPORT
    setsockopt(listenFd_, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes));
#endif
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(cfg_.port);
    if (cfg_.bind == "0.0.0.0" || cfg_.bind.empty()) addr.sin_addr.s_addr = INADDR_ANY;
    else inet_pton(AF_INET, cfg_.bind.c_str(), &addr.sin_addr);

    if (bind(listenFd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        WQS_ERROR("bind %s:%u: %s", cfg_.bind.c_str(), cfg_.port, strerror(errno));
        return 1;
    }
    if (listen(listenFd_, 128) < 0) {
        WQS_ERROR("listen: %s", strerror(errno));
        return 1;
    }
    setNonBlock(listenFd_);
    running_ = true;
    ThreadPool pool(cfg_.threads);
    WQS_INFO("Query server listening on %s:%u (%u threads)",
             cfg_.bind.c_str(), cfg_.port, pool.size());

    while (running_) {
        pollfd pfd{listenFd_, POLLIN, 0};
        int pr = poll(&pfd, 1, 500);
        if (pr < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (pr == 0) continue;
        sockaddr_in cli{};
        socklen_t cl = sizeof(cli);
        int fd = accept(listenFd_, reinterpret_cast<sockaddr*>(&cli), &cl);
        if (fd < 0) continue;
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        // Connection I/O lives on its own thread so the query pool never blocks on recv.
        std::thread([fd, this, &pool] {
            handleClient(fd, *backends_, editor_, pool);
        }).detach();
    }
    return 0;
}

} // namespace wqs
