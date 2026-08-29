#pragma once

#include <cstdio>
#include <cstdarg>
#include <mutex>

namespace wqs {

inline std::mutex& logMutex() {
    static std::mutex m;
    return m;
}

inline void logf(const char* level, const char* fmt, ...) {
    std::lock_guard<std::mutex> lock(logMutex());
    std::fprintf(stderr, "[%s] ", level);
    va_list ap;
    va_start(ap, fmt);
    std::vfprintf(stderr, fmt, ap);
    va_end(ap);
    std::fputc('\n', stderr);
}

#define WQS_INFO(...)  ::wqs::logf("info",  __VA_ARGS__)
#define WQS_WARN(...)  ::wqs::logf("warn",  __VA_ARGS__)
#define WQS_ERROR(...) ::wqs::logf("error", __VA_ARGS__)

} // namespace wqs
