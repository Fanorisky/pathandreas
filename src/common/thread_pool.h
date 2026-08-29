#pragma once

#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>
#include <atomic>
#include <future>

namespace wqs {

class ThreadPool {
public:
    explicit ThreadPool(unsigned n = 0) {
        if (n == 0) n = std::max(2u, std::thread::hardware_concurrency());
        stop_ = false;
        workers_.reserve(n);
        for (unsigned i = 0; i < n; ++i) {
            workers_.emplace_back([this] {
                for (;;) {
                    std::function<void()> job;
                    {
                        std::unique_lock<std::mutex> lk(mu_);
                        cv_.wait(lk, [this] { return stop_ || !jobs_.empty(); });
                        if (stop_ && jobs_.empty()) return;
                        job = std::move(jobs_.front());
                        jobs_.pop();
                    }
                    job();
                }
            });
        }
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lk(mu_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& t : workers_) t.join();
    }

    template <class F>
    auto submit(F&& f) -> std::future<decltype(f())> {
        using R = decltype(f());
        auto task = std::make_shared<std::packaged_task<R()>>(std::forward<F>(f));
        auto fut = task->get_future();
        {
            std::lock_guard<std::mutex> lk(mu_);
            jobs_.emplace([task] { (*task)(); });
        }
        cv_.notify_one();
        return fut;
    }

    unsigned size() const { return static_cast<unsigned>(workers_.size()); }

private:
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> jobs_;
    std::mutex mu_;
    std::condition_variable cv_;
    bool stop_ = false;
};

} // namespace wqs
