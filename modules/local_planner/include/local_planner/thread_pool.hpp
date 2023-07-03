#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <queue>
#include <thread>
#include <vector>

namespace luhsoccer::local_planner {

class Semaphore {
    std::mutex mutex;
    std::condition_variable condition;
    unsigned long count = 0;  // Initialized as locked.

   public:
    void release() {
        std::lock_guard lock(this->mutex);
        this->count++;
        this->condition.notify_one();
    }

    void acquire() {
        std::unique_lock lock(this->mutex);
        while (this->count == 0)  // Handle spurious wake-ups.
            this->condition.wait(lock);
        this->count--;
    }

    bool tryAcquire() {
        std::lock_guard lock(this->mutex);
        if (this->count) {
            this->count--;
            return true;
        }
        return false;
    }
};

class ThreadPool {
   public:
    ThreadPool(size_t number_of_threads = std::thread::hardware_concurrency() / 2);
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool& operator=(ThreadPool&&) = delete;
    ~ThreadPool();
    bool start(std::atomic_bool& should_run);

    bool stop();

    void addJob(const std::function<void()>& job);

   private:
    enum class PoolState { CREATED, RUNNING, STOPPED };
    PoolState state;
    void threadLoop(std::atomic_bool& should_run, std::atomic_bool& thread_pool_should_run);

    std::vector<std::thread> threads;
    Semaphore sem;
    std::mutex job_acquire_mtx;
    std::queue<std::function<void()>> job_queue;
    std::atomic_bool thread_pool_should_run;
};
}  // namespace luhsoccer::local_planner