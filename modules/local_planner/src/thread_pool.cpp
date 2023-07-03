#include "local_planner/thread_pool.hpp"

namespace luhsoccer::local_planner {
ThreadPool::ThreadPool(size_t number_of_threads)
    : state(PoolState::CREATED), threads(number_of_threads), thread_pool_should_run(false) {}

ThreadPool::~ThreadPool() { this->stop(); }

bool ThreadPool::start(std::atomic_bool& should_run) {
    if (this->state != PoolState::CREATED) return false;
    this->thread_pool_should_run = true;
    for (auto& thread : this->threads) {
        thread = std::thread([this, &should_run]() { this->threadLoop(should_run, this->thread_pool_should_run); });
    }
    this->state = PoolState::RUNNING;
    return true;
}

bool ThreadPool::stop() {
    if (this->state != PoolState::RUNNING) return false;
    this->thread_pool_should_run = false;

    for (size_t i = 0; i < this->threads.size(); i++) {
        this->sem.release();
    }

    for (auto& thread : this->threads) {
        thread.join();
    }

    return true;
}

void ThreadPool::addJob(const std::function<void()>& job) {
    std::unique_lock lock(this->job_acquire_mtx);
    this->job_queue.push(job);
    this->sem.release();
}

void ThreadPool::threadLoop(std::atomic_bool& should_run, std::atomic_bool& thread_pool_should_run) {
    std::unique_lock lock(this->job_acquire_mtx, std::defer_lock);

    while (should_run && thread_pool_should_run) {
        this->sem.acquire();
        if (!should_run || !thread_pool_should_run) return;

        lock.lock();
        auto job = this->job_queue.front();
        this->job_queue.pop();
        lock.unlock();
        job();
    }
}

}  // namespace luhsoccer::local_planner