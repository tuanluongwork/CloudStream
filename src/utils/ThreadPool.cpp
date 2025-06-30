#include "utils/ThreadPool.h"

namespace CloudStream {

ThreadPool::ThreadPool(size_t num_threads) {
    for (size_t i = 0; i < num_threads; ++i) {
        threads_.emplace_back(&ThreadPool::workerThread, this, i);
    }
}

ThreadPool::~ThreadPool() {
    stop();
}

void ThreadPool::workerThread(size_t thread_id) {
    while (true) {
        Task task;
        
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            
            condition_.wait(lock, [this] { 
                return stop_ || paused_ || !tasks_.empty(); 
            });
            
            if (stop_ && tasks_.empty()) {
                return;
            }
            
            if (paused_) {
                continue;
            }
            
            if (!tasks_.empty()) {
                task = std::move(const_cast<Task&>(tasks_.top()));
                tasks_.pop();
                active_tasks_++;
            }
        }
        
        if (task.func) {
            auto start_time = std::chrono::steady_clock::now();
            task.func();
            auto end_time = std::chrono::steady_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                end_time - start_time).count();
            
            stats_.tasks_completed++;
            stats_.total_execution_time_ms += duration;
            
            active_tasks_--;
            finished_condition_.notify_all();
        }
    }
}

void ThreadPool::pause() {
    paused_ = true;
}

void ThreadPool::resume() {
    paused_ = false;
    condition_.notify_all();
}

void ThreadPool::stop() {
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        stop_ = true;
    }
    
    condition_.notify_all();
    
    for (auto& thread : threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}

void ThreadPool::wait() {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    finished_condition_.wait(lock, [this] {
        return tasks_.empty() && active_tasks_ == 0;
    });
}

size_t ThreadPool::getQueueSize() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return tasks_.size();
}

} // namespace CloudStream 