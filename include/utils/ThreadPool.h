#pragma once

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <memory>
#include <atomic>
#include <type_traits>

namespace CloudStream {

/**
 * @brief High-performance thread pool implementation with task queue
 */
class ThreadPool {
public:
    explicit ThreadPool(size_t num_threads = std::thread::hardware_concurrency());
    ~ThreadPool();
    
    // Disable copy
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    
    // Enable move
    ThreadPool(ThreadPool&&) = default;
    ThreadPool& operator=(ThreadPool&&) = default;
    
    /**
     * @brief Enqueue a task for execution
     * @tparam F Callable type
     * @tparam Args Argument types
     * @param f Function to execute
     * @param args Arguments to pass to the function
     * @return Future that will hold the result
     */
    template<typename F, typename... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::invoke_result_t<F, Args...>>;
    
    /**
     * @brief Enqueue a task with priority
     */
    template<typename F, typename... Args>
    auto enqueuePriority(int priority, F&& f, Args&&... args)
        -> std::future<typename std::invoke_result_t<F, Args...>>;
    
    /**
     * @brief Execute tasks in parallel with automatic partitioning
     */
    template<typename Iterator, typename Func>
    void parallel_for(Iterator begin, Iterator end, Func f);
    
    /**
     * @brief Map operation over a container in parallel
     */
    template<typename Container, typename Func>
    auto parallel_map(const Container& input, Func f) 
        -> std::vector<decltype(f(*input.begin()))>;
    
    // Control methods
    void pause();
    void resume();
    void stop();
    void wait();
    
    // Status
    size_t getNumThreads() const { return threads_.size(); }
    size_t getQueueSize() const;
    bool isPaused() const { return paused_; }
    bool isStopped() const { return stop_; }
    
    // Performance monitoring
    struct Statistics {
        std::atomic<uint64_t> tasks_completed{0};
        std::atomic<uint64_t> tasks_queued{0};
        std::atomic<uint64_t> total_wait_time_ms{0};
        std::atomic<uint64_t> total_execution_time_ms{0};
    };
    
    const Statistics& getStatistics() const { return stats_; }
    
private:
    struct Task {
        std::function<void()> func;
        int priority{0};
        
        bool operator<(const Task& other) const {
            return priority < other.priority;
        }
    };
    
    // Worker thread function
    void workerThread(size_t thread_id);
    
    // Thread management
    std::vector<std::thread> threads_;
    std::priority_queue<Task> tasks_;
    
    // Synchronization
    mutable std::mutex queue_mutex_;
    std::condition_variable condition_;
    std::condition_variable finished_condition_;
    
    // State
    std::atomic<bool> stop_{false};
    std::atomic<bool> paused_{false};
    std::atomic<size_t> active_tasks_{0};
    
    // Statistics
    Statistics stats_;
};

// Template implementations

template<typename F, typename... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args) 
    -> std::future<typename std::invoke_result_t<F, Args...>> {
    return enqueuePriority(0, std::forward<F>(f), std::forward<Args>(args)...);
}

template<typename F, typename... Args>
auto ThreadPool::enqueuePriority(int priority, F&& f, Args&&... args)
    -> std::future<typename std::invoke_result_t<F, Args...>> {
    using return_type = typename std::invoke_result_t<F, Args...>;
    
    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );
    
    std::future<return_type> res = task->get_future();
    
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        if(stop_) {
            throw std::runtime_error("enqueue on stopped ThreadPool");
        }
        
        tasks_.emplace(Task{[task](){ (*task)(); }, priority});
        stats_.tasks_queued++;
    }
    
    condition_.notify_one();
    return res;
}

template<typename Iterator, typename Func>
void ThreadPool::parallel_for(Iterator begin, Iterator end, Func f) {
    const size_t total_size = std::distance(begin, end);
    const size_t num_threads = threads_.size();
    const size_t chunk_size = (total_size + num_threads - 1) / num_threads;
    
    std::vector<std::future<void>> futures;
    futures.reserve(num_threads);
    
    for(size_t i = 0; i < num_threads && begin != end; ++i) {
        Iterator chunk_begin = begin;
        Iterator chunk_end = begin;
        
        std::advance(chunk_end, std::min(chunk_size, 
                                        static_cast<size_t>(std::distance(begin, end))));
        
        futures.push_back(enqueue([chunk_begin, chunk_end, f]() {
            for(auto it = chunk_begin; it != chunk_end; ++it) {
                f(*it);
            }
        }));
        
        begin = chunk_end;
    }
    
    for(auto& future : futures) {
        future.wait();
    }
}

template<typename Container, typename Func>
auto ThreadPool::parallel_map(const Container& input, Func f) 
    -> std::vector<decltype(f(*input.begin()))> {
    using ResultType = decltype(f(*input.begin()));
    
    std::vector<std::future<ResultType>> futures;
    futures.reserve(input.size());
    
    for(const auto& item : input) {
        futures.push_back(enqueue(f, item));
    }
    
    std::vector<ResultType> results;
    results.reserve(input.size());
    
    for(auto& future : futures) {
        results.push_back(future.get());
    }
    
    return results;
}

} // namespace CloudStream 