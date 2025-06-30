#pragma once

#include <chrono>
#include <string>

namespace CloudStream {

class Timer {
public:
    Timer() : start_time_(std::chrono::high_resolution_clock::now()) {}
    
    void reset() {
        start_time_ = std::chrono::high_resolution_clock::now();
    }
    
    double elapsed() const {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time_);
        return duration.count() / 1000000.0;
    }
    
    double elapsedMilliseconds() const {
        return elapsed() * 1000.0;
    }
    
private:
    std::chrono::high_resolution_clock::time_point start_time_;
};

class ScopedTimer {
public:
    explicit ScopedTimer(const std::string& name) : name_(name), timer_() {}
    
    ~ScopedTimer() {
        double elapsed = timer_.elapsedMilliseconds();
        // Log timing info
    }
    
private:
    std::string name_;
    Timer timer_;
};

} // namespace CloudStream 