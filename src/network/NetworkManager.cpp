#include "network/NetworkManager.h"
#include "utils/Logger.h"
#include <thread>

namespace CloudStream {

NetworkManager::NetworkManager() 
    : io_context_(std::make_unique<boost::asio::io_context>())
    , work_(std::make_unique<boost::asio::io_context::work>(*io_context_)) {
}

NetworkManager::~NetworkManager() {
    stop();
}

void NetworkManager::start() {
    if (running_) {
        return;
    }
    
    running_ = true;
    
    // Start IO threads
    size_t thread_count = std::thread::hardware_concurrency();
    for (size_t i = 0; i < thread_count; ++i) {
        io_threads_.emplace_back([this]() {
            try {
                io_context_->run();
            } catch (const std::exception& e) {
                Logger::instance().error("NetworkManager IO thread error: {}", e.what());
            }
        });
    }
    
    Logger::instance().info("NetworkManager started with {} IO threads", thread_count);
}

void NetworkManager::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    // Stop work and context
    work_.reset();
    io_context_->stop();
    
    // Wait for threads to finish
    for (auto& thread : io_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    io_threads_.clear();
    
    Logger::instance().info("NetworkManager stopped");
}

} // namespace CloudStream 