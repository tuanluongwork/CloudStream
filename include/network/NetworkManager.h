#pragma once

#include <boost/asio.hpp>
#include <memory>
#include <thread>

namespace CloudStream {

class NetworkManager {
public:
    static NetworkManager& instance() {
        static NetworkManager manager;
        return manager;
    }
    
    boost::asio::io_context& getIOContext() { return *io_context_; }
    
    void start();
    void stop();
    bool isRunning() const { return running_; }
    
private:
    NetworkManager();
    ~NetworkManager();
    
    NetworkManager(const NetworkManager&) = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;
    
    std::unique_ptr<boost::asio::io_context> io_context_;
    std::unique_ptr<boost::asio::io_context::work> work_;
    std::vector<std::thread> io_threads_;
    std::atomic<bool> running_{false};
};

} // namespace CloudStream 