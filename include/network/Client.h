#pragma once

#include <boost/asio.hpp>
#include <memory>
#include <functional>
#include <queue>
#include <atomic>
#include "network/StreamProtocol.h"
#include "core/PointCloud.h"

namespace CloudStream {

class Client {
public:
    using ConnectedCallback = std::function<void(bool success, const std::string& error)>;
    using PointCloudCallback = std::function<void(PointCloud::Ptr)>;
    using StatusCallback = std::function<void(const Protocol::StatusUpdate&)>;
    
    Client();
    ~Client();
    
    // Connection management
    void connect(const std::string& host, uint16_t port, ConnectedCallback callback);
    void disconnect();
    bool isConnected() const { return connected_; }
    
    // Callbacks
    void setPointCloudCallback(PointCloudCallback callback) { 
        point_cloud_callback_ = callback; 
    }
    void setStatusCallback(StatusCallback callback) { 
        status_callback_ = callback; 
    }
    
    // Control commands
    void requestStream();
    void pauseStream();
    void resumeStream();
    void setQuality(float quality);
    void setTargetFPS(uint32_t fps);
    
    // Statistics
    Protocol::StatusUpdate getLastStatus() const { return last_status_; }
    
private:
    void handleConnect(const boost::system::error_code& error);
    void handleRead();
    void handleWrite();
    void processMessage(const MessageParser::ParseResult& result);
    void sendMessage(const std::vector<uint8_t>& data);
    
    // Networking
    std::unique_ptr<boost::asio::io_context> io_context_;
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_;
    std::thread io_thread_;
    
    // Protocol
    MessageBuilder message_builder_;
    MessageParser message_parser_;
    std::vector<uint8_t> read_buffer_;
    std::queue<std::vector<uint8_t>> write_queue_;
    std::mutex write_mutex_;
    
    // State
    std::atomic<bool> connected_{false};
    std::string client_id_;
    
    // Callbacks
    ConnectedCallback connect_callback_;
    PointCloudCallback point_cloud_callback_;
    StatusCallback status_callback_;
    
    // Point cloud assembly
    std::unique_ptr<Protocol::PointCloudHeader> current_header_;
    std::map<uint32_t, std::vector<uint8_t>> received_chunks_;
    
    // Statistics
    Protocol::StatusUpdate last_status_;
    std::chrono::steady_clock::time_point last_update_time_;
};

} // namespace CloudStream 