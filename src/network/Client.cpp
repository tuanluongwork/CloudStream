#include "network/Client.h"
#include "utils/Logger.h"
#include <boost/asio/connect.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/read.hpp>

namespace CloudStream {

using boost::asio::ip::tcp;

Client::Client() 
    : io_context_(std::make_unique<boost::asio::io_context>())
    , socket_(std::make_unique<tcp::socket>(*io_context_))
    , read_buffer_(Protocol::MAX_PACKET_SIZE) {
    
    // Generate unique client ID
    client_id_ = "CloudStreamClient_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
}

Client::~Client() {
    disconnect();
}

void Client::connect(const std::string& host, uint16_t port, ConnectedCallback callback) {
    if (connected_) {
        callback(false, "Already connected");
        return;
    }
    
    connect_callback_ = callback;
    
    Logger::instance().info("Connecting to {}:{}", host, port);
    
    // Resolve endpoint
    tcp::resolver resolver(*io_context_);
    auto endpoints = resolver.resolve(host, std::to_string(port));
    
    // Async connect
    boost::asio::async_connect(
        *socket_,
        endpoints,
        [this](boost::system::error_code ec, tcp::endpoint) {
            handleConnect(ec);
        });
    
    // Start IO thread
    io_thread_ = std::thread([this]() {
        try {
            io_context_->run();
        } catch (const std::exception& e) {
            Logger::instance().error("Client IO thread error: {}", e.what());
        }
    });
}

void Client::disconnect() {
    if (!connected_) return;
    
    connected_ = false;
    
    Logger::instance().info("Disconnecting client");
    
    // Close socket
    boost::system::error_code ec;
    socket_->shutdown(tcp::socket::shutdown_both, ec);
    socket_->close(ec);
    
    // Stop IO context
    io_context_->stop();
    
    // Wait for IO thread
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
    
    // Reset state
    io_context_->restart();
    socket_ = std::make_unique<tcp::socket>(*io_context_);
    message_parser_.reset();
    current_header_.reset();
    received_chunks_.clear();
}

void Client::requestStream() {
    if (!connected_) return;
    
    auto msg = message_builder_.buildControlCommand(Protocol::ControlCommand::START_STREAM);
    sendMessage(msg);
}

void Client::pauseStream() {
    if (!connected_) return;
    
    auto msg = message_builder_.buildControlCommand(Protocol::ControlCommand::PAUSE_STREAM);
    sendMessage(msg);
}

void Client::resumeStream() {
    if (!connected_) return;
    
    auto msg = message_builder_.buildControlCommand(Protocol::ControlCommand::RESUME_STREAM);
    sendMessage(msg);
}

void Client::setQuality(float quality) {
    if (!connected_) return;
    
    std::vector<uint8_t> params(sizeof(float));
    std::memcpy(params.data(), &quality, sizeof(float));
    
    auto msg = message_builder_.buildControlCommand(Protocol::ControlCommand::SET_QUALITY, params);
    sendMessage(msg);
}

void Client::setTargetFPS(uint32_t fps) {
    if (!connected_) return;
    
    std::vector<uint8_t> params(sizeof(uint32_t));
    std::memcpy(params.data(), &fps, sizeof(uint32_t));
    
    auto msg = message_builder_.buildControlCommand(Protocol::ControlCommand::SET_FPS, params);
    sendMessage(msg);
}

void Client::handleConnect(const boost::system::error_code& error) {
    if (!error) {
        connected_ = true;
        Logger::instance().info("Connected successfully");
        
        // Send handshake
        auto handshake = message_builder_.buildHandshake(client_id_);
        sendMessage(handshake);
        
        // Start reading
        handleRead();
        
        // Notify callback
        if (connect_callback_) {
            connect_callback_(true, "");
        }
    } else {
        Logger::instance().error("Connection failed: {}", error.message());
        
        if (connect_callback_) {
            connect_callback_(false, error.message());
        }
    }
}

void Client::handleRead() {
    if (!connected_) return;
    
    socket_->async_read_some(
        boost::asio::buffer(read_buffer_),
        [this](boost::system::error_code ec, size_t bytes_transferred) {
            if (!ec && bytes_transferred > 0) {
                message_parser_.addData(read_buffer_.data(), bytes_transferred);
                
                while (message_parser_.hasCompleteMessage()) {
                    auto result = message_parser_.parseNextMessage();
                    if (result.success) {
                        processMessage(result);
                    }
                }
                
                handleRead();
            } else {
                if (ec != boost::asio::error::operation_aborted) {
                    Logger::instance().error("Read error: {}", ec.message());
                    connected_ = false;
                }
            }
        });
}

void Client::handleWrite() {
    if (!connected_ || write_queue_.empty()) return;
    
    const auto& data = write_queue_.front();
    
    boost::asio::async_write(
        *socket_,
        boost::asio::buffer(data),
        [this](boost::system::error_code ec, size_t /*bytes_transferred*/) {
            if (!ec) {
                std::lock_guard<std::mutex> lock(write_mutex_);
                write_queue_.pop();
                
                if (!write_queue_.empty()) {
                    handleWrite();
                }
            } else {
                Logger::instance().error("Write error: {}", ec.message());
                connected_ = false;
            }
        });
}

void Client::processMessage(const MessageParser::ParseResult& result) {
    switch (result.type) {
        case Protocol::MessageType::HANDSHAKE_ACK: {
            bool accepted = !result.payload.empty() && result.payload[0] == 1;
            Logger::instance().info("Handshake {}", accepted ? "accepted" : "rejected");
            
            if (accepted) {
                // Request initial stream
                requestStream();
            }
            break;
        }
        
        case Protocol::MessageType::POINT_CLOUD_HEADER: {
            current_header_ = std::make_unique<Protocol::PointCloudHeader>();
            if (MessageParser::parsePointCloudHeader(result.payload, *current_header_)) {
                Logger::instance().info("Receiving point cloud with {} points in {} chunks",
                                      current_header_->total_points, current_header_->chunk_count);
                received_chunks_.clear();
            }
            break;
        }
        
        case Protocol::MessageType::POINT_CLOUD_DATA: {
            if (!current_header_) break;
            
            uint32_t chunk_index;
            std::vector<uint8_t> chunk_data;
            
            if (MessageParser::parsePointCloudChunk(result.payload, chunk_index, chunk_data)) {
                received_chunks_[chunk_index] = std::move(chunk_data);
                
                // Check if all chunks received
                if (received_chunks_.size() == current_header_->chunk_count) {
                    assemblePointCloud();
                }
            }
            break;
        }
        
        case Protocol::MessageType::POINT_CLOUD_END: {
            if (current_header_ && !received_chunks_.empty()) {
                assemblePointCloud();
            }
            break;
        }
        
        case Protocol::MessageType::STATUS_UPDATE: {
            if (MessageParser::parseStatusUpdate(result.payload, last_status_)) {
                last_update_time_ = std::chrono::steady_clock::now();
                
                if (status_callback_) {
                    status_callback_(last_status_);
                }
            }
            break;
        }
        
        case Protocol::MessageType::ERROR: {
            uint32_t error_code = 0;
            if (result.payload.size() >= sizeof(error_code)) {
                std::memcpy(&error_code, result.payload.data(), sizeof(error_code));
            }
            Logger::instance().error("Server error: code {}", error_code);
            break;
        }
        
        default:
            Logger::instance().warning("Unhandled message type");
            break;
    }
}

void Client::sendMessage(const std::vector<uint8_t>& data) {
    if (!connected_) return;
    
    bool write_in_progress = false;
    
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        write_in_progress = !write_queue_.empty();
        write_queue_.push(data);
    }
    
    if (!write_in_progress) {
        handleWrite();
    }
}

void Client::assemblePointCloud() {
    if (!current_header_ || received_chunks_.empty()) return;
    
    Logger::instance().info("Assembling point cloud from {} chunks", received_chunks_.size());
    
    auto cloud = std::make_shared<PointCloud>();
    cloud->reserve(current_header_->total_points);
    
    // Assemble points from chunks
    for (uint32_t i = 0; i < current_header_->chunk_count; ++i) {
        auto it = received_chunks_.find(i);
        if (it != received_chunks_.end()) {
            const auto& chunk_data = it->second;
            
            // Deserialize points
            size_t num_points = chunk_data.size() / sizeof(Point);
            const Point* points = reinterpret_cast<const Point*>(chunk_data.data());
            
            for (size_t j = 0; j < num_points; ++j) {
                cloud->addPoint(points[j]);
            }
        }
    }
    
    Logger::instance().info("Assembled point cloud with {} points", cloud->size());
    
    // Notify callback
    if (point_cloud_callback_) {
        point_cloud_callback_(cloud);
    }
    
    // Clean up
    current_header_.reset();
    received_chunks_.clear();
}

} // namespace CloudStream 