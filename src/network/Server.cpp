#include "network/Server.h"
#include "utils/Logger.h"
#include "utils/Timer.h"
#include <boost/asio/ip/tcp.hpp>
#include <boost/bind/bind.hpp>
#include <fstream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace CloudStream {

using boost::asio::ip::tcp;

// ClientSession implementation
Server::ClientSession::ClientSession(tcp::socket socket, Server* server)
    : socket_(std::move(socket))
    , server_(server)
    , read_buffer_(Protocol::MAX_PACKET_SIZE) {
    
    // Generate unique client ID
    auto endpoint = socket_.remote_endpoint();
    client_id_ = endpoint.address().to_string() + ":" + std::to_string(endpoint.port());
}

Server::ClientSession::~ClientSession() {
    stop();
}

void Server::ClientSession::start() {
    Logger::instance().info("Client session started: {}", client_id_);
    
    // Start reading
    handleRead();
}

void Server::ClientSession::stop() {
    if (!active_) return;
    
    active_ = false;
    
    boost::system::error_code ec;
    socket_.shutdown(tcp::socket::shutdown_both, ec);
    socket_.close(ec);
    
    Logger::instance().info("Client session stopped: {}", client_id_);
}

void Server::ClientSession::sendPointCloud(PointCloud::Ptr cloud) {
    if (!cloud || !active_) return;
    
    // Build and send point cloud header
    Protocol::PointCloudHeader header{};
    header.total_points = static_cast<uint32_t>(cloud->size());
    header.chunk_count = (header.total_points + 1000 - 1) / 1000; // 1000 points per chunk
    header.compression = Protocol::CompressionType::NONE;
    header.has_colors = 1;
    header.has_normals = 1;
    header.has_intensity = 1;
    header.point_stride = sizeof(Point);
    
    auto bbox = cloud->getBoundingBox();
    header.bounding_box_min[0] = bbox.min.x;
    header.bounding_box_min[1] = bbox.min.y;
    header.bounding_box_min[2] = bbox.min.z;
    header.bounding_box_max[0] = bbox.max.x;
    header.bounding_box_max[1] = bbox.max.y;
    header.bounding_box_max[2] = bbox.max.z;
    
    strncpy(header.description, "CloudStream Point Cloud", sizeof(header.description) - 1);
    
    auto header_msg = message_builder_.buildPointCloudHeader(header);
    sendMessage(header_msg);
    
    // Send point cloud chunks
    const size_t points_per_chunk = 1000;
    std::vector<uint8_t> chunk_data;
    chunk_data.reserve(points_per_chunk * sizeof(Point));
    
    for (uint32_t chunk_idx = 0; chunk_idx < header.chunk_count; ++chunk_idx) {
        chunk_data.clear();
        
        size_t start_idx = chunk_idx * points_per_chunk;
        size_t end_idx = std::min(start_idx + points_per_chunk, cloud->size());
        
        for (size_t i = start_idx; i < end_idx; ++i) {
            const Point& point = (*cloud)[i];
            chunk_data.insert(chunk_data.end(), 
                            reinterpret_cast<const uint8_t*>(&point),
                            reinterpret_cast<const uint8_t*>(&point) + sizeof(Point));
        }
        
        auto chunk_msg = message_builder_.buildPointCloudChunk(chunk_idx, chunk_data, false);
        sendMessage(chunk_msg);
    }
    
    // Send end marker
    auto end_msg = message_builder_.createMessage(Protocol::MessageType::POINT_CLOUD_END, {});
    sendMessage(end_msg);
}

void Server::ClientSession::handleRead() {
    if (!active_) return;
    
    auto self = shared_from_this();
    
    socket_.async_read_some(
        boost::asio::buffer(read_buffer_),
        [this, self](boost::system::error_code ec, size_t bytes_transferred) {
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
                    Logger::instance().info("Client disconnected: {} ({})", 
                                          client_id_, ec.message());
                }
                server_->removeClient(client_id_);
            }
        });
}

void Server::ClientSession::handleWrite() {
    if (!active_ || write_queue_.empty()) return;
    
    auto self = shared_from_this();
    const auto& data = write_queue_.front();
    
    boost::asio::async_write(
        socket_,
        boost::asio::buffer(data),
        [this, self](boost::system::error_code ec, size_t /*bytes_transferred*/) {
            if (!ec) {
                std::lock_guard<std::mutex> lock(write_mutex_);
                write_queue_.pop();
                
                if (!write_queue_.empty()) {
                    handleWrite();
                }
            } else {
                Logger::instance().error("Write error for client {}: {}", 
                                       client_id_, ec.message());
                server_->removeClient(client_id_);
            }
        });
}

void Server::ClientSession::processMessage(const MessageParser::ParseResult& result) {
    switch (result.type) {
        case Protocol::MessageType::HANDSHAKE: {
            std::string client_id;
            if (MessageParser::parseHandshake(result.payload, client_id)) {
                Logger::instance().info("Handshake from client: {}", client_id);
                
                // Send acknowledgment
                auto ack = message_builder_.buildHandshakeAck(true, "Welcome to CloudStream");
                sendMessage(ack);
                
                // Send initial point cloud if available
                server_->broadcastPointCloud();
            }
            break;
        }
        
        case Protocol::MessageType::CONTROL_COMMAND: {
            if (!result.payload.empty()) {
                auto cmd = static_cast<Protocol::ControlCommand>(result.payload[0]);
                
                switch (cmd) {
                    case Protocol::ControlCommand::START_STREAM:
                        Logger::instance().info("Client {} requested stream start", client_id_);
                        sendPointCloud(server_->point_cloud_);
                        break;
                        
                    case Protocol::ControlCommand::STOP_STREAM:
                        Logger::instance().info("Client {} requested stream stop", client_id_);
                        break;
                        
                    default:
                        break;
                }
            }
            break;
        }
        
        default:
            Logger::instance().warning("Unhandled message type from client {}", client_id_);
            break;
    }
}

void Server::ClientSession::sendMessage(const std::vector<uint8_t>& data) {
    if (!active_) return;
    
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

// Server implementation
Server::Server(uint16_t port, size_t thread_count)
    : port_(port)
    , thread_pool_(thread_count) {
    
    io_context_ = std::make_unique<boost::asio::io_context>();
    acceptor_ = std::make_unique<tcp::acceptor>(*io_context_, tcp::endpoint(tcp::v4(), port));
}

Server::~Server() {
    stop();
}

void Server::start() {
    if (running_) return;
    
    running_ = true;
    
    Logger::instance().info("Starting server on port {}", port_);
    
    // Start accepting connections
    acceptConnections();
    
    // Start IO threads
    for (size_t i = 0; i < 2; ++i) {
        io_threads_.emplace_back([this]() {
            try {
                io_context_->run();
            } catch (const std::exception& e) {
                Logger::instance().error("Server IO thread error: {}", e.what());
            }
        });
    }
    
    // Start broadcast thread
    broadcast_thread_ = std::thread([this]() {
        while (running_) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // Send status updates to all clients
            Protocol::StatusUpdate status{};
            status.points_sent = 0;
            status.points_received = 0;
            status.bandwidth_mbps = 0.0f;
            status.fps = 30.0f;
            status.dropped_frames = 0;
            status.latency_ms = 10;
            
            auto status_msg = MessageBuilder().buildStatusUpdate(status);
            
            std::lock_guard<std::mutex> lock(clients_mutex_);
            for (auto& [id, client] : clients_) {
                client->sendMessage(status_msg);
            }
        }
    });
    
    Logger::instance().info("Server started successfully");
}

void Server::stop() {
    if (!running_) return;
    
    running_ = false;
    
    Logger::instance().info("Stopping server...");
    
    // Close acceptor
    acceptor_->close();
    
    // Disconnect all clients
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (auto& [id, client] : clients_) {
            client->stop();
        }
        clients_.clear();
    }
    
    // Stop IO context
    io_context_->stop();
    
    // Wait for threads
    for (auto& thread : io_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    if (broadcast_thread_.joinable()) {
        broadcast_thread_.join();
    }
    
    Logger::instance().info("Server stopped");
    
    // Notify shutdown
    shutdown_cv_.notify_all();
}

void Server::setSourceFile(const std::string& path) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    source_file_ = path;
    
    // Load point cloud from file
    if (!path.empty() && path != "demo") {
        // In a real implementation, load from file
        Logger::instance().info("Loading point cloud from: {}", path);
    }
    
    // For demo, generate sample point cloud
    auto cloud = std::make_shared<PointCloud>();
    
    // Generate a sphere of points
    const int num_points = 10000;
    const float radius = 5.0f;
    
    for (int i = 0; i < num_points; ++i) {
        float theta = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
        float phi = static_cast<float>(rand()) / RAND_MAX * M_PI;
        float r = radius * (0.8f + 0.2f * static_cast<float>(rand()) / RAND_MAX);
        
        Point point;
        point.position = glm::vec3(
            r * sin(phi) * cos(theta),
            r * sin(phi) * sin(theta),
            r * cos(phi)
        );
        
        point.normal = glm::normalize(point.position);
        
        // Color based on position
        point.color = glm::vec4(
            (point.position.x + radius) / (2.0f * radius),
            (point.position.y + radius) / (2.0f * radius),
            (point.position.z + radius) / (2.0f * radius),
            1.0f
        );
        
        point.intensity = 1.0f;
        
        cloud->addPoint(point);
    }
    
    setPointCloud(cloud);
}

void Server::setPointCloud(PointCloud::Ptr cloud) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    point_cloud_ = cloud;
    
    if (cloud) {
        Logger::instance().info("Point cloud set with {} points", cloud->size());
        broadcastPointCloud();
    }
}

size_t Server::getClientCount() const {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    return clients_.size();
}

std::vector<std::string> Server::getConnectedClients() const {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    std::vector<std::string> client_ids;
    client_ids.reserve(clients_.size());
    
    for (const auto& [id, client] : clients_) {
        client_ids.push_back(id);
    }
    
    return client_ids;
}

void Server::acceptConnections() {
    auto socket = std::make_shared<tcp::socket>(*io_context_);
    
    acceptor_->async_accept(
        *socket,
        [this, socket](boost::system::error_code ec) {
            handleAccept(socket, ec);
        });
}

void Server::handleAccept(std::shared_ptr<tcp::socket> socket,
                         const boost::system::error_code& error) {
    if (!error && running_) {
        auto session = std::make_shared<ClientSession>(std::move(*socket), this);
        
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            clients_[session->getClientId()] = session;
        }
        
        session->start();
        
        Logger::instance().info("New client connected. Total clients: {}", getClientCount());
        
        // Continue accepting
        acceptConnections();
    } else if (running_) {
        Logger::instance().error("Accept error: {}", error.message());
        
        // Retry after a delay
        std::this_thread::sleep_for(std::chrono::seconds(1));
        acceptConnections();
    }
}

void Server::broadcastPointCloud() {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    if (!point_cloud_) return;
    
    std::lock_guard<std::mutex> clients_lock(clients_mutex_);
    for (auto& [id, client] : clients_) {
        thread_pool_.enqueue([client, cloud = point_cloud_]() {
            client->sendPointCloud(cloud);
        });
    }
}

void Server::removeClient(const std::string& client_id) {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.erase(client_id);
    
    Logger::instance().info("Client removed: {}. Total clients: {}", 
                          client_id, clients_.size());
}

} // namespace CloudStream 