#pragma once

#include <boost/asio.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <mutex>
#include <condition_variable>
#include "network/StreamProtocol.h"
#include "core/PointCloud.h"
#include "utils/ThreadPool.h"

namespace CloudStream {

class Server {
public:
    Server(uint16_t port, size_t thread_count = 4);
    ~Server();
    
    // Server control
    void start();
    void stop();
    bool isRunning() const { return running_; }
    
    // Data source
    void setSourceFile(const std::string& path);
    void setPointCloud(PointCloud::Ptr cloud);
    
    // Client management
    size_t getClientCount() const;
    std::vector<std::string> getConnectedClients() const;
    
    // Shutdown coordination
    std::mutex& getShutdownMutex() { return shutdown_mutex_; }
    std::condition_variable& getShutdownCondition() { return shutdown_cv_; }
    
private:
    class ClientSession : public std::enable_shared_from_this<ClientSession> {
    public:
        ClientSession(boost::asio::ip::tcp::socket socket, Server* server);
        ~ClientSession();
        
        void start();
        void stop();
        void sendPointCloud(PointCloud::Ptr cloud);
        
        const std::string& getClientId() const { return client_id_; }
        
    private:
        void handleRead();
        void handleWrite();
        void processMessage(const MessageParser::ParseResult& result);
        
        boost::asio::ip::tcp::socket socket_;
        Server* server_;
        std::string client_id_;
        MessageBuilder message_builder_;
        MessageParser message_parser_;
        std::vector<uint8_t> read_buffer_;
        std::queue<std::vector<uint8_t>> write_queue_;
        std::mutex write_mutex_;
        std::atomic<bool> active_{true};
    };
    
    void acceptConnections();
    void handleAccept(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
                     const boost::system::error_code& error);
    void broadcastPointCloud();
    void removeClient(const std::string& client_id);
    
    // Network
    uint16_t port_;
    std::unique_ptr<boost::asio::io_context> io_context_;
    std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptor_;
    std::vector<std::thread> io_threads_;
    
    // Clients
    std::unordered_map<std::string, std::shared_ptr<ClientSession>> clients_;
    mutable std::mutex clients_mutex_;
    
    // Data
    PointCloud::Ptr point_cloud_;
    std::string source_file_;
    std::mutex data_mutex_;
    
    // Threading
    ThreadPool thread_pool_;
    std::atomic<bool> running_{false};
    std::thread broadcast_thread_;
    
    // Shutdown coordination
    std::mutex shutdown_mutex_;
    std::condition_variable shutdown_cv_;
};

} // namespace CloudStream 