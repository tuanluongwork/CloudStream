#include <iostream>
#include <thread>
#include <chrono>
#include "network/Client.h"
#include "utils/Logger.h"

using namespace CloudStream;

int main(int argc, char* argv[]) {
    // Initialize logger
    Logger::instance().setLevel(Logger::Level::Info);
    
    // Parse command line arguments
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <host> <port>" << std::endl;
        return 1;
    }
    
    std::string host = argv[1];
    uint16_t port = static_cast<uint16_t>(std::stoi(argv[2]));
    
    // Create client
    Client client;
    
    // Set up callbacks
    client.setPointCloudCallback([](PointCloud::Ptr cloud) {
        Logger::instance().info("Received point cloud with {} points", cloud->size());
        
        // Process the point cloud here
        auto stats = cloud->computeStatistics();
        Logger::instance().info("Centroid: ({}, {}, {})", 
                              stats.centroid.x, stats.centroid.y, stats.centroid.z);
        Logger::instance().info("Bounding box size: ({}, {}, {})",
                              stats.bounds.size().x, stats.bounds.size().y, stats.bounds.size().z);
    });
    
    client.setStatusCallback([](const Protocol::StatusUpdate& status) {
        Logger::instance().info("Status - FPS: {:.1f}, Bandwidth: {:.2f} MB/s, Latency: {} ms",
                              status.fps, status.bandwidth_mbps, status.latency_ms);
    });
    
    // Connect to server
    Logger::instance().info("Connecting to {}:{}...", host, port);
    
    bool connected = false;
    client.connect(host, port, [&connected](bool success, const std::string& error) {
        if (success) {
            Logger::instance().info("Connected successfully!");
            connected = true;
        } else {
            Logger::instance().error("Connection failed: {}", error);
        }
    });
    
    // Wait for connection
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    if (!connected) {
        return 1;
    }
    
    // Request stream
    client.requestStream();
    
    // Run for a while
    Logger::instance().info("Receiving data... Press Ctrl+C to stop.");
    
    try {
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // You could add interactive commands here
            // For example, pause/resume stream, change quality, etc.
        }
    } catch (const std::exception& e) {
        Logger::instance().error("Error: {}", e.what());
    }
    
    // Disconnect
    client.disconnect();
    Logger::instance().info("Disconnected.");
    
    return 0;
} 