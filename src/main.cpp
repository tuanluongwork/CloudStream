#include <QApplication>
#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <thread>
#include "visualization/MainWindow.h"
#include "network/Server.h"
#include "network/Client.h"
#include "utils/Logger.h"

namespace po = boost::program_options;
using namespace CloudStream;

enum class Mode {
    Standalone,
    Server,
    Client
};

struct Config {
    Mode mode{Mode::Standalone};
    std::string host{"localhost"};
    uint16_t port{8080};
    std::string file_path;
    std::string source_path;
    bool verbose{false};
    size_t thread_count{std::thread::hardware_concurrency()};
};

Config parseCommandLine(int argc, char* argv[]) {
    Config config;
    
    po::options_description desc("CloudStream - Real-time Point Cloud Streaming\n\nOptions");
    desc.add_options()
        ("help,h", "Show this help message")
        ("server", "Run in server mode (streaming)")
        ("client", "Run in client mode (visualization)")
        ("file,f", po::value<std::string>(&config.file_path), 
         "Point cloud file to load (standalone mode)")
        ("source,s", po::value<std::string>(&config.source_path), 
         "Source file for streaming (server mode)")
        ("host", po::value<std::string>(&config.host)->default_value("localhost"), 
         "Server hostname (client mode)")
        ("port,p", po::value<uint16_t>(&config.port)->default_value(8080), 
         "Network port")
        ("threads,t", po::value<size_t>(&config.thread_count)->default_value(config.thread_count),
         "Number of worker threads")
        ("verbose,v", po::bool_switch(&config.verbose), "Enable verbose logging");
    
    po::variables_map vm;
    
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
        
        if (vm.count("help")) {
            std::cout << desc << std::endl;
            std::exit(0);
        }
        
        // Determine mode
        if (vm.count("server")) {
            config.mode = Mode::Server;
            if (config.source_path.empty() && config.file_path.empty()) {
                throw std::runtime_error("Server mode requires --source or --file");
            }
            if (config.source_path.empty()) {
                config.source_path = config.file_path;
            }
        } else if (vm.count("client")) {
            config.mode = Mode::Client;
        } else {
            config.mode = Mode::Standalone;
            if (config.file_path.empty()) {
                // Generate demo point cloud if no file specified
                config.file_path = "demo";
            }
        }
        
    } catch (const po::error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << desc << std::endl;
        std::exit(1);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::exit(1);
    }
    
    return config;
}

int runStandaloneMode(int argc, char* argv[], const Config& config) {
    QApplication app(argc, argv);
    
    Logger::instance().setLevel(config.verbose ? Logger::Level::Debug : Logger::Level::Info);
    Logger::instance().info("Starting CloudStream in standalone mode");
    
    MainWindow window;
    window.setWindowTitle("CloudStream - Point Cloud Viewer");
    
    if (config.file_path == "demo") {
        window.loadDemoPointCloud();
    } else {
        window.loadPointCloudFile(QString::fromStdString(config.file_path));
    }
    
    window.show();
    
    return app.exec();
}

int runServerMode(const Config& config) {
    Logger::instance().setLevel(config.verbose ? Logger::Level::Debug : Logger::Level::Info);
    Logger::instance().info("Starting CloudStream server on port {}", config.port);
    
    try {
        Server server(config.port, config.thread_count);
        server.setSourceFile(config.source_path);
        
        server.start();
        
        Logger::instance().info("Server running. Press Ctrl+C to stop.");
        
        // Keep server running
        std::unique_lock<std::mutex> lock(server.getShutdownMutex());
        server.getShutdownCondition().wait(lock);
        
    } catch (const std::exception& e) {
        Logger::instance().error("Server error: {}", e.what());
        return 1;
    }
    
    return 0;
}

int runClientMode(int argc, char* argv[], const Config& config) {
    QApplication app(argc, argv);
    
    Logger::instance().setLevel(config.verbose ? Logger::Level::Debug : Logger::Level::Info);
    Logger::instance().info("Starting CloudStream client, connecting to {}:{}", 
                           config.host, config.port);
    
    MainWindow window;
    window.setWindowTitle("CloudStream - Network Client");
    
    // Connect to server
    if (!window.connectToServer(QString::fromStdString(config.host), config.port)) {
        Logger::instance().error("Failed to connect to server");
        return 1;
    }
    
    window.show();
    
    return app.exec();
}

int main(int argc, char* argv[]) {
    try {
        Config config = parseCommandLine(argc, argv);
        
        switch (config.mode) {
            case Mode::Standalone:
                return runStandaloneMode(argc, argv, config);
                
            case Mode::Server:
                return runServerMode(config);
                
            case Mode::Client:
                return runClientMode(argc, argv, config);
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 